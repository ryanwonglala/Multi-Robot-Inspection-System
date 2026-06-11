#!/usr/bin/env python3
"""Split an inspection route across robots and run one inspection_runner per
robot (subprocess + namespace; becomes an action-client fan-out in v0.4).

Usage:
  ros2 run task_layer task_allocator.py --ros-args \
      -p route:='storage_area,utility_area,server_room,central_hall'

Exit codes: 0 = all robots finished OK, 2 = bad input, 5 = some robot failed.
"""
from __future__ import annotations

import math
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import yaml

from task_layer.report_writer import default_report_dir


def default_share(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('task_layer')) / 'config' / name)


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.declare_parameter('route', '')
        self.declare_parameter('robots_yaml', default_share('robots.yaml'))
        self.declare_parameter('world_model_path', default_share('world_model.yaml'))
        self.declare_parameter('report_dir', default_report_dir())
        self.declare_parameter('pose_wait_sec', 5.0)
        self.declare_parameter('return_home', True)
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        with open(self.get_parameter('robots_yaml').value, encoding='utf-8') as f:
            self.robots = yaml.safe_load(f)['robots']
        with open(self.get_parameter('world_model_path').value, encoding='utf-8') as f:
            self.world_model = yaml.safe_load(f)

        # AMCL latches its last pose (transient_local); a default volatile
        # subscription would never see it for a robot that is standing still.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_poses = {}
        for ns, info in self.robots.items():
            self.create_subscription(
                PoseWithCovarianceStamped, info['amcl_pose_topic'],
                lambda msg, ns=ns: self.robot_poses.__setitem__(
                    ns, (msg.pose.pose.position.x, msg.pose.pose.position.y)),
                latched)

    def area_center(self, area_key: str) -> tuple[float, float]:
        center = self.world_model['areas'][area_key]['center']
        return float(center[0]), float(center[1])

    def wait_for_poses(self):
        deadline = time.time() + float(self.get_parameter('pose_wait_sec').value)
        while time.time() < deadline and len(self.robot_poses) < len(self.robots):
            rclpy.spin_once(self, timeout_sec=0.1)
        # Robots without a live amcl_pose fall back to their home area center.
        for ns, info in self.robots.items():
            if ns not in self.robot_poses:
                self.robot_poses[ns] = self.area_center(info['home_area'])
                self.get_logger().warn(
                    f'{ns}: no amcl_pose received, using {info["home_area"]} center')

    def allocate(self, route: list[str]) -> dict[str, list[str]]:
        """Greedy with a per-robot quota: each area goes to the robot whose
        *virtual* position is closest to the area center; that robot's
        virtual position then moves there. The quota (ceil(N/robots)) stops
        the greedy cascade where one robot that is 'on the way' swallows the
        whole route while the others idle."""
        quota = math.ceil(len(route) / max(len(self.robots), 1))
        cursor = dict(self.robot_poses)
        plan: dict[str, list[str]] = {ns: [] for ns in self.robots}
        for area_key in route:
            ax, ay = self.area_center(area_key)
            candidates = [ns for ns in cursor if len(plan[ns]) < quota]
            best = min(candidates, key=lambda ns: math.hypot(
                cursor[ns][0] - ax, cursor[ns][1] - ay))
            plan[best].append(area_key)
            cursor[best] = (ax, ay)
        return plan

    def run_once(self) -> int:
        route = [item.strip() for item in
                 str(self.get_parameter('route').value).replace(';', ',').split(',')
                 if item.strip()]
        if not route:
            self.get_logger().error("Parameter 'route' is required")
            return 2
        unknown = [a for a in route if a not in self.world_model['areas']]
        if unknown:
            self.get_logger().error(f'Unknown areas: {unknown}')
            return 2
        walled = [a for a in route
                  if not self.world_model['areas'][a].get('accessible', True)]
        if walled:
            self.get_logger().error(f'Walled-off areas in route: {walled}')
            return 2

        self.wait_for_poses()
        plan = self.allocate(route)
        for ns, areas in plan.items():
            self.get_logger().info(f'Allocation: {ns} -> {areas or "(idle)"}')

        use_sim_time = str(bool(self.get_parameter('use_sim_time').value)).lower()
        return_home = str(bool(self.get_parameter('return_home').value)).lower()
        report_root = Path(self.get_parameter('report_dir').value)
        procs = {}
        for ns, areas in plan.items():
            if not areas:
                continue
            report_dir = report_root / ns
            report_dir.mkdir(parents=True, exist_ok=True)
            command = [
                'ros2', 'run', 'task_layer', 'inspection_runner.py', '--ros-args',
                '-r', f'__ns:=/{ns}',
                '-p', f'use_sim_time:={use_sim_time}',
                '-p', f"route:={','.join(areas)}",
                '-p', f'return_home:={return_home}',
                '-p', f'report_dir:={report_dir}',
            ]
            log_file = open(report_dir / 'allocator_run.log', 'w', encoding='utf-8')
            procs[ns] = (subprocess.Popen(
                command, stdout=log_file, stderr=subprocess.STDOUT, text=True), log_file)
            self.get_logger().info(f'{ns}: inspecting {areas}')

        codes = {}
        for ns, (process, log_file) in procs.items():
            codes[ns] = process.wait()
            log_file.close()
            self.get_logger().info(f'{ns}: finished with code {codes[ns]}')
        return 0 if all(code == 0 for code in codes.values()) else 5


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    try:
        code = node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
