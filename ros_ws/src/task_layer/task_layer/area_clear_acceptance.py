#!/usr/bin/env python3
"""Acceptance harness for the area_clear check (plan C5 / P1-4 gates).

Drives the robot (own namespace) to each requested area center, stops,
collects consecutive scans and runs AreaClearChecker; one JSON line per
stop, CSV archive, summary at the end. Peer robots' amcl poses come from
robots.yaml for the peer filter.

  ros2 run task_layer area_clear_acceptance.py --ros-args -r __ns:=/tb3 \
      -p areas:='central_hall,storage_area' -p rounds:=3
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
import yaml

from task_layer.area_clear_check import AreaClearChecker, pose_uncertain

DEFAULT_AREAS = ('entrance_lobby,main_corridor,central_hall,north_hall,'
                 'east_hall,server_room,storage_area,utility_area')


def share_path(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('task_layer')) / name)


def yaw_of(quat) -> float:
    return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


class AreaClearAcceptance(Node):
    def __init__(self):
        super().__init__('area_clear_acceptance')
        self.declare_parameter('areas', DEFAULT_AREAS)
        # Extra/override stops as 'label:x:y,label:x:y' — areas longer than
        # the lidar diameter (east_hall, 7.2 m vs 2x3.5 m range) leave AMCL
        # unconstrained along their axis at the center; observe such areas
        # from points near a wall instead.
        self.declare_parameter('stops', '')
        self.declare_parameter('rounds', 1)
        self.declare_parameter('frames', 5)
        self.declare_parameter('settle_sec', 1.0)
        self.declare_parameter('nav_timeout_sec', 180.0)
        # AMCL covariance is conservative: measured std ~0.24 while true
        # error was 0.09 m. The gate only catches gross divergence.
        self.declare_parameter('max_pose_std', 0.35)
        self.declare_parameter('min_alignment', 0.80)
        self.declare_parameter('map_yaml', share_path('maps/tb3_map.yaml'))
        self.declare_parameter('robots_yaml', share_path('config/robots.yaml'))
        self.declare_parameter('world_model_path', share_path('config/world_model.yaml'))
        self.declare_parameter('out_csv', '')
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.ns = self.get_namespace().strip('/')
        with open(self.get_parameter('world_model_path').value, encoding='utf-8') as f:
            self.world_model = yaml.safe_load(f)
        self.checker = AreaClearChecker(self.get_parameter('map_yaml').value)

        self._scans: list[LaserScan] = []
        self._collecting = False
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._own_pose = None  # (x, y, yaw, cov)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._on_own_pose, latched)
        self._peer_poses: dict[str, tuple] = {}
        with open(self.get_parameter('robots_yaml').value, encoding='utf-8') as f:
            robots = yaml.safe_load(f)['robots']
        for peer, info in robots.items():
            if peer == self.ns:
                continue
            self.create_subscription(
                PoseWithCovarianceStamped, info['amcl_pose_topic'],
                lambda msg, peer=peer: self._peer_poses.__setitem__(
                    peer, (msg.pose.pose.position.x, msg.pose.pose.position.y)),
                latched)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd = self.create_publisher(Twist, 'cmd_vel', 10)

    def relocalize_spin(self, duration_sec: float = 14.0, w: float = 0.5):
        """AMCL only updates on motion (update_min_d/a): a robot that
        arrives with an inflated covariance keeps it forever while parked.
        One slow full turn at the stop feeds AMCL fresh geometry."""
        msg = Twist()
        msg.angular.z = w
        end = time.time() + duration_sec
        while time.time() < end:
            self._cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._cmd.publish(Twist())

    def _on_scan(self, msg):
        if self._collecting:
            self._scans.append(msg)

    def _on_own_pose(self, msg):
        p = msg.pose.pose
        self._own_pose = (p.position.x, p.position.y, yaw_of(p.orientation),
                          list(msg.pose.covariance))

    def goto(self, x: float, y: float) -> str:
        if not self._nav.wait_for_server(timeout_sec=10.0):
            return 'server_unavailable'
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        send = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return 'rejected'
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result, timeout_sec=float(self.get_parameter('nav_timeout_sec').value))
        res = result.result()
        return 'succeeded' if res is not None and res.status == 4 else 'nav_failed'

    def collect_scans(self, frames: int) -> list[LaserScan]:
        self._scans = []
        self._collecting = True
        deadline = time.time() + 20.0
        while len(self._scans) < frames and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self._collecting = False
        return self._scans[:frames]

    def run(self) -> int:
        areas = [a.strip() for a in
                 str(self.get_parameter('areas').value).split(',') if a.strip()]
        rounds = int(self.get_parameter('rounds').value)
        frames = int(self.get_parameter('frames').value)
        settle = float(self.get_parameter('settle_sec').value)
        max_std = float(self.get_parameter('max_pose_std').value)
        out_csv = (self.get_parameter('out_csv').value
                   or f'/tmp/area_clear_{self.ns or "root"}.csv')

        stops_spec = []
        for item in str(self.get_parameter('stops').value).split(','):
            if item.strip():
                label, sx, sy = item.strip().split(':')
                stops_spec.append((label, float(sx), float(sy)))
        plan = [(a, float(self.world_model['areas'][a]['center'][0]),
                 float(self.world_model['areas'][a]['center'][1])) for a in areas]
        plan += stops_spec

        rows = ['ns,round,area,status,n_anomalies,evidence_cells,pose_std,anomalies']
        total_anomalies, stops = 0, 0
        for rnd in range(1, rounds + 1):
            for area_key, cx, cy in plan:
                nav = self.goto(cx, cy)
                if nav != 'succeeded':
                    self.get_logger().error(f'{area_key}: {nav}')
                    rows.append(f'{self.ns},{rnd},{area_key},{nav},,,,')
                    continue
                min_align = float(self.get_parameter('min_alignment').value)
                for attempt in (1, 2):
                    settle_until = time.time() + settle
                    while time.time() < settle_until:
                        rclpy.spin_once(self, timeout_sec=0.05)
                    scans = self.collect_scans(frames)
                    if len(scans) < frames or self._own_pose is None:
                        rows.append(f'{self.ns},{rnd},{area_key},no_data,,,,')
                        break
                    x, y, yaw, cov = self._own_pose
                    std = round(math.sqrt(max(cov[0], cov[7], 0.0)), 3)
                    uncertain = pose_uncertain(cov, max_std)
                    align = 0.0 if uncertain else sum(
                        self.checker.alignment_ratio(s, (x, y, yaw))
                        for s in scans) / len(scans)
                    if uncertain or align < min_align:
                        if attempt == 1:
                            self.get_logger().warn(
                                f'{area_key}: pose_std={std} align={align:.2f}'
                                ' — spinning in place to relocalize')
                            self.relocalize_spin()
                            continue
                        if uncertain:
                            rows.append(f'{self.ns},{rnd},{area_key},'
                                        f'skipped_uncertain_pose,,,{std},')
                        else:
                            self.get_logger().warn(
                                f'{area_key}: scan-map alignment {align:.2f}, skipping')
                            rows.append(f'{self.ns},{rnd},{area_key},'
                                        f'skipped_misaligned,,,{std},{align:.2f}')
                        break
                    report = self.checker.check(scans, (x, y, yaw),
                                                peers=list(self._peer_poses.values()))
                    report['alignment'] = round(align, 3)
                    stops += 1
                    n = len(report['anomalies'])
                    total_anomalies += n
                    line = {'ns': self.ns, 'round': rnd, 'area': area_key,
                            'n_anomalies': n, **report}
                    print(json.dumps(line), flush=True)
                    payload = json.dumps(report['anomalies']).replace(',', ';')
                    rows.append(f"{self.ns},{rnd},{area_key},checked,{n},"
                                f"{report['evidence_cells']},{std},{payload}")
                    break
        Path(out_csv).write_text('\n'.join(rows) + '\n', encoding='utf-8')
        print(f'SUMMARY ns={self.ns} stops={stops} anomalies={total_anomalies} '
              f'csv={out_csv}', flush=True)
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = AreaClearAcceptance()
    try:
        code = node.run()
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
