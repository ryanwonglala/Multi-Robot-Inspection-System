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
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
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
            registry = yaml.safe_load(f)
        self.robots = registry['robots']
        self.home_gate = registry.get('home_gate') or {}
        with open(self.get_parameter('world_model_path').value, encoding='utf-8') as f:
            self.world_model = yaml.safe_load(f)

        self._plan_clients: dict = {}   # ns -> ActionClient | False (unavailable)
        self._cost_cache: dict = {}     # (ns, start_xy, area) -> meters

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
                home = info.get('home_pose') or {}
                if {'x', 'y'} <= home.keys():
                    self.robot_poses[ns] = (float(home['x']), float(home['y']))
                else:
                    self.robot_poses[ns] = self.area_center(info['home_area'])
                self.get_logger().warn(
                    f'{ns}: no amcl_pose received, assuming home position')

    def path_length(self, ns: str, start: tuple, goal: tuple) -> float | None:
        """Planner-reported path length in meters, or None when the robot's
        planner is unavailable or finds no path."""
        client = self._plan_clients.get(ns)
        if client is False:
            return None
        if client is None:
            action_name = (self.robots[ns]['nav_action'].rsplit('/', 1)[0]
                           + '/compute_path_to_pose')
            client = ActionClient(self, ComputePathToPose, action_name)
            if not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    f'{ns}: planner unavailable, using straight-line distances')
                self._plan_clients[ns] = False
                return None
            self._plan_clients[ns] = client
        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = float(start[0])
        goal_msg.start.pose.position.y = float(start[1])
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = float(goal[0])
        goal_msg.goal.pose.position.y = float(goal[1])
        goal_msg.goal.pose.orientation.w = 1.0
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        result = result_future.result()
        if result is None:
            return None
        poses = result.result.path.poses
        if len(poses) < 2:
            return None
        return sum(
            math.hypot(b.pose.position.x - a.pose.position.x,
                       b.pose.position.y - a.pose.position.y)
            for a, b in zip(poses, poses[1:]))

    def travel_cost(self, ns: str, start: tuple, area_key: str) -> float:
        """Real path length from start to the area center (walls count);
        straight-line fallback keeps allocation alive without a planner."""
        key = (ns, start, area_key)
        if key not in self._cost_cache:
            goal = self.area_center(area_key)
            length = self.path_length(ns, start, goal)
            if length is None:
                length = math.hypot(start[0] - goal[0], start[1] - goal[1])
            self._cost_cache[key] = length
        return self._cost_cache[key]

    def allocate(self, route: list[str]) -> dict[str, list[str]]:
        """Cheapest (robot, area) pair first, repeated until the route is
        exhausted. Costs are planner path lengths from each robot's *virtual*
        position (it moves onto an area once assigned), so the split ignores
        the order the operator picked the rooms in and walls between a robot
        and a room count at their detour cost. The quota (ceil(N/robots))
        stops the greedy cascade where one robot that is 'on the way'
        swallows the whole route while the others idle."""
        quota = math.ceil(len(route) / max(len(self.robots), 1))
        cursor = dict(self.robot_poses)
        plan: dict[str, list[str]] = {ns: [] for ns in self.robots}
        remaining = list(route)
        while remaining:
            candidates = [ns for ns in cursor if len(plan[ns]) < quota]
            _, best, area_key = min(
                (self.travel_cost(ns, cursor[ns], area), ns, area)
                for ns in candidates for area in remaining)
            plan[best].append(area_key)
            cursor[best] = self.area_center(area_key)
            remaining.remove(area_key)
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
        report_root = Path(self.get_parameter('report_dir').value)
        procs = {}
        for ns, areas in plan.items():
            if not areas:
                continue
            report_dir = report_root / ns
            report_dir.mkdir(parents=True, exist_ok=True)
            # Runners never return home on their own: the allocator owns the
            # end-of-mission return so it can arbitrate the shared doorway
            # into the mother_base bay (unmanaged simultaneous returns wedged
            # each other in that opening). See return_all_home().
            command = [
                'ros2', 'run', 'task_layer', 'inspection_runner.py', '--ros-args',
                '-r', f'__ns:=/{ns}',
                '-p', f'use_sim_time:={use_sim_time}',
                '-p', f"route:={','.join(areas)}",
                '-p', 'return_home:=false',
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

        if bool(self.get_parameter('return_home').value):
            results = self.return_all_home(list(procs))
            for ns, ok in results.items():
                if not ok:
                    codes[ns] = codes.get(ns) or 6
        return 0 if all(code == 0 for code in codes.values()) else 5

    def _home_goal(self, ns: str) -> NavigateToPose.Goal:
        home = self.robots[ns]['home_pose']
        yaw = float(home.get('yaw', 0.0))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(home['x'])
        goal.pose.pose.position.y = float(home['y'])
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def _start_home(self, ns: str, state: dict):
        send_future = state['client'].send_goal_async(self._home_goal(ns))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{ns}: return-home goal rejected')
            state['state'] = 'failed'
            return
        state['handle'] = handle
        state['result_future'] = handle.get_result_async()
        state['state'] = 'driving'

    def return_all_home(self, ns_list: list[str],
                        timeout_sec: float = 300.0) -> dict[str, bool]:
        """Concurrent return with a doorway mutex: every robot drives home at
        once, but only one at a time may thread the funnel into the
        mother_base bay (simultaneous unmanaged returns wedged there). A
        robot approaching a held gate is cancelled in place and re-sent once
        the holder has cleared the zone."""
        gate_x = float(self.home_gate.get('x', -1.65))
        gate_y = float(self.home_gate.get('y', -3.3))
        gate_radius = float(self.home_gate.get('radius', 1.0))
        hold_radius = gate_radius + 0.7  # stop before physically entering

        states: dict[str, dict] = {}
        for ns in ns_list:
            home = self.robots[ns].get('home_pose') or {}
            if not {'x', 'y'} <= home.keys():
                self.get_logger().warn(
                    f'{ns}: no home_pose in robots.yaml, skipping return')
                continue
            client = ActionClient(self, NavigateToPose, self.robots[ns]['nav_action'])
            if not client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'{ns}: nav action server unavailable for return')
                states[ns] = {'state': 'failed', 'handle': None}
                continue
            states[ns] = {'client': client, 'state': 'init',
                          'handle': None, 'result_future': None}
            self.get_logger().info(
                f"{ns}: returning home x={home['x']} y={home['y']}")
            self._start_home(ns, states[ns])

        def gate_dist(ns: str) -> float:
            px, py = self.robot_poses.get(ns, (math.inf, math.inf))
            return math.hypot(px - gate_x, py - gate_y)

        def past_gate(ns: str) -> bool:
            # Already closer to its dock than the gate is: it has threaded
            # the funnel and must not be held on the inside (observed: a
            # robot paused pointlessly in the bay right after passing).
            home = self.robots[ns]['home_pose']
            hx, hy = float(home['x']), float(home['y'])
            px, py = self.robot_poses.get(ns, (math.inf, math.inf))
            return math.hypot(px - hx, py - hy) < math.hypot(gate_x - hx, gate_y - hy)

        holder = None
        deadline = time.time() + timeout_sec
        while (time.time() < deadline
               and any(s['state'] in ('driving', 'held') for s in states.values())):
            rclpy.spin_once(self, timeout_sec=0.1)
            for ns, st in states.items():
                if st['state'] == 'driving' and st['result_future'].done():
                    result = st['result_future'].result()
                    if result is not None and result.status == 4:  # SUCCEEDED
                        st['state'] = 'done'
                        self.get_logger().info(f'{ns}: return home succeeded')
                    else:
                        st['state'] = 'failed'
                        status = getattr(result, 'status', 'no result')
                        self.get_logger().error(
                            f'{ns}: return home failed (status {status})')
            # The holder keeps the gate while it is inside and still driving.
            if holder is not None and (states[holder]['state'] != 'driving'
                                       or gate_dist(holder) > gate_radius):
                holder = None
            if holder is None:
                inside = [ns for ns, st in states.items()
                          if st['state'] == 'driving' and gate_dist(ns) <= gate_radius]
                if inside:
                    holder = min(inside, key=gate_dist)
            for ns, st in states.items():
                if ns == holder:
                    continue
                if (holder is not None and st['state'] == 'driving'
                        and gate_dist(ns) <= hold_radius and not past_gate(ns)):
                    st['handle'].cancel_goal_async()
                    st['state'] = 'held'
                    self.get_logger().info(
                        f'{ns}: holding before home gate ({holder} is inside)')
                elif holder is None and st['state'] == 'held':
                    self.get_logger().info(f'{ns}: gate clear, resuming return')
                    self._start_home(ns, st)
        for ns, st in states.items():
            if st['state'] in ('driving', 'held'):
                self.get_logger().error(f'{ns}: return home timed out')
                if st.get('handle') is not None:
                    st['handle'].cancel_goal_async()
        return {ns: st['state'] == 'done' for ns, st in states.items()}


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
