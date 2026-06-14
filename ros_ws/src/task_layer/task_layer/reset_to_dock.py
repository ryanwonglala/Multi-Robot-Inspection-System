#!/usr/bin/env python3
"""Hard-reset every robot to its charging dock (SIMULATION ONLY).

Test aid for when a robot is wedged / recovery-looping / AMCL-drifted and the
operator wants a clean known state without restarting the whole stack. Unlike a
NavigateToPose "return home", this does NOT depend on localisation being
correct -- it teleports in Gazebo and re-seeds AMCL, so it works even when the
robot's RViz pose has drifted.

Per robot (namespace = Gazebo entity name = robots.yaml key), in order:
  1. Teleport the Gazebo model to its dock pose (robots.yaml home_pose).
  2. Re-seed AMCL by publishing <ns>/initialpose at the dock pose.
  3. Clear the global + local costmaps (wipe stale / drift-induced ghost walls).

This is a sim cheat (a real robot cannot teleport); it exists purely to keep
testing fast. The GUI's "Abort & Reset to Dock" button kills the running
inspection first, then runs this.
"""
from __future__ import annotations

import math
from pathlib import Path
import subprocess
import time

from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.node import Node
import yaml


def load_robots() -> dict:
    share = get_package_share_directory('task_layer')
    with (Path(share) / 'config' / 'robots.yaml').open(encoding='utf-8') as f:
        return (yaml.safe_load(f) or {}).get('robots') or {}


class ResetToDock(Node):
    def __init__(self):
        super().__init__('reset_to_dock')
        # Optional filter: reset only these namespaces (comma/space separated).
        # Empty = every robot in robots.yaml.
        self.declare_parameter('robots', '')
        self.declare_parameter('teleport_z', 0.05)
        self.declare_parameter('initialpose_repeat', 8)
        self.declare_parameter('clear_costmap_timeout_sec', 5.0)

    def selected_namespaces(self, registry: dict) -> list[str]:
        raw = str(self.get_parameter('robots').value or '').replace(',', ' ').split()
        if not raw:
            return list(registry.keys())
        return [ns for ns in raw if ns in registry]

    def cancel_nav_goals(self, ns: str):
        """Cancel ALL active goals on the robot's NavigateToPose server. This is
        the crucial step: Nav2's bt_navigator keeps executing the last goal
        independently of the runner that sent it -- killing the runner does NOT
        stop the robot, so without this the robot drives off again right after
        being teleported home. A default CancelGoal request (zero goal_id + zero
        stamp) cancels every goal."""
        srv = f'/{ns}/navigate_to_pose/_action/cancel_goal'
        client = self.create_client(CancelGoal, srv)
        timeout = float(self.get_parameter('clear_costmap_timeout_sec').value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'{ns}: {srv} unavailable, skipping cancel')
            self.destroy_client(client)
            return
        future = client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.done() and future.result() is not None:
            n = len(future.result().goals_canceling)
            self.get_logger().info(f'{ns}: cancelled {n} active nav goal(s)')
        else:
            self.get_logger().warn(f'{ns}: cancel_goal call timed out')
        self.destroy_client(client)

    def teleport(self, ns: str, x: float, y: float, yaw: float) -> bool:
        """Move the Gazebo model <ns> to the dock pose via the gz CLI (talks to
        gzserver directly; no gazebo_ros plugin dependency)."""
        z = float(self.get_parameter('teleport_z').value)
        cmd = ['gz', 'model', '-m', ns,
               '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', f'{z:.4f}',
               '-R', '0', '-P', '0', '-Y', f'{yaw:.4f}']
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().error(f'{ns}: gz teleport failed: {exc}')
            return False
        if result.returncode != 0:
            self.get_logger().error(
                f'{ns}: gz teleport rc={result.returncode} {result.stderr.strip()}')
            return False
        self.get_logger().info(f'{ns}: teleported to dock ({x:.2f}, {y:.2f}, {yaw:.2f})')
        return True

    def reseed_amcl(self, ns: str, x: float, y: float, yaw: float):
        """Publish <ns>/initialpose a few times so AMCL re-localises at the
        dock (covariance matched to set_initial_pose_node.py)."""
        topic = f'/{ns}/initialpose'
        pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        cov = [0.0] * 36
        cov[0] = 0.25 * 0.25     # x
        cov[7] = 0.25 * 0.25     # y
        cov[35] = (math.pi / 12.0) ** 2  # yaw
        msg.pose.covariance = cov
        repeat = int(self.get_parameter('initialpose_repeat').value)
        for _ in range(max(1, repeat)):
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)
        self.get_logger().info(f'{ns}: re-seeded AMCL at dock')
        self.destroy_publisher(pub)

    def clear_costmaps(self, ns: str):
        timeout = float(self.get_parameter('clear_costmap_timeout_sec').value)
        for scope in ('global', 'local'):
            srv = f'/{ns}/{scope}_costmap/clear_entirely_{scope}_costmap'
            client = self.create_client(ClearEntireCostmap, srv)
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().warn(f'{ns}: {srv} unavailable, skipping clear')
                self.destroy_client(client)
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if future.done():
                self.get_logger().info(f'{ns}: cleared {scope} costmap')
            else:
                self.get_logger().warn(f'{ns}: {scope} costmap clear timed out')
            self.destroy_client(client)

    def run(self) -> int:
        registry = load_robots()
        namespaces = self.selected_namespaces(registry)
        if not namespaces:
            self.get_logger().error('no robots to reset (robots.yaml empty?)')
            return 1
        ok = True
        for ns in namespaces:
            home = (registry.get(ns) or {}).get('home_pose') or {}
            if not {'x', 'y'} <= home.keys():
                self.get_logger().warn(f'{ns}: no home_pose, skipping')
                continue
            x, y = float(home['x']), float(home['y'])
            yaw = float(home.get('yaw', 0.0))
            self.get_logger().info(f'--- resetting {ns} to dock ---')
            # Cancel the live nav goal FIRST, or bt_navigator keeps driving the
            # robot to it after we teleport (the bug that made the robot set off
            # again from the dock).
            self.cancel_nav_goals(ns)
            ok = self.teleport(ns, x, y, yaw) and ok
            # Let the teleport settle in the physics step before re-seeding.
            time.sleep(0.4)
            self.reseed_amcl(ns, x, y, yaw)
            self.clear_costmaps(ns)
        self.get_logger().info('reset-to-dock complete')
        return 0 if ok else 5


def main(args=None):
    rclpy.init(args=args)
    node = ResetToDock()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    main()
