#!/usr/bin/env python3
"""Bounded real-robot validation for a patrol viewpoint and stationary scan."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


STATUS = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}

# On the physical Burger, a 1.000 rad Nav2 Spin command produces an odometry
# rotation of approximately 1.047 rad after settling (the desired 60 degrees).
CALIBRATED_SIX_DIRECTION_STEP_RAD = 1.000


def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


class RotationValidation(Node):
    def __init__(self) -> None:
        super().__init__("validate_vp_rotation")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin = ActionClient(self, Spin, "spin")
        self.pose: tuple[float, float, float] | None = None
        self.odom_pose: tuple[float, float, float] | None = None
        self.scan: LaserScan | None = None
        self.local_costmap: Costmap | None = None
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._pose_callback, 10
        )
        self.create_subscription(Odometry, "odom", self._odom_callback, 10)
        self.create_subscription(
            LaserScan, "scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Costmap, "local_costmap/costmap_raw", self._costmap_callback, 10
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation.z, pose.orientation.w),
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.odom_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation.z, pose.orientation.w),
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def _costmap_callback(self, msg: Costmap) -> None:
        self.local_costmap = msg

    def wait_for_pose(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def _wait_bounded(self, handle, timeout: float) -> int:
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and not result_future.done()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if result_future.done():
            return result_future.result().status

        self.get_logger().warning("Motion timeout; requesting cancellation")
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        terminal_deadline = time.monotonic() + 5.0
        while (
            rclpy.ok()
            and not result_future.done()
            and time.monotonic() < terminal_deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            raise RuntimeError("Cancellation was not confirmed terminal")
        return result_future.result().status

    def navigate(self, x: float, y: float, yaw: float, timeout: float) -> int:
        if not self.nav.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("navigate_to_pose action is unavailable")
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        if not send.done() or send.result() is None or not send.result().accepted:
            raise RuntimeError("VP navigation goal was not accepted")
        return self._wait_bounded(send.result(), timeout)

    def spin_once(self, radians: float, timeout: float) -> int:
        if not self.spin.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("spin action is unavailable")
        goal = Spin.Goal()
        goal.target_yaw = radians
        goal.time_allowance = Duration(sec=int(timeout))
        send = self.spin.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        if not send.done() or send.result() is None or not send.result().accepted:
            raise RuntimeError("Spin goal was not accepted")
        return self._wait_bounded(send.result(), timeout + 5.0)

    def diagnose_clearance(self, timeout: float = 5.0) -> None:
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and (
                self.scan is None
                or self.odom_pose is None
                or self.local_costmap is None
            )
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.scan is None or self.odom_pose is None or self.local_costmap is None:
            raise RuntimeError("Timed out waiting for scan/odom/local costmap")

        finite = [
            (distance, self.scan.angle_min + index * self.scan.angle_increment)
            for index, distance in enumerate(self.scan.ranges)
            if math.isfinite(distance)
            and self.scan.range_min <= distance <= self.scan.range_max
        ]
        scan_min = min(finite)
        print(
            f"SCAN_MIN={scan_min[0]:.4f} ANGLE_RAD={scan_min[1]:.4f}",
            flush=True,
        )

        costmap = self.local_costmap
        resolution = costmap.metadata.resolution
        origin_x = costmap.metadata.origin.position.x
        origin_y = costmap.metadata.origin.position.y
        robot_x, robot_y, _ = self.odom_pose
        nearest: dict[int, float] = {}
        within = {0.11: 0, 0.15: 0, 0.22: 0, 0.30: 0}
        for index, cost in enumerate(costmap.data):
            if cost < 253 or cost == 255:
                continue
            cell_x = origin_x + (index % costmap.metadata.size_x + 0.5) * resolution
            cell_y = origin_y + (index // costmap.metadata.size_x + 0.5) * resolution
            distance = math.hypot(cell_x - robot_x, cell_y - robot_y)
            nearest[cost] = min(nearest.get(cost, math.inf), distance)
            for radius in within:
                if distance <= radius:
                    within[radius] += 1
        print(
            f"ODOM_XY=({robot_x:.4f},{robot_y:.4f}) "
            f"NEAREST_COST_CELLS={nearest} WITHIN={within}",
            flush=True,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("nav", "spin-six", "diagnose"))
    parser.add_argument("--nav-timeout", type=float, default=60.0)
    parser.add_argument("--spin-timeout", type=float, default=15.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument(
        "--step-yaw", type=float, default=CALIBRATED_SIX_DIRECTION_STEP_RAD
    )
    args = parser.parse_args()

    rclpy.init()
    node = RotationValidation()
    try:
        if args.mode == "nav":
            status = node.navigate(0.268, -0.249, 2.251, args.nav_timeout)
            node.wait_for_pose()
            print(f"NAV_STATUS={STATUS.get(status, status)} POSE={node.pose}", flush=True)
            return 0 if status == GoalStatus.STATUS_SUCCEEDED else 2

        if args.mode == "diagnose":
            node.diagnose_clearance()
            return 0

        # AMCL only republishes after its configured motion threshold, so a
        # stationary robot may provide no fresh message to a newly started
        # validator. Use the already-validated VP center as the drift baseline;
        # rotation itself will trigger fresh AMCL pose updates.
        node.wait_for_pose()
        start = node.pose if node.pose is not None else (0.268, -0.249, 0.0)
        odom_deadline = time.monotonic() + 5.0
        while node.odom_pose is None and time.monotonic() < odom_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_pose is None:
            raise RuntimeError("No odom pose available before rotation")
        previous_odom_yaw = node.odom_pose[2]
        cumulative_odom = 0.0
        if args.count < 1:
            raise ValueError("--count must be at least 1")
        for index in range(1, args.count + 1):
            status = node.spin_once(args.step_yaw, args.spin_timeout)
            if status != GoalStatus.STATUS_SUCCEEDED:
                print(f"SPIN_{index}_STATUS={STATUS.get(status, status)}", flush=True)
                return 3
            settle_deadline = time.monotonic() + args.settle
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert node.pose is not None
            assert node.odom_pose is not None
            odom_step = math.atan2(
                math.sin(node.odom_pose[2] - previous_odom_yaw),
                math.cos(node.odom_pose[2] - previous_odom_yaw),
            )
            cumulative_odom += odom_step
            previous_odom_yaw = node.odom_pose[2]
            drift = math.hypot(node.pose[0] - start[0], node.pose[1] - start[1])
            vp_error = math.hypot(node.pose[0] - 0.268, node.pose[1] + 0.249)
            print(
                f"SPIN_{index}_STATUS=succeeded POSE={node.pose} "
                f"ODOM_YAW={node.odom_pose[2]:.4f} "
                f"ODOM_STEP={odom_step:.4f} ODOM_TOTAL={cumulative_odom:.4f} "
                f"DRIFT={drift:.4f} VP_ERROR={vp_error:.4f}",
                flush=True,
            )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
