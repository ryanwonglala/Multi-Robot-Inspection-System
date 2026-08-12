#!/usr/bin/env python3
"""Guarded short reverse using odometry, heading hold, and rear lidar."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class BackupController(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("backup_odom_lidar")
        self.args = args
        self.target_heading = math.radians(args.target_heading_deg)
        self.create_subscription(
            Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, args.battery_topic, self.on_battery, 10)
        self.timer = self.create_timer(0.1, self.control_tick)

        self.cmd_pub = None
        self.started_at = time.monotonic()
        self.motion_started_at = None
        self.state = "preflight"
        self.done = False
        self.outcome = "running"
        self.reason = ""
        self.odom_at = None
        self.scan_at = None
        self.battery_at = None
        self.odom_xy = None
        self.initial_xy = None
        self.yaw = None
        self.rear_median = None
        self.battery_v = None
        self.battery_present = False
        self.battery_min_v = None
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.trace = []

    def on_odom(self, message: Odometry) -> None:
        self.odom_at = time.monotonic()
        self.odom_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y))
        self.yaw = yaw_from_odom(message)

    def on_scan(self, message: LaserScan) -> None:
        values = []
        center = math.pi
        half_width = math.radians(self.args.rear_half_width_deg)
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(wrap(angle - center)) > half_width:
                continue
            if math.isfinite(value) and message.range_min <= value <= message.range_max:
                values.append(float(value))
        if len(values) >= self.args.min_rear_points:
            self.rear_median = statistics.median(values)
            self.scan_at = time.monotonic()

    def on_battery(self, message: BatteryState) -> None:
        self.battery_at = time.monotonic()
        self.battery_v = float(message.voltage)
        self.battery_present = bool(message.present)
        if self.battery_min_v is None or self.battery_v < self.battery_min_v:
            self.battery_min_v = self.battery_v

    def progress_m(self) -> float | None:
        if self.initial_xy is None or self.odom_xy is None:
            return None
        dx = self.odom_xy[0] - self.initial_xy[0]
        dy = self.odom_xy[1] - self.initial_xy[1]
        forward_projection = (
            dx * math.cos(self.target_heading) +
            dy * math.sin(self.target_heading))
        return -forward_projection

    def publish_zero(self) -> None:
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())
        self.previous_linear = 0.0
        self.previous_angular = 0.0

    def finish(self, outcome: str, reason: str) -> None:
        if self.done:
            return
        self.publish_zero()
        self.outcome = outcome
        self.reason = reason
        self.state = "done"
        self.done = True

    def preflight_tick(self, now: float) -> None:
        if now - self.started_at > self.args.preflight_timeout:
            self.finish("aborted", "preflight_timeout")
            return
        if any(value is None for value in (
                self.odom_at, self.scan_at, self.battery_at,
                self.odom_xy, self.yaw, self.rear_median, self.battery_v)):
            return
        if max(now - self.odom_at, now - self.scan_at, now - self.battery_at) > 1.0:
            return
        if not self.battery_present or self.battery_v < self.args.min_voltage:
            self.finish("aborted", "battery_not_safe")
            return
        if self.rear_median <= self.args.target_rear_median_m:
            self.finish("aborted", "already_at_or_beyond_lidar_target")
            return
        existing = self.get_publishers_info_by_topic(self.args.cmd_topic)
        if existing:
            names = ",".join(sorted({item.node_name for item in existing}))
            self.finish("aborted", f"existing_cmd_vel_publishers:{names}")
            return
        self.cmd_pub = self.create_publisher(Twist, self.args.cmd_topic, 10)
        self.initial_xy = self.odom_xy
        self.motion_started_at = now
        self.state = "running"

    def running_tick(self, now: float) -> None:
        if now - self.motion_started_at > self.args.max_runtime:
            self.finish("aborted", "motion_timeout")
            return
        if any(value is None for value in (
                self.odom_at, self.scan_at, self.battery_at,
                self.yaw, self.rear_median, self.battery_v)):
            self.finish("aborted", "feedback_missing")
            return
        if now - self.odom_at > 0.5:
            self.finish("aborted", "odom_lost")
            return
        if now - self.scan_at > 0.5:
            self.finish("aborted", "rear_scan_lost")
            return
        if now - self.battery_at > 1.0:
            self.finish("aborted", "battery_feedback_lost")
            return
        if not self.battery_present or self.battery_v < self.args.min_voltage:
            self.finish("aborted", "battery_not_safe")
            return

        progress = self.progress_m()
        remaining = self.args.distance_m - progress
        heading_error = wrap(self.target_heading - self.yaw)
        odom_reached = remaining <= self.args.distance_tolerance_m
        lidar_reached = (
            self.rear_median <=
            self.args.target_rear_median_m + self.args.lidar_stop_margin_m)
        if odom_reached or lidar_reached:
            lidar_error = self.rear_median - self.args.target_rear_median_m
            heading_error_deg = math.degrees(heading_error)
            crosscheck_ok = (
                abs(remaining) <= self.args.crosscheck_distance_m and
                abs(lidar_error) <= self.args.crosscheck_lidar_m and
                abs(heading_error_deg) <= self.args.heading_tolerance_deg)
            self.finish(
                "reached" if crosscheck_ok else "stopped_for_crosscheck",
                "odom_and_lidar_consistent" if crosscheck_ok else
                ("odom_stop" if odom_reached else "lidar_stop"))
            return
        if self.rear_median < self.args.minimum_safe_rear_m:
            self.finish("aborted", "rear_clearance_limit")
            return

        if remaining > 0.03:
            desired_linear = -self.args.max_linear
        else:
            speed = clamp(
                0.5 * remaining, self.args.min_linear, self.args.max_linear_near)
            desired_linear = -speed
        if abs(math.degrees(heading_error)) > self.args.heading_pause_deg:
            desired_linear = 0.0
        desired_angular = clamp(
            self.args.heading_kp * heading_error,
            -self.args.max_angular, self.args.max_angular)

        dt = 0.1
        linear_step = self.args.max_linear_accel * dt
        angular_step = self.args.max_angular_accel * dt
        linear = clamp(
            desired_linear,
            self.previous_linear - linear_step,
            self.previous_linear + linear_step)
        angular = clamp(
            desired_angular,
            self.previous_angular - angular_step,
            self.previous_angular + angular_step)
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_pub.publish(command)
        self.previous_linear = linear
        self.previous_angular = angular
        self.trace.append({
            "elapsed_sec": now - self.motion_started_at,
            "progress_m": progress,
            "remaining_m": remaining,
            "heading_error_deg": math.degrees(heading_error),
            "rear_median_m": self.rear_median,
            "cmd_linear_x_mps": linear,
            "cmd_angular_z_rps": angular,
            "battery_v": self.battery_v,
        })

    def control_tick(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if self.state == "preflight":
            self.preflight_tick(now)
        elif self.state == "running":
            self.running_tick(now)

    def result(self) -> dict:
        progress = self.progress_m()
        heading_error = None if self.yaw is None else math.degrees(
            wrap(self.target_heading - self.yaw))
        return {
            "schema_version": 1,
            "outcome": self.outcome,
            "reason": self.reason,
            "target": {
                "remaining_reverse_distance_m": self.args.distance_m,
                "target_heading_deg": self.args.target_heading_deg,
                "target_rear_median_m": self.args.target_rear_median_m,
            },
            "final": {
                "progress_m": progress,
                "remaining_m": None if progress is None else self.args.distance_m - progress,
                "heading_error_deg": heading_error,
                "rear_median_m": self.rear_median,
                "rear_median_error_m": (
                    None if self.rear_median is None else
                    self.rear_median - self.args.target_rear_median_m),
            },
            "battery_min_v": self.battery_min_v,
            "trace": self.trace,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--distance-m", required=True, type=float)
    parser.add_argument("--target-heading-deg", required=True, type=float)
    parser.add_argument("--target-rear-median-m", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--battery-topic", default="/battery_state")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--rear-half-width-deg", type=float, default=10.0)
    parser.add_argument("--min-rear-points", type=int, default=8)
    parser.add_argument("--min-voltage", type=float, default=10.5)
    parser.add_argument("--max-linear", type=float, default=0.020)
    parser.add_argument("--max-linear-near", type=float, default=0.012)
    parser.add_argument("--min-linear", type=float, default=0.007)
    parser.add_argument("--max-angular", type=float, default=0.06)
    parser.add_argument("--heading-kp", type=float, default=1.5)
    parser.add_argument("--heading-pause-deg", type=float, default=3.0)
    parser.add_argument("--heading-tolerance-deg", type=float, default=1.5)
    parser.add_argument("--max-linear-accel", type=float, default=0.03)
    parser.add_argument("--max-angular-accel", type=float, default=0.15)
    parser.add_argument("--distance-tolerance-m", type=float, default=0.002)
    parser.add_argument("--lidar-stop-margin-m", type=float, default=0.002)
    parser.add_argument("--crosscheck-distance-m", type=float, default=0.012)
    parser.add_argument("--crosscheck-lidar-m", type=float, default=0.012)
    parser.add_argument("--minimum-safe-rear-m", type=float, default=0.30)
    parser.add_argument("--max-runtime", type=float, default=12.0)
    parser.add_argument("--preflight-timeout", type=float, default=10.0)
    args = parser.parse_args()
    if not args.enable_motion:
        parser.error("--enable-motion is required")
    if args.distance_m <= 0:
        parser.error("--distance-m must be positive")

    rclpy.init()
    node = BackupController(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish("aborted", "operator_interrupt")
    finally:
        for _ in range(5):
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=0.05)
        result = node.result()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({
        "output": str(args.output),
        "outcome": result["outcome"],
        "reason": result["reason"],
        "final": result["final"],
        "battery_min_v": result["battery_min_v"],
    }, indent=2))
    return 0 if result["outcome"] == "reached" else 2


if __name__ == "__main__":
    raise SystemExit(main())
