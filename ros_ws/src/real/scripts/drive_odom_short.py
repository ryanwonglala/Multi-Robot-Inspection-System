#!/usr/bin/env python3
"""Guarded centimetre-scale straight motion with odometry heading hold."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ShortDrive(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("drive_odom_short")
        self.args = args
        self.target_heading = math.radians(args.target_heading_deg)
        self.direction = 1.0 if args.distance_m > 0.0 else -1.0
        self.target_distance = abs(args.distance_m)
        self.create_subscription(
            Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, args.battery_topic, self.on_battery, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.cmd_pub = None
        self.started_at = time.monotonic()
        self.motion_started_at = None
        self.odom_at = None
        self.battery_at = None
        self.xy = None
        self.initial_xy = None
        self.yaw = None
        self.battery_v = None
        self.battery_present = False
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.done = False
        self.outcome = "running"
        self.reason = ""
        self.trace = []

    def on_odom(self, message: Odometry) -> None:
        self.odom_at = time.monotonic()
        self.xy = (float(message.pose.pose.position.x),
                   float(message.pose.pose.position.y))
        self.yaw = yaw_from_odom(message)

    def on_battery(self, message: BatteryState) -> None:
        self.battery_at = time.monotonic()
        self.battery_v = float(message.voltage)
        self.battery_present = bool(message.present)

    def progress(self) -> float | None:
        if self.initial_xy is None or self.xy is None:
            return None
        dx = self.xy[0] - self.initial_xy[0]
        dy = self.xy[1] - self.initial_xy[1]
        forward = (dx * math.cos(self.target_heading) +
                   dy * math.sin(self.target_heading))
        return self.direction * forward

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
        self.done = True

    def tick(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if self.motion_started_at is None:
            if now - self.started_at > self.args.preflight_timeout:
                self.finish("aborted", "preflight_timeout")
                return
            if None in (self.odom_at, self.battery_at, self.xy, self.yaw,
                        self.battery_v):
                return
            if max(now - self.odom_at, now - self.battery_at) > 1.0:
                return
            if not self.battery_present or self.battery_v < self.args.min_voltage:
                self.finish("aborted", "battery_not_safe")
                return
            existing = self.get_publishers_info_by_topic(self.args.cmd_topic)
            if existing:
                names = ",".join(sorted({item.node_name for item in existing}))
                self.finish("aborted", f"existing_cmd_vel_publishers:{names}")
                return
            self.cmd_pub = self.create_publisher(Twist, self.args.cmd_topic, 10)
            self.initial_xy = self.xy
            self.motion_started_at = now
            return

        if now - self.motion_started_at > self.args.max_runtime:
            self.finish("aborted", "motion_timeout")
            return
        if max(now - self.odom_at, now - self.battery_at) > 0.7:
            self.finish("aborted", "feedback_lost")
            return
        if not self.battery_present or self.battery_v < self.args.min_voltage:
            self.finish("aborted", "battery_not_safe")
            return
        progress = self.progress()
        remaining = self.target_distance - progress
        heading_error = wrap(self.target_heading - self.yaw)
        if remaining <= self.args.distance_tolerance_m:
            self.finish("reached", "odom_distance_reached")
            return
        if progress < -0.015 or progress > self.target_distance + 0.025:
            self.finish("aborted", "odom_progress_limit")
            return
        speed = clamp(
            0.6 * remaining, self.args.min_linear, self.args.max_linear)
        desired_linear = self.direction * speed
        if abs(math.degrees(heading_error)) > self.args.heading_pause_deg:
            desired_linear = 0.0
        desired_angular = clamp(
            self.args.heading_kp * heading_error,
            -self.args.max_angular, self.args.max_angular)
        linear_step = self.args.max_linear_accel * 0.1
        angular_step = self.args.max_angular_accel * 0.1
        linear = clamp(desired_linear,
                       self.previous_linear - linear_step,
                       self.previous_linear + linear_step)
        angular = clamp(desired_angular,
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
            "cmd_linear_x_mps": linear,
            "cmd_angular_z_rps": angular,
            "battery_v": self.battery_v,
        })

    def result(self) -> dict:
        progress = self.progress()
        return {
            "schema_version": 1,
            "outcome": self.outcome,
            "reason": self.reason,
            "target_distance_m": self.args.distance_m,
            "target_heading_deg": self.args.target_heading_deg,
            "progress_m": progress,
            "remaining_m": None if progress is None else
                self.target_distance - progress,
            "trace": self.trace,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--distance-m", required=True, type=float)
    parser.add_argument("--target-heading-deg", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--battery-topic", default="/battery_state")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--min-voltage", type=float, default=10.5)
    parser.add_argument("--min-linear", type=float, default=0.010)
    parser.add_argument("--max-linear", type=float, default=0.020)
    parser.add_argument("--max-angular", type=float, default=0.10)
    parser.add_argument("--heading-kp", type=float, default=1.5)
    parser.add_argument("--heading-pause-deg", type=float, default=4.0)
    parser.add_argument("--max-linear-accel", type=float, default=0.03)
    parser.add_argument("--max-angular-accel", type=float, default=0.20)
    parser.add_argument("--distance-tolerance-m", type=float, default=0.002)
    parser.add_argument("--max-runtime", type=float, default=12.0)
    parser.add_argument("--preflight-timeout", type=float, default=8.0)
    args = parser.parse_args()
    if not args.enable_motion:
        parser.error("--enable-motion is required")
    if abs(args.distance_m) < 0.005 or abs(args.distance_m) > 0.25:
        parser.error("--distance-m must be between 0.005 and 0.25 m")

    rclpy.init()
    node = ShortDrive(args)
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
            json.dumps(result, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result["outcome"] == "reached" else 2


if __name__ == "__main__":
    raise SystemExit(main())
