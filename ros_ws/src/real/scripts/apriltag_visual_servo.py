#!/usr/bin/env python3
"""Guarded AprilTag visual servo from the VP4 handoff pose to reference A."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
import yaml


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def tag_normal_yaw_deg(rotation: np.ndarray) -> float:
    normal = -rotation[:, 2]
    return math.degrees(math.atan2(float(normal[0]), float(normal[2])))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class VisualServo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("apriltag_visual_servo")
        self.args = args
        calibration = yaml.safe_load(
            args.camera_yaml.read_text(encoding="utf-8"))
        self.camera_matrix = np.asarray(
            calibration["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(
            calibration["distortion_coefficients"]["data"], dtype=np.float64)
        reference_data = json.loads(args.reference.read_text(encoding="utf-8"))
        reference = reference_data["reference"]
        self.target_range = float(reference["range_m"])
        self.target_bearing = float(reference["horizontal_bearing_deg"])
        target_rotation = np.asarray(
            reference["rotation_matrix_tag_to_camera"], dtype=np.float64)
        self.target_normal_yaw = tag_normal_yaw_deg(target_rotation)
        self.tag_id = int(reference_data["tag_id"])
        self.tag_size = float(reference_data["tag_size_m"])

        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        half = self.tag_size / 2.0
        self.object_points = np.asarray([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        self.create_subscription(
            CompressedImage, args.image_topic,
            self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, args.battery_topic, self.on_battery, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_subscription(
            LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.control_tick)

        self.cmd_pub = None
        self.state = "preflight"
        self.done = False
        self.outcome = "running"
        self.reason = ""
        self.started_at = time.monotonic()
        self.motion_started_at = None
        self.valid_since = None
        self.goal_since = None
        self.last_tag_at = None
        self.battery_at = None
        self.battery_v = None
        self.battery_present = False
        self.battery_min_v = None
        self.odom_xy = None
        self.initial_odom_xy = None
        self.scan_at = None
        self.rear_median = None
        self.latest = None
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.trace = []
        self.frames_received = 0
        self.valid_observations = 0
        self.invalid_frames = 0

    def on_battery(self, message: BatteryState) -> None:
        self.battery_at = time.monotonic()
        self.battery_v = float(message.voltage)
        self.battery_present = bool(message.present)
        if self.battery_min_v is None or self.battery_v < self.battery_min_v:
            self.battery_min_v = self.battery_v

    def on_odom(self, message: Odometry) -> None:
        self.odom_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y))

    def on_scan(self, message: LaserScan) -> None:
        values = []
        center = math.pi
        half_width = math.radians(self.args.rear_half_width_deg)
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(math.atan2(
                    math.sin(angle - center),
                    math.cos(angle - center))) > half_width:
                continue
            if (math.isfinite(value) and
                    message.range_min <= value <= message.range_max):
                values.append(float(value))
        if len(values) >= self.args.min_rear_points:
            self.rear_median = statistics.median(values)
            self.scan_at = time.monotonic()

    def on_image(self, message: CompressedImage) -> None:
        self.frames_received += 1
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.invalid_frames += 1
            return
        corners, ids, _ = self.detector.detectMarkers(image)
        values = [] if ids is None else ids.reshape(-1).tolist()
        matches = [index for index, value in enumerate(values)
                   if value == self.tag_id]
        if len(matches) != 1:
            self.invalid_frames += 1
            return
        image_points = corners[matches[0]].reshape(4, 2).astype(np.float64)
        success, rotation_vector, translation_vector = cv2.solvePnP(
            self.object_points, image_points,
            self.camera_matrix, self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            self.invalid_frames += 1
            return
        translation = translation_vector.reshape(3)
        if translation[2] <= 0:
            self.invalid_frames += 1
            return
        projected, _ = cv2.projectPoints(
            self.object_points, rotation_vector, translation_vector,
            self.camera_matrix, self.distortion)
        residual = projected.reshape(4, 2) - image_points
        reprojection_error = float(np.sqrt(
            np.mean(np.sum(residual ** 2, axis=1))))
        if reprojection_error > self.args.max_reprojection_error_px:
            self.invalid_frames += 1
            return
        rotation, _ = cv2.Rodrigues(rotation_vector)
        current_range = float(np.linalg.norm(translation))
        current_bearing = math.degrees(math.atan2(
            float(translation[0]), float(translation[2])))
        current_normal_yaw = tag_normal_yaw_deg(rotation)
        now = time.monotonic()
        self.last_tag_at = now
        self.valid_observations += 1
        self.latest = {
            "range_m": current_range,
            "bearing_deg": current_bearing,
            "normal_yaw_deg": current_normal_yaw,
            "range_error_m": current_range - self.target_range,
            "bearing_error_deg": wrap_degrees(
                current_bearing - self.target_bearing),
            "normal_yaw_error_deg": wrap_degrees(
                current_normal_yaw - self.target_normal_yaw),
            "reprojection_error_px": reprojection_error,
        }
        if self.valid_since is None:
            self.valid_since = now

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
            if self.frames_received == 0:
                reason = "image_stream_timeout"
            elif self.latest is None:
                reason = "apriltag_not_detected"
            elif self.battery_at is None:
                reason = "battery_feedback_timeout"
            elif self.odom_xy is None:
                reason = "odom_feedback_timeout"
            elif self.scan_at is None or self.rear_median is None:
                reason = "rear_scan_timeout"
            else:
                reason = "preflight_timeout"
            self.finish("aborted", reason)
            return
        if self.latest is None or self.valid_since is None:
            return
        if now - self.valid_since < self.args.preflight_stable_sec:
            return
        if (self.battery_at is None or
                now - self.battery_at > self.args.battery_timeout):
            return
        if (self.scan_at is None or
                now - self.scan_at > self.args.rear_scan_timeout or
                self.rear_median is None):
            return
        if not self.battery_present or self.battery_v < self.args.min_voltage:
            self.finish("aborted", "battery_not_safe")
            return
        if (self.latest["range_error_m"] <
                -self.args.max_initial_close_error_m):
            self.finish("aborted", "start_pose_too_close")
            return
        if self.latest["range_error_m"] > self.args.max_initial_range_error_m:
            self.finish("aborted", "start_pose_too_far")
            return
        existing = self.get_publishers_info_by_topic(self.args.cmd_topic)
        if existing:
            names = ",".join(sorted({item.node_name for item in existing}))
            self.finish("aborted", f"existing_cmd_vel_publishers:{names}")
            return
        self.cmd_pub = self.create_publisher(Twist, self.args.cmd_topic, 10)
        self.initial_odom_xy = self.odom_xy
        self.motion_started_at = now
        self.state = "running"

    def running_tick(self, now: float) -> None:
        if (self.args.max_runtime > 0.0 and
                now - self.motion_started_at > self.args.max_runtime):
            self.finish("aborted", "motion_timeout")
            return
        if (self.battery_at is None or
                now - self.battery_at > self.args.battery_timeout):
            self.finish("aborted", "battery_feedback_lost")
            return
        if not self.battery_present or self.battery_v < self.args.min_voltage:
            self.finish("aborted", "battery_not_safe")
            return
        if (self.scan_at is None or
                now - self.scan_at > self.args.rear_scan_timeout):
            self.finish("aborted", "rear_scan_lost")
            return
        if (self.last_tag_at is None or
                now - self.last_tag_at > self.args.tag_timeout):
            # A blurred frame or brief occlusion must not terminate a docking
            # that is otherwise converging. Hold still until a fresh tag pose
            # arrives; battery, odometry and lidar safety checks remain active.
            self.publish_zero()
            self.goal_since = None
            return
        if self.initial_odom_xy is not None and self.odom_xy is not None:
            displacement = math.hypot(
                self.odom_xy[0] - self.initial_odom_xy[0],
                self.odom_xy[1] - self.initial_odom_xy[1])
            if displacement > self.args.max_odom_displacement_m:
                self.finish("aborted", "odom_displacement_limit")
                return

        error = self.latest
        if error["range_error_m"] < -self.args.max_initial_close_error_m:
            self.finish("aborted", "range_recovery_limit")
            return
        at_goal = (
            abs(error["range_error_m"]) <= self.args.range_tolerance_m and
            abs(error["bearing_error_deg"]) <= self.args.bearing_tolerance_deg and
            abs(error["normal_yaw_error_deg"]) <= self.args.normal_tolerance_deg)
        if at_goal:
            self.publish_zero()
            if self.goal_since is None:
                self.goal_since = now
            elif now - self.goal_since >= self.args.goal_stable_sec:
                self.finish("reached", "reference_pose_stable")
            return
        self.goal_since = None

        combined_deg = (
            0.75 * error["bearing_error_deg"] +
            0.25 * error["normal_yaw_error_deg"])
        desired_angular = clamp(
            -1.2 * math.radians(combined_deg),
            -self.args.max_angular, self.args.max_angular)
        if (abs(error["bearing_error_deg"]) > 6.0 or
                abs(error["normal_yaw_error_deg"]) > 10.0):
            desired_linear = 0.0
        elif error["range_error_m"] > self.args.range_tolerance_m:
            desired_linear = clamp(
                0.35 * error["range_error_m"],
                self.args.min_linear, self.args.max_linear)
        elif error["range_error_m"] < -self.args.range_tolerance_m:
            if (self.rear_median is None or
                    self.rear_median <= self.args.minimum_safe_rear_m):
                self.finish("aborted", "rear_clearance_limit")
                return
            desired_linear = -clamp(
                0.35 * abs(error["range_error_m"]),
                self.args.min_linear, self.args.max_linear)
        else:
            desired_linear = 0.0

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
            **error,
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
        displacement = None
        if self.initial_odom_xy is not None and self.odom_xy is not None:
            displacement = math.hypot(
                self.odom_xy[0] - self.initial_odom_xy[0],
                self.odom_xy[1] - self.initial_odom_xy[1])
        return {
            "schema_version": 1,
            "outcome": self.outcome,
            "reason": self.reason,
            "reference_file": str(self.args.reference),
            "target": {
                "range_m": self.target_range,
                "bearing_deg": self.target_bearing,
                "normal_yaw_deg": self.target_normal_yaw,
            },
            "final": self.latest,
            "frames_received": self.frames_received,
            "valid_observations": self.valid_observations,
            "invalid_frames": self.invalid_frames,
            "battery_min_v": self.battery_min_v,
            "odom_displacement_m": displacement,
            "rear_median_m": self.rear_median,
            "limits": {
                "max_linear_mps": self.args.max_linear,
                "max_angular_rps": self.args.max_angular,
                "max_runtime_sec": self.args.max_runtime,
                "tag_timeout_sec": self.args.tag_timeout,
            },
            "trace": self.trace,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--camera-yaml", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--image-topic", default="/image_raw/compressed")
    parser.add_argument("--battery-topic", default="/battery_state")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--min-voltage", type=float, default=10.5)
    parser.add_argument("--battery-timeout", type=float, default=2.0)
    parser.add_argument("--rear-scan-timeout", type=float, default=2.0)
    parser.add_argument("--max-linear", type=float, default=0.035)
    parser.add_argument("--min-linear", type=float, default=0.010)
    parser.add_argument("--max-angular", type=float, default=0.08)
    parser.add_argument("--max-linear-accel", type=float, default=0.03)
    parser.add_argument("--max-angular-accel", type=float, default=0.15)
    parser.add_argument("--max-runtime", type=float, default=25.0)
    parser.add_argument("--preflight-timeout", type=float, default=12.0)
    parser.add_argument("--preflight-stable-sec", type=float, default=1.0)
    parser.add_argument("--tag-timeout", type=float, default=0.4)
    parser.add_argument("--goal-stable-sec", type=float, default=1.0)
    parser.add_argument("--range-tolerance-m", type=float, default=0.012)
    parser.add_argument("--bearing-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--normal-tolerance-deg", type=float, default=1.5)
    parser.add_argument("--max-initial-range-error-m", type=float, default=0.40)
    parser.add_argument("--max-initial-close-error-m", type=float, default=0.12)
    parser.add_argument("--max-odom-displacement-m", type=float, default=0.35)
    parser.add_argument("--rear-half-width-deg", type=float, default=10.0)
    parser.add_argument("--min-rear-points", type=int, default=8)
    parser.add_argument("--minimum-safe-rear-m", type=float, default=0.30)
    parser.add_argument("--max-reprojection-error-px", type=float, default=2.0)
    args = parser.parse_args()
    if not args.enable_motion:
        parser.error("--enable-motion is required; use the read-only monitor first")
    if args.battery_timeout <= 0.0 or args.rear_scan_timeout <= 0.0:
        parser.error("feedback timeout values must be positive")

    rclpy.init()
    node = VisualServo(args)
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
    print(json.dumps({
        "output": str(args.output),
        "outcome": result["outcome"],
        "reason": result["reason"],
        "final": result["final"],
        "frames_received": result["frames_received"],
        "valid_observations": result["valid_observations"],
        "invalid_frames": result["invalid_frames"],
        "battery_min_v": result["battery_min_v"],
        "odom_displacement_m": result["odom_displacement_m"],
    }, indent=2))
    return 0 if result["outcome"] == "reached" else 2


if __name__ == "__main__":
    raise SystemExit(main())
