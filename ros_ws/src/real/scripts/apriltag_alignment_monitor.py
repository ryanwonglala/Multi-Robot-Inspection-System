#!/usr/bin/env python3
"""Read-only live AprilTag alignment monitor for the TB3 VP4 approach.

This node intentionally has no Twist publisher. It estimates the live Tag pose,
compares it with a recorded reference, and reports what a future controller
would request without being able to move the robot.
"""

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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import yaml


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def tag_normal_yaw_deg(rotation: np.ndarray) -> float:
    # The printed face normal points opposite the Tag frame's +Z axis for the
    # IPPE convention used here. Express its horizontal angle in camera axes.
    normal = -rotation[:, 2]
    return math.degrees(math.atan2(float(normal[0]), float(normal[2])))


def suggested_command(
        range_error_m: float,
        bearing_error_deg: float,
        normal_yaw_error_deg: float) -> dict[str, float | str]:
    # Read-only proposal. A later motion node may adopt these signs only after
    # the operator validates this monitor at the real B pose.
    combined_angle_deg = 0.75 * bearing_error_deg + 0.25 * normal_yaw_error_deg
    angular = float(np.clip(-1.2 * math.radians(combined_angle_deg), -0.15, 0.15))
    if abs(bearing_error_deg) > 8.0 or abs(normal_yaw_error_deg) > 12.0:
        linear = 0.0
    elif range_error_m > 0.01:
        linear = float(np.clip(0.5 * range_error_m, 0.015, 0.04))
    else:
        linear = 0.0
    if angular > 0.005:
        turn = "left_ccw"
    elif angular < -0.005:
        turn = "right_cw"
    else:
        turn = "hold_heading"
    return {
        "linear_x_mps_dry_run": linear,
        "angular_z_rps_dry_run": angular,
        "turn": turn,
    }


class AlignmentMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("apriltag_alignment_monitor")
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
        self.samples: list[dict] = []
        self.frame_count = 0
        self.create_subscription(
            CompressedImage, args.topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, message: CompressedImage) -> None:
        self.frame_count += 1
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            return
        corners, ids, _ = self.detector.detectMarkers(image)
        values = [] if ids is None else ids.reshape(-1).tolist()
        matches = [index for index, value in enumerate(values)
                   if value == self.tag_id]
        if len(matches) != 1:
            return
        image_points = corners[matches[0]].reshape(4, 2).astype(np.float64)
        success, rotation_vector, translation_vector = cv2.solvePnP(
            self.object_points, image_points,
            self.camera_matrix, self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            return
        translation = translation_vector.reshape(3)
        if translation[2] <= 0:
            return
        projected, _ = cv2.projectPoints(
            self.object_points, rotation_vector, translation_vector,
            self.camera_matrix, self.distortion)
        residual = projected.reshape(4, 2) - image_points
        reprojection_error = float(np.sqrt(
            np.mean(np.sum(residual ** 2, axis=1))))
        if reprojection_error > self.args.max_reprojection_error_px:
            return
        rotation, _ = cv2.Rodrigues(rotation_vector)
        current_range = float(np.linalg.norm(translation))
        current_bearing = math.degrees(math.atan2(
            float(translation[0]), float(translation[2])))
        current_normal_yaw = tag_normal_yaw_deg(rotation)
        range_error = current_range - self.target_range
        bearing_error = wrap_degrees(current_bearing - self.target_bearing)
        normal_yaw_error = wrap_degrees(
            current_normal_yaw - self.target_normal_yaw)
        guidance = suggested_command(
            range_error, bearing_error, normal_yaw_error)
        self.samples.append({
            "stamp_sec": int(message.header.stamp.sec),
            "stamp_nanosec": int(message.header.stamp.nanosec),
            "range_m": current_range,
            "bearing_deg": current_bearing,
            "normal_yaw_deg": current_normal_yaw,
            "range_error_m": range_error,
            "bearing_error_deg": bearing_error,
            "normal_yaw_error_deg": normal_yaw_error,
            "reprojection_error_px": reprojection_error,
            **guidance,
        })


def aggregate(samples: list[dict], key: str) -> dict[str, float]:
    values = [float(sample[key]) for sample in samples]
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--camera-yaml", required=True, type=Path)
    parser.add_argument("--topic", default="/image_raw/compressed")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-reprojection-error-px", type=float, default=2.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    rclpy.init()
    node = AlignmentMonitor(args)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.samples:
        raise RuntimeError(
            f"No valid Tag {node.tag_id} observations received from {args.topic}")
    result = {
        "schema_version": 1,
        "mode": "read_only_no_cmd_vel_publisher",
        "topic": args.topic,
        "duration_sec": args.duration,
        "frames_received": node.frame_count,
        "valid_observations": len(node.samples),
        "target": {
            "reference_file": str(args.reference),
            "range_m": node.target_range,
            "bearing_deg": node.target_bearing,
            "normal_yaw_deg": node.target_normal_yaw,
        },
        "live": {
            key: aggregate(node.samples, key)
            for key in (
                "range_m", "bearing_deg", "normal_yaw_deg",
                "range_error_m", "bearing_error_deg",
                "normal_yaw_error_deg", "reprojection_error_px",
                "linear_x_mps_dry_run", "angular_z_rps_dry_run")
        },
        "suggested_turn_counts": {
            turn: sum(sample["turn"] == turn for sample in node.samples)
            for turn in ("left_ccw", "right_cw", "hold_heading")
        },
        "samples": node.samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    concise = {
        "output": str(args.output),
        "mode": result["mode"],
        "frames_received": result["frames_received"],
        "valid_observations": result["valid_observations"],
        "mean_range_error_m": result["live"]["range_error_m"]["mean"],
        "mean_bearing_error_deg": result["live"]["bearing_error_deg"]["mean"],
        "mean_normal_yaw_error_deg": result["live"]["normal_yaw_error_deg"]["mean"],
        "mean_proposed_linear_mps": result["live"]["linear_x_mps_dry_run"]["mean"],
        "mean_proposed_angular_rps": result["live"]["angular_z_rps_dry_run"]["mean"],
        "suggested_turn_counts": result["suggested_turn_counts"],
    }
    print(json.dumps(concise, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
