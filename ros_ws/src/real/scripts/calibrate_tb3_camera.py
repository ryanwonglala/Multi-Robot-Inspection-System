#!/usr/bin/env python3
"""Calibrate the physical TB3 USB camera from a printed checkerboard."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import yaml


class CalibrationCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("calibrate_tb3_camera")
        self.latest: CompressedImage | None = None
        self.create_subscription(
            CompressedImage,
            topic,
            lambda message: setattr(self, "latest", message),
            qos_profile_sensor_data,
        )


def descriptor(
    corners: np.ndarray, width: int, height: int, cols: int, rows: int
) -> tuple[float, ...]:
    points = corners.reshape(-1, 2)
    center = points.mean(axis=0)
    x, y, w, h = cv2.boundingRect(points.astype(np.float32))
    area = max(float(w * h) / float(width * height), 1e-6)
    top_left = points[0]
    top_right = points[cols - 1]
    bottom_left = points[(rows - 1) * cols]
    bottom_right = points[-1]
    vector = top_right - top_left
    angle = math.atan2(float(vector[1]), float(vector[0]))
    top = max(float(np.linalg.norm(top_right - top_left)), 1e-6)
    bottom = max(float(np.linalg.norm(bottom_right - bottom_left)), 1e-6)
    left = max(float(np.linalg.norm(bottom_left - top_left)), 1e-6)
    right = max(float(np.linalg.norm(bottom_right - top_right)), 1e-6)
    return (
        float(center[0] / width),
        float(center[1] / height),
        math.log(area),
        angle,
        math.log(top / bottom),
        math.log(left / right),
    )


def descriptor_distance(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    position = math.hypot(left[0] - right[0], left[1] - right[1])
    scale = abs(left[2] - right[2])
    angle = abs(math.atan2(
        math.sin(left[3] - right[3]), math.cos(left[3] - right[3])
    ))
    perspective = math.hypot(left[4] - right[4], left[5] - right[5])
    return position + 0.35 * scale + 0.15 * angle + 0.25 * perspective


def write_camera_yaml(
    path: Path,
    width: int,
    height: int,
    matrix: np.ndarray,
    distortion: np.ndarray,
) -> None:
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    data = {
        "image_width": width,
        "image_height": height,
        "camera_name": "tb3_usb_camera",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(distortion.size),
            "data": [float(value) for value in distortion.ravel()],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/image_raw/compressed")
    parser.add_argument("--cols", type=int, default=7, help="inner corner columns")
    parser.add_argument("--rows", type=int, default=5, help="inner corner rows")
    parser.add_argument("--square", type=float, default=0.024, help="square size in metres")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--min-diversity", type=float, default=0.08)
    parser.add_argument(
        "--manual",
        action="store_true",
        help="show a preview and accept a detected view only when SPACE is pressed",
    )
    args = parser.parse_args()

    pattern = (args.cols, args.rows)
    object_template = np.zeros((args.rows * args.cols, 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    )
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    descriptors: list[tuple[float, ...]] = []
    image_size: tuple[int, int] | None = None

    rclpy.init()
    node = CalibrationCapture(args.topic)
    last_stamp: tuple[int, int] | None = None
    deadline = time.monotonic() + args.timeout
    try:
        while len(image_points) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            message = node.latest
            if message is None:
                continue
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            image = cv2.imdecode(
                np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR
            )
            if image is None:
                continue
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image_size = (gray.shape[1], gray.shape[0])
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK,
            )
            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (
                        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                        30,
                        0.001,
                    ),
                )
            key = -1
            if args.manual:
                preview = image.copy()
                if found:
                    cv2.drawChessboardCorners(preview, pattern, corners, found)
                status = (
                    f"{len(image_points)}/{args.samples} "
                    + ("READY - SPACE to capture" if found else "board not detected")
                )
                cv2.putText(
                    preview,
                    status,
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0) if found else (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("TB3 calibration - SPACE capture, Q quit", preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key != ord(" "):
                    continue
            if not found:
                if args.manual and key == ord(" "):
                    print("REJECTED=no_complete_checkerboard", flush=True)
                continue
            current = descriptor(
                corners, image_size[0], image_size[1], args.cols, args.rows
            )
            if descriptors and min(
                descriptor_distance(current, previous) for previous in descriptors
            ) < args.min_diversity:
                if args.manual:
                    print("REJECTED=too_similar_to_existing_view", flush=True)
                continue
            descriptors.append(current)
            object_points.append(object_template.copy())
            image_points.append(corners)
            print(
                f"ACCEPTED={len(image_points)}/{args.samples} "
                f"CENTER=({current[0]:.2f},{current[1]:.2f}) "
                f"LOG_AREA={current[2]:.2f}",
                flush=True,
            )
    finally:
        if args.manual:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    if image_size is None or len(image_points) < max(12, args.samples // 2):
        raise RuntimeError(
            f"Only {len(image_points)} diverse checkerboard views collected; "
            "need at least 12"
        )

    x_span = max(item[0] for item in descriptors) - min(item[0] for item in descriptors)
    y_span = max(item[1] for item in descriptors) - min(item[1] for item in descriptors)
    scale_span = max(item[2] for item in descriptors) - min(
        item[2] for item in descriptors
    )
    perspective_span = max(
        max(abs(item[4]), abs(item[5])) for item in descriptors
    )
    coverage_failures = []
    if x_span < 0.30:
        coverage_failures.append(f"x_span={x_span:.3f}<0.30")
    if y_span < 0.25:
        coverage_failures.append(f"y_span={y_span:.3f}<0.25")
    if scale_span < 0.50:
        coverage_failures.append(f"log_area_span={scale_span:.3f}<0.50")
    if perspective_span < 0.12:
        coverage_failures.append(
            f"perspective={perspective_span:.3f}<0.12"
        )
    if coverage_failures:
        raise RuntimeError(
            "Calibration view coverage insufficient: " + ", ".join(coverage_failures)
        )

    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    total_error = 0.0
    for index, points in enumerate(object_points):
        projected, _ = cv2.projectPoints(
            points, rotations[index], translations[index], matrix, distortion
        )
        total_error += cv2.norm(image_points[index], projected, cv2.NORM_L2) / len(
            projected
        )
    mean_error = total_error / len(object_points)
    if not (
        0.35 * image_size[0] <= matrix[0, 0] <= 2.0 * image_size[0]
        and 0.35 * image_size[1] <= matrix[1, 1] <= 2.5 * image_size[1]
        and 0.25 * image_size[0] <= matrix[0, 2] <= 0.75 * image_size[0]
        and 0.25 * image_size[1] <= matrix[1, 2] <= 0.75 * image_size[1]
    ):
        raise RuntimeError(
            "Calibration solved but intrinsics are implausible for this frame: "
            f"fx={matrix[0, 0]:.3f} fy={matrix[1, 1]:.3f} "
            f"cx={matrix[0, 2]:.3f} cy={matrix[1, 2]:.3f}"
        )
    write_camera_yaml(args.output, image_size[0], image_size[1], matrix, distortion)
    print(
        f"CALIBRATION_SAVED={args.output} SAMPLES={len(image_points)} "
        f"RMS_PX={rms:.4f} MEAN_REPROJECTION_PX={mean_error:.4f} "
        f"FX={matrix[0, 0]:.3f} FY={matrix[1, 1]:.3f} "
        f"CX={matrix[0, 2]:.3f} CY={matrix[1, 2]:.3f}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
