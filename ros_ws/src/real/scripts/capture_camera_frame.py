#!/usr/bin/env python3
"""Capture a settled frame or short burst from the TB3 camera stream."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FrameCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("capture_camera_frame")
        self.frames: list[CompressedImage] = []
        self.create_subscription(
            CompressedImage,
            topic,
            lambda message: self.frames.append(message),
            qos_profile_sensor_data,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/image_raw/compressed")
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument(
        "--burst-count",
        type=int,
        default=1,
        help="Number of final consecutive frames to save (default: 1)",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    if args.frames < 1 or args.burst_count < 1:
        parser.error("--frames and --burst-count must both be positive")

    rclpy.init()
    node = FrameCapture(args.topic)
    try:
        deadline = time.monotonic() + args.timeout
        target_count = args.frames + args.burst_count - 1
        while len(node.frames) < target_count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.frames:
            raise RuntimeError(f"No image received from {args.topic}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        selected = node.frames[-min(args.burst_count, len(node.frames)):]
        saved = []
        shape = None
        for index, message in enumerate(selected):
            image = cv2.imdecode(
                np.frombuffer(bytes(message.data), np.uint8),
                cv2.IMREAD_COLOR,
            )
            if image is None:
                raise RuntimeError("JPEG decode failed")
            shape = image.shape
            if args.burst_count == 1:
                path = args.output
            else:
                suffix = args.output.suffix or ".jpg"
                path = args.output.with_name(
                    f"{args.output.stem}_{index:03d}{suffix}")
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"Could not write {path}")
            saved.append(path)
        print(
            f"CAPTURED={len(saved)} FIRST={saved[0]} LAST={saved[-1]} "
            f"WIDTH={shape[1]} HEIGHT={shape[0]} FRAMES={len(node.frames)}",
            flush=True,
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
