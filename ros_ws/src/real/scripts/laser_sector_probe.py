#!/usr/bin/env python3
"""Record robust distance statistics for one LaserScan angular sector."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SectorProbe(Node):
    def __init__(self, topic: str, center_rad: float, half_width_rad: float) -> None:
        super().__init__("laser_sector_probe")
        self.center_rad = center_rad
        self.half_width_rad = half_width_rad
        self.scans = []
        self.create_subscription(
            LaserScan, topic, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, message: LaserScan) -> None:
        values = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(wrap(angle - self.center_rad)) > self.half_width_rad:
                continue
            if math.isfinite(value) and message.range_min <= value <= message.range_max:
                values.append(float(value))
        if not values:
            self.scans.append({"valid_count": 0})
            return
        values.sort()
        self.scans.append({
            "valid_count": len(values),
            "minimum_m": values[0],
            "median_m": statistics.median(values),
            "maximum_m": values[-1],
        })


def summary(values: list[float]) -> dict[str, float]:
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--center-deg", type=float, default=180.0)
    parser.add_argument("--half-width-deg", type=float, default=10.0)
    parser.add_argument("--scans", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.scans < 1 or args.timeout <= 0 or args.half_width_deg <= 0:
        parser.error("--scans, --timeout and --half-width-deg must be positive")

    rclpy.init()
    node = SectorProbe(
        args.topic, math.radians(args.center_deg),
        math.radians(args.half_width_deg))
    try:
        deadline = time.monotonic() + args.timeout
        while len(node.scans) < args.scans and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    valid = [scan for scan in node.scans if scan["valid_count"] > 0]
    if not valid:
        raise RuntimeError("No valid ranges received in the requested sector")
    result = {
        "topic": args.topic,
        "center_deg": args.center_deg,
        "half_width_deg": args.half_width_deg,
        "requested_scans": args.scans,
        "received_scans": len(node.scans),
        "valid_scans": len(valid),
        "valid_points_per_scan": summary(
            [float(scan["valid_count"]) for scan in valid]),
        "sector_minimum_m": summary([scan["minimum_m"] for scan in valid]),
        "sector_median_m": summary([scan["median_m"] for scan in valid]),
        "sector_maximum_m": summary([scan["maximum_m"] for scan in valid]),
        "scans": node.scans,
    }
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": None if args.output is None else str(args.output),
        "received_scans": result["received_scans"],
        "valid_scans": result["valid_scans"],
        "valid_points_per_scan": result["valid_points_per_scan"],
        "sector_minimum_m": result["sector_minimum_m"],
        "sector_median_m": result["sector_median_m"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
