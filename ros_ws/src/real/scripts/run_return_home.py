#!/usr/bin/env python3
"""Guarded standalone navigation from the validated unload pose to Home."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from run_hybrid_docking import (
    DockingBridge,
    Pose2D,
    emergency_zero_velocity,
    heading_delta_degrees,
)


class ReturnHomeRun:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.share = Path(get_package_share_directory("real"))
        self.world_path = args.world or (
            self.share / "config/world_model_real_v5.yaml")
        self.map_path = args.map or self.share / "maps/lab_arena_v5.yaml"
        self.nav_params = args.nav_params or self.share / "config/nav2_real.yaml"
        self.script_dir = self.share / "scripts"
        world = yaml.safe_load(self.world_path.read_text(encoding="utf-8"))

        start = world["robot_start"]
        home = start["pose"]
        docking = world["areas"]["arena"]["anomaly_handling"][
            "former_vp2_region"]["approach"]["hybrid_docking_reference"]
        final = docking["perfect_stop"]
        self.initial_pose = Pose2D(
            float(final["x"]), float(final["y"]), float(final["yaw"]))
        self.home_pose = Pose2D(
            float(home["x"]), float(home["y"]), float(home["yaw"]))
        self.home_xy_tolerance = float(start.get("xy_tolerance_m", 0.05))
        self.home_yaw_tolerance = float(start.get("yaw_tolerance_rad", 0.08))

        self.nav_process: subprocess.Popen | None = None
        self.nav_log_handle = None
        self.rviz_process: subprocess.Popen | None = None
        self.bridge: DockingBridge | None = None
        self.started = time.monotonic()
        report_root = args.report_dir or (
            self.share.parents[4] / "reports/return_home")
        report_root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("return_home_%Y%m%d_%H%M%S")
        self.report_path = report_root / f"{stamp}_report.json"
        self.nav_log_path = report_root / f"{stamp}_nav2.log"
        self.report = {
            "schema_version": 1,
            "started_at": datetime.now().astimezone().isoformat(),
            "outcome": "running",
            "reason": "",
            "initial_pose_source": "operator_confirmed_perfect_stop",
            "initial_pose": self.initial_pose.__dict__,
            "home_pose": self.home_pose.__dict__,
            "home_xy_tolerance_m": self.home_xy_tolerance,
            "home_yaw_tolerance_rad": self.home_yaw_tolerance,
            "phases": [],
            "nav2_log": str(self.nav_log_path),
        }
        self.write_report()

    def write_report(self) -> None:
        self.report["elapsed_sec"] = time.monotonic() - self.started
        self.report_path.write_text(
            json.dumps(self.report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")

    def phase(self, name: str, started: float, result: dict) -> None:
        self.report["phases"].append({
            "name": name,
            "elapsed_sec": time.monotonic() - started,
            "result": result,
        })
        self.write_report()

    def start_nav2(self) -> None:
        self.nav_log_handle = self.nav_log_path.open("w", encoding="utf-8")
        self.nav_process = subprocess.Popen([
            "ros2", "launch", "nav2_bringup", "bringup_launch.py",
            f"map:={self.map_path}", f"params_file:={self.nav_params}",
            "use_sim_time:=false", "autostart:=true",
            "use_composition:=False",
        ], stdout=self.nav_log_handle, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        if self.args.use_rviz:
            self.rviz_process = subprocess.Popen([
                "rviz2", "-d", str(self.share / "rviz/localization_view.rviz")
            ], start_new_session=True)

    @staticmethod
    def stop_process(process: subprocess.Popen | None) -> None:
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)

    def close_navigation(self) -> None:
        self.stop_process(self.rviz_process)
        self.rviz_process = None
        self.stop_process(self.nav_process)
        self.nav_process = None
        if self.nav_log_handle is not None:
            self.nav_log_handle.close()
            self.nav_log_handle = None

    def run(self) -> int:
        if self.args.check_only:
            self.report["outcome"] = "check_only_ok"
            self.report["reason"] = "configuration_loaded_no_motion"
            self.write_report()
            print(json.dumps({
                "outcome": self.report["outcome"],
                "initial_pose": self.initial_pose.__dict__,
                "home_pose": self.home_pose.__dict__,
                "report": str(self.report_path),
            }, indent=2))
            return 0

        rclpy.init()
        self.bridge = DockingBridge()
        navigation = None
        try:
            self.bridge.wait_base_health(
                self.args.base_feedback_timeout, self.args.min_voltage)
            self.bridge.wait_cmd_idle(3.0)
            self.start_nav2()
            self.bridge.wait_nav_ready(self.args.nav_startup_timeout)

            started = time.monotonic()
            localized = self.bridge.initialise_near(self.initial_pose)
            initial_error = math.hypot(
                localized.x - self.initial_pose.x,
                localized.y - self.initial_pose.y)
            localized_result = {
                "pose": localized.__dict__,
                "distance_from_confirmed_stop_m": initial_error,
            }
            self.phase("localize_at_final_stop", started, localized_result)
            if initial_error > self.args.max_start_error_m:
                raise RuntimeError(
                    f"start_not_at_final_stop:{initial_error:.3f}m")

            started = time.monotonic()
            navigation = self.bridge.navigate(
                self.home_pose, self.args.nav_timeout)
            self.phase("nav2_to_home_xy", started, navigation)
            if navigation["outcome"] != "reached":
                raise RuntimeError(
                    "home_navigation_failed:" + navigation["reason"])
        finally:
            self.close_navigation()
            if self.bridge is not None:
                try:
                    self.bridge.force_zero_velocity()
                finally:
                    self.bridge.destroy_node()
                    self.bridge = None
            if rclpy.ok():
                rclpy.shutdown()

        final_pose = (navigation or {}).get("final_pose") or {}
        if "yaw" not in final_pose:
            raise RuntimeError("home_navigation_heading_missing")
        correction = heading_delta_degrees(
            float(final_pose["yaw"]), self.home_pose.yaw)
        started = time.monotonic()
        if abs(math.radians(correction)) > self.home_yaw_tolerance:
            completed = subprocess.run([
                sys.executable, str(self.script_dir / "rotate_odom_test.py"),
                "--odom-topic", "/odom", "--cmd-topic", "/cmd_vel",
                "--degrees", f"{correction:.6f}",
                "--max-speed", "0.25", "--min-speed", "0.05",
                "--tolerance-deg",
                f"{math.degrees(self.home_yaw_tolerance):.6f}",
                "--timeout", "35", "--battery-topic", "/battery_state",
                "--min-voltage", str(self.args.min_voltage),
            ], text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, check=False)
            orientation = {
                "outcome": "reached" if completed.returncode == 0 else "aborted",
                "exit_code": completed.returncode,
                "requested_correction_deg": correction,
                "console": completed.stdout[-4000:],
            }
        else:
            orientation = {
                "outcome": "already_aligned",
                "requested_correction_deg": correction,
            }
        self.phase("restore_home_yaw", started, orientation)
        if orientation["outcome"] == "aborted":
            raise RuntimeError("home_orientation_failed")

        self.report["outcome"] = "home_reached"
        self.report["reason"] = "home_xy_and_authored_yaw_restored"
        self.write_report()
        print(json.dumps({
            "outcome": self.report["outcome"],
            "home_pose": self.home_pose.__dict__,
            "report": str(self.report_path),
        }, indent=2))
        return 0

    def close_with_error(self, error: Exception) -> int:
        self.close_navigation()
        try:
            emergency_zero_velocity()
        except Exception as stop_error:
            self.report["emergency_stop_error"] = str(stop_error)
        self.report["outcome"] = "aborted"
        self.report["reason"] = str(error)
        self.write_report()
        print(json.dumps({
            "outcome": "aborted", "reason": str(error),
            "report": str(self.report_path),
        }, indent=2))
        return 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Return from the validated unload pose to Home")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--use-rviz", action="store_true")
    parser.add_argument("--min-voltage", type=float, default=11.0)
    parser.add_argument("--base-feedback-timeout", type=float, default=30.0)
    parser.add_argument("--nav-startup-timeout", type=float, default=0.0)
    parser.add_argument("--nav-timeout", type=float, default=0.0)
    parser.add_argument("--max-start-error-m", type=float, default=0.20)
    parser.add_argument("--report-dir", type=Path)
    parser.add_argument("--world", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--nav-params", type=Path)
    args = parser.parse_args()
    if args.check_only == args.enable_motion:
        parser.error("select exactly one of --enable-motion or --check-only")
    if args.base_feedback_timeout <= 0.0:
        parser.error("--base-feedback-timeout must be positive")
    if (args.nav_startup_timeout < 0.0 or args.nav_timeout < 0.0 or
            args.max_start_error_m <= 0.0):
        parser.error("timeout values cannot be negative and start error must be positive")

    run = ReturnHomeRun(args)
    try:
        return run.run()
    except KeyboardInterrupt:
        return run.close_with_error(RuntimeError("operator_interrupt"))
    except Exception as error:
        return run.close_with_error(error)


if __name__ == "__main__":
    raise SystemExit(main())
