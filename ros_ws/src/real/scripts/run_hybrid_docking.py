#!/usr/bin/env python3
"""Run the complete guarded VP3-to-unloading-dock sequence."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import yaml


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def heading_delta_degrees(current_yaw: float, target_yaw: float) -> float:
    """Shortest relative rotation from the current map yaw to target yaw."""
    delta = target_yaw - current_yaw
    return math.degrees(math.atan2(math.sin(delta), math.cos(delta)))


def vp3_scan_start_pose(authored_vp3: "Pose2D") -> "Pose2D":
    """Standalone handoff pose after the validated six-view VP3 sweep."""
    return Pose2D(authored_vp3.x, authored_vp3.y, 0.0)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def practical_tag_gate(result: dict) -> bool:
    """Accept a dock pose whose residual yaw can be removed by the flip.

    At reference A, range and bearing determine the robot position.  Trying to
    drive the remaining small tag-normal error to zero at the same time can
    make the differential drive controller stall: bearing and normal error may
    request opposite turns.  The following 180-degree turn can compensate this
    bounded residual without changing the A position.
    """
    final = result.get("final") or {}
    try:
        return (
            abs(float(final["range_error_m"])) <= 0.012 and
            abs(float(final["bearing_error_deg"])) <= 1.10 and
            abs(float(final["normal_yaw_error_deg"])) <= 4.00)
    except (KeyError, TypeError, ValueError):
        return False


def compensated_terminal_rotation(result: dict) -> float:
    """Return the flip angle after removing the bounded tag yaw residual."""
    normal_error = float(result["final"]["normal_yaw_error_deg"])
    if abs(normal_error) > 4.00:
        raise ValueError("tag_normal_error_outside_compensation_gate")
    return 176.7 - normal_error


def safe_backup_continuation(result: dict) -> float | None:
    if result.get("reason") != "motion_timeout":
        return None
    final = result.get("final") or {}
    try:
        remaining = float(final["remaining_m"])
        lidar_error = float(final["rear_median_error_m"])
        heading_error = abs(float(final["heading_error_deg"]))
    except (KeyError, TypeError, ValueError):
        return None
    if not 0.002 < remaining <= 0.040:
        return None
    if not 0.002 < lidar_error <= 0.045:
        return None
    if abs(remaining - lidar_error) > 0.015 or heading_error > 1.5:
        return None
    return remaining


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class DockingBridge(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_docking_orchestrator")
        self.battery_v = None
        self.battery_present = False
        self.battery_at = None
        self.odom_at = None
        self.odom_stamp = None
        self.scan_at = None
        self.image_at = None
        self.last_nav_feedback = None
        self.nav_recoveries = 0
        self.create_subscription(
            BatteryState, "/battery_state", self._battery, 10)
        self.create_subscription(
            Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, "/image_raw/compressed",
            self._image, qos_profile_sensor_data)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.nomotion_client = self.create_client(
            Empty, "/request_nomotion_update")
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in (
                "amcl", "controller_server", "planner_server",
                "bt_navigator", "velocity_smoother")
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _battery(self, message: BatteryState) -> None:
        self.battery_v = float(message.voltage)
        self.battery_present = bool(message.present)
        self.battery_at = time.monotonic()

    def _odom(self, message: Odometry) -> None:
        self.odom_at = time.monotonic()
        self.odom_stamp = message.header.stamp

    def _scan(self, _message: LaserScan) -> None:
        self.scan_at = time.monotonic()

    def _image(self, _message: CompressedImage) -> None:
        self.image_at = time.monotonic()

    def spin_for(self, duration: float) -> None:
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_base_health(self, timeout: float, min_voltage: float) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            fresh = all(stamp is not None and now - stamp < 1.0 for stamp in (
                self.battery_at, self.odom_at, self.scan_at, self.image_at))
            if fresh:
                if not self.battery_present or self.battery_v < min_voltage:
                    raise RuntimeError(
                        f"battery_not_safe:{self.battery_v}")
                return
        now = time.monotonic()
        missing = [
            name for name, stamp in (
                ("battery", self.battery_at), ("odom", self.odom_at),
                ("scan", self.scan_at), ("camera", self.image_at))
            if stamp is None or now - stamp >= 1.0
        ]
        raise RuntimeError(
            "base_feedback_timeout:missing=" + ",".join(missing))

    def cmd_publishers(self) -> list[str]:
        return sorted({
            item.node_name
            for item in self.get_publishers_info_by_topic("/cmd_vel")
        })

    def wait_cmd_idle(self, timeout: float) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.cmd_publishers():
                return
        raise RuntimeError(
            "cmd_vel_publishers_remain:" + ",".join(self.cmd_publishers()))

    def force_zero_velocity(self, duration: float = 1.0) -> None:
        self.wait_cmd_idle(8.0)
        publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_publisher(publisher)
        self.spin_for(0.25)

    def wait_nav_ready(self, timeout: float) -> None:
        started = time.monotonic()
        while (timeout <= 0.0 or
               time.monotonic() - started < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
            nav_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
            amcl_ready = self.nomotion_client.wait_for_service(timeout_sec=0.1)
            lifecycle_ready = all(
                self._lifecycle_active(client)
                for client in self.lifecycle_clients.values())
            if nav_ready and amcl_ready and lifecycle_ready:
                self.spin_for(1.0)
                return
        raise RuntimeError("nav2_startup_timeout")

    def _lifecycle_active(self, client) -> bool:
        if not client.wait_for_service(timeout_sec=0.05):
            return False
        future = client.call_async(GetState.Request())
        end = time.monotonic() + 0.5
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            return False
        return future.result().current_state.label == "active"

    def initialise_near(self, pose: Pose2D) -> Pose2D:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        z, w = quaternion_from_yaw(pose.yaw)
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = math.radians(15.0) ** 2
        def publish_initial_pose() -> None:
            # The TB3 clock has been measured 1.4--1.6 s behind the laptop.
            # Stamp the pose with the robot's latest odometry time; laptop-now
            # makes AMCL extrapolate into the future and can leave the previous
            # (usually Home) TF visible during the handoff.
            if self.odom_stamp is None:
                raise RuntimeError("initial_pose_requires_odom_stamp")
            message.header.stamp.sec = self.odom_stamp.sec
            message.header.stamp.nanosec = self.odom_stamp.nanosec
            self.initial_pose_pub.publish(message)
            self.spin_for(0.30)

        for _ in range(3):
            publish_initial_pose()
        request = self.nomotion_client.call_async(Empty.Request())
        end = time.monotonic() + 4.0
        while not request.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not request.done():
            raise RuntimeError("amcl_nomotion_update_timeout")
        self.spin_for(2.0)
        samples = []
        last_seen = None
        last_republish = time.monotonic()
        end = time.monotonic() + 15.0
        while time.monotonic() < end and len(samples) < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time())
            except Exception:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            candidate = Pose2D(
                float(translation.x), float(translation.y),
                yaw_from_quaternion(rotation))
            last_seen = candidate
            # Do not mistake a stable but stale Home transform for successful
            # VP3 localization.  Wait until the newly requested map pose is
            # visible, then require five mutually stable samples.
            if math.hypot(candidate.x - pose.x, candidate.y - pose.y) <= 0.50:
                samples.append(candidate)
            else:
                samples.clear()
            if time.monotonic() - last_republish >= 2.0:
                publish_initial_pose()
                last_republish = time.monotonic()
            self.spin_for(0.2)
        if len(samples) < 3:
            if last_seen is None:
                raise RuntimeError("map_pose_unavailable")
            miss = math.hypot(last_seen.x - pose.x, last_seen.y - pose.y)
            raise RuntimeError(
                f"initial_pose_not_applied:last_error={miss:.3f}m")
        spread = max(
            math.hypot(a.x - b.x, a.y - b.y)
            for a in samples for b in samples)
        if spread > 0.05:
            raise RuntimeError(f"map_pose_unstable:{spread:.3f}m")
        return samples[-1]

    def navigate(self, target: Pose2D, timeout: float) -> dict:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target.x
        goal.pose.pose.position.y = target.y
        z, w = quaternion_from_yaw(target.yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w

        started = time.monotonic()

        def feedback_callback(message) -> None:
            self.last_nav_feedback = message.feedback
            self.nav_recoveries = max(
                self.nav_recoveries,
                int(message.feedback.number_of_recoveries))

        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=feedback_callback)
        while not send_future.done() and time.monotonic() - started < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send_future.done():
            return {"outcome": "aborted", "reason": "goal_send_timeout"}
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {"outcome": "aborted", "reason": "goal_rejected"}
        result_future = handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (timeout > 0.0 and
                    time.monotonic() - started > timeout):
                cancel = handle.cancel_goal_async()
                while not cancel.done() and time.monotonic() - started < timeout + 3:
                    rclpy.spin_once(self, timeout_sec=0.1)
                return {"outcome": "aborted", "reason": "navigation_timeout"}
        wrapped = result_future.result()
        feedback = self.last_nav_feedback
        pose = None
        if feedback is not None:
            p = feedback.current_pose.pose
            pose = {
                "x": float(p.position.x),
                "y": float(p.position.y),
                "yaw": yaw_from_quaternion(p.orientation),
            }
        return {
            "outcome": "reached" if wrapped.status == GoalStatus.STATUS_SUCCEEDED
                else "aborted",
            "reason": f"nav2_status_{wrapped.status}",
            "elapsed_sec": time.monotonic() - started,
            "recoveries": self.nav_recoveries,
            "final_pose": pose,
        }


class HybridDockingRun:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.share = Path(get_package_share_directory("real"))
        self.world_path = args.world or self.share / "config/world_model_real_v5.yaml"
        self.map_path = args.map or self.share / "maps/lab_arena_v5.yaml"
        self.nav_params = args.nav_params or self.share / "config/nav2_real.yaml"
        self.camera_yaml = args.camera_yaml or self.share / "config/tb3_usb_camera_640x480.yaml"
        self.tag_reference = args.tag_reference or (
            self.share / "config/apriltag_A_left_shift_1p5cm.json")
        self.script_dir = self.share / "scripts"
        self.world = yaml.safe_load(self.world_path.read_text(encoding="utf-8"))
        viewpoints = self.world["areas"]["arena"]["viewpoints"]
        vp3_data = next(item for item in viewpoints if item["id"] == "vp3")
        docking = self.world["areas"]["arena"]["anomaly_handling"][
            "former_vp2_region"]["approach"]["hybrid_docking_reference"]
        coarse = docking["coarse_A"]
        fallback = docking["nav2_handoff_B"]
        final = docking["perfect_stop"]
        self.vp3 = Pose2D(vp3_data["x"], vp3_data["y"], vp3_data["yaw"])
        if args.initial_x is None:
            self.initial_pose = vp3_scan_start_pose(self.vp3)
            self.initial_pose_source = "vp3_scan_start_yaw"
        else:
            self.initial_pose = Pose2D(
                float(args.initial_x), float(args.initial_y),
                float(args.initial_yaw))
            self.initial_pose_source = "runtime_handoff"
        self.coarse_a = Pose2D(coarse["x"], coarse["y"], coarse["yaw"])
        self.fallback_b = Pose2D(fallback["x"], fallback["y"], fallback["yaw"])
        self.reverse_distance = float(final["final_reverse_distance_m"])
        self.rear_reference = float(final["rear_lidar_reference_m"])
        self.nav_process = None
        self.nav_log_handle = None
        self.bridge = None
        self.started = time.monotonic()
        self.report = {
            "schema_version": 1,
            "started_at": datetime.now().astimezone().isoformat(),
            "outcome": "running",
            "reason": "",
            "configuration": {
                "world": str(self.world_path),
                "initial_pose": self.initial_pose.__dict__,
                "initial_pose_source": self.initial_pose_source,
                "base_feedback_timeout_sec": args.base_feedback_timeout,
                "nav_startup_timeout_sec": args.nav_startup_timeout,
                "coarse_A": self.coarse_a.__dict__,
                "fallback_B": self.fallback_b.__dict__,
                "tag_reference": str(self.tag_reference),
                "reverse_distance_m": self.reverse_distance,
                "rear_lidar_reference_m": self.rear_reference,
            },
            "phases": [],
            "arm_unload_triggered": False,
        }
        if args.report_dir:
            self.report_dir = args.report_dir
        else:
            workspace_doc = self.share.parents[3] / "doc"
            self.report_dir = workspace_doc / datetime.now().strftime("%Y%m%d") / "apriltag"
        self.report_dir.mkdir(parents=True, exist_ok=True)
        self.run_id = datetime.now().strftime("hybrid_%Y%m%d_%H%M%S")
        self.report_path = self.report_dir / f"{self.run_id}_report.json"

    def add_phase(self, name: str, started: float, result: dict) -> None:
        self.report["phases"].append({
            "name": name,
            "elapsed_sec": time.monotonic() - started,
            "result": result,
        })
        self.write_report()

    def write_report(self) -> None:
        self.report["elapsed_sec"] = time.monotonic() - self.started
        self.report_path.write_text(
            json.dumps(self.report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")

    def start_nav2(self) -> None:
        nav_log = self.report_dir / f"{self.run_id}_nav2.log"
        self.nav_log_handle = nav_log.open("w", encoding="utf-8")
        command = [
            "ros2", "launch", "nav2_bringup", "bringup_launch.py",
            f"map:={self.map_path}",
            f"params_file:={self.nav_params}",
            "use_sim_time:=false", "autostart:=true", "use_composition:=False",
        ]
        self.nav_process = subprocess.Popen(
            command, stdout=self.nav_log_handle, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        self.report["nav2_log"] = str(nav_log)

    def stop_nav2(self) -> None:
        if self.nav_process is None:
            return
        if self.nav_process.poll() is None:
            os.killpg(self.nav_process.pid, signal.SIGINT)
            try:
                self.nav_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.nav_process.terminate()
                try:
                    self.nav_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.nav_process.kill()
                    self.nav_process.wait(timeout=2)
        if self.nav_log_handle is not None:
            self.nav_log_handle.close()
        self.nav_process = None

    def child(self, name: str, command: list[str], output: Path | None = None) -> dict:
        started = time.monotonic()
        completed = subprocess.run(
            command, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, check=False)
        result = {
            "exit_code": completed.returncode,
            "console": completed.stdout[-4000:],
        }
        if output is not None and output.exists():
            result.update(load_json(output))
            result["output_file"] = str(output)
        self.add_phase(name, started, result)
        return result

    def run(self) -> int:
        if self.args.check_only:
            self.report["outcome"] = "check_only_ok"
            self.report["reason"] = "configuration_loaded_no_motion"
            self.write_report()
            print(json.dumps({
                "outcome": self.report["outcome"],
                "report": str(self.report_path),
                "coarse_A": self.coarse_a.__dict__,
            }, indent=2))
            return 0

        used_fallback = False
        rclpy.init()
        self.bridge = DockingBridge()
        try:
            # A fresh FastDDS participant starts immediately after the patrol
            # Nav2 stack is torn down.  On the hotspot, rediscovering all four
            # robot-side publishers has repeatedly taken longer than 8 s even
            # while the remote processes remained healthy.  This is a bounded
            # pre-motion wait, so allow the measured discovery window.
            self.bridge.wait_base_health(
                self.args.base_feedback_timeout, self.args.min_voltage)
            self.bridge.wait_cmd_idle(3.0)
            if self.args.resume_from_a:
                started = time.monotonic()
                self.add_phase("resume_from_A_preflight", started, {
                    "battery_v": self.bridge.battery_v,
                    "image_stream_fresh": True,
                    "cmd_vel_publishers": [],
                })
            else:
                started = time.monotonic()
                self.start_nav2()
                self.bridge.wait_nav_ready(self.args.nav_startup_timeout)
                # A completed VP3 camera sweep normally ends at its sixth scan
                # yaw, not the authored VP3 entry yaw.  A full-workflow handoff
                # therefore supplies the live map pose captured after the
                # operator has loaded the tray.  Standalone VP3 tests retain the
                # historical authored-pose default.
                initial = self.bridge.initialise_near(self.initial_pose)
                distance = math.hypot(
                    initial.x - self.vp3.x, initial.y - self.vp3.y)
                initial_result = {
                    "pose": initial.__dict__, "distance_from_vp3_m": distance,
                    "initial_pose_source": self.initial_pose_source,
                    "requested_initial_pose": self.initial_pose.__dict__,
                    "battery_v": self.bridge.battery_v,
                    "image_stream_fresh": True,
                }
                self.add_phase("localize_near_vp3", started, initial_result)
                if distance > self.args.max_start_error_m:
                    raise RuntimeError(f"start_not_near_vp3:{distance:.3f}m")

                if self.args.localize_only:
                    self.report["outcome"] = "localization_check_ok"
                    self.report["reason"] = "vp3_initial_pose_applied_no_motion"
                    self.write_report()
                    print(json.dumps({
                        "outcome": self.report["outcome"],
                        "reason": self.report["reason"],
                        "pose": initial.__dict__,
                        "distance_from_vp3_m": distance,
                        "report": str(self.report_path),
                    }, indent=2))
                    return 0

                started = time.monotonic()
                navigation = self.bridge.navigate(
                    self.coarse_a, self.args.nav_timeout)
                self.add_phase("nav2_to_coarse_A", started, navigation)
                used_fallback = navigation["outcome"] != "reached"
                if used_fallback:
                    started = time.monotonic()
                    fallback = self.bridge.navigate(
                        self.fallback_b, self.args.nav_timeout)
                    self.add_phase("nav2_fallback_to_B", started, fallback)
                    if fallback["outcome"] != "reached":
                        raise RuntimeError("nav2_coarse_A_and_B_failed")
        finally:
            self.stop_nav2()
            if self.bridge is not None:
                try:
                    self.bridge.force_zero_velocity()
                finally:
                    self.bridge.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        # PositionGoalChecker intentionally ignores yaw for patrol viewpoints.
        # Parking is different: the camera must face the wall-mounted tag.
        # Correct the terminal heading explicitly after Nav2 has released
        # /cmd_vel, using the final map heading only to calculate a relative
        # odometry rotation.
        if self.args.resume_from_a:
            started = time.monotonic()
            self.add_phase("align_camera_to_apriltag", started, {
                "outcome": "operator_aligned_resume",
            })
        else:
            terminal_pose = self.fallback_b if used_fallback else self.coarse_a
            terminal_navigation = fallback if used_fallback else navigation
            final_pose = terminal_navigation.get("final_pose") or {}
            if "yaw" not in final_pose:
                raise RuntimeError("terminal_navigation_heading_missing")
            heading_correction = heading_delta_degrees(
                float(final_pose["yaw"]), terminal_pose.yaw)
            if abs(heading_correction) > 1.0:
                heading_align = self.child("align_camera_to_apriltag", [
                    sys.executable,
                    str(self.script_dir / "rotate_odom_test.py"),
                    "--odom-topic", "/odom", "--cmd-topic", "/cmd_vel",
                    "--degrees", f"{heading_correction:.6f}",
                    "--max-speed", "0.25", "--min-speed", "0.05",
                    "--tolerance-deg", "1.0", "--timeout", "35",
                    "--battery-topic", "/battery_state",
                    "--min-voltage", str(self.args.min_voltage),
                ])
                if heading_align["exit_code"] != 0:
                    raise RuntimeError("terminal_heading_alignment_failed")
            else:
                started = time.monotonic()
                self.add_phase("align_camera_to_apriltag", started, {
                    "outcome": "already_aligned",
                    "heading_correction_deg": heading_correction,
                })

        servo_runtime = self.args.servo_timeout
        servo = None
        recoverable_servo_reasons = {
            "motion_timeout", "tag_lost", "image_stream_timeout",
            "preflight_timeout",
        }
        for attempt in range(1, 4):
            servo_output = self.report_dir / (
                f"{self.run_id}_tag_servo_{attempt}.json")
            servo = self.child(f"apriltag_terminal_{attempt}", [
                sys.executable,
                str(self.script_dir / "apriltag_visual_servo.py"),
                "--enable-motion", "--reference", str(self.tag_reference),
                "--camera-yaml", str(self.camera_yaml),
                "--output", str(servo_output),
                "--min-voltage", str(self.args.min_voltage),
                "--max-runtime", str(servo_runtime),
                "--preflight-timeout", "30.0",
                "--max-angular", "0.12", "--max-linear", "0.035",
                "--max-odom-displacement-m",
                "0.35" if used_fallback else "0.12",
                "--max-initial-close-error-m", "0.12",
                "--preflight-stable-sec", "0.50",
                "--goal-stable-sec", "0.50",
                "--bearing-tolerance-deg", "1.10",
                "--normal-tolerance-deg", "4.00",
            ], servo_output)
            if servo.get("outcome") == "reached" or practical_tag_gate(servo):
                break
            if servo.get("reason") not in recoverable_servo_reasons:
                raise RuntimeError(
                    "apriltag_terminal_failed:" + str(servo.get("reason")))
            emergency_zero_velocity(0.8)
        if (servo is None or
                (servo.get("outcome") != "reached" and
                 not practical_tag_gate(servo))):
            raise RuntimeError(
                "apriltag_terminal_recovery_exhausted:" +
                str(None if servo is None else servo.get("reason")))

        rotation_degrees = compensated_terminal_rotation(servo)
        rotate = self.child("rotate_for_reverse", [
            sys.executable, str(self.script_dir / "rotate_odom_test.py"),
            "--odom-topic", "/odom", "--cmd-topic", "/cmd_vel",
            "--degrees", f"{rotation_degrees:.6f}", "--max-speed", "0.30",
            "--min-speed", "0.06", "--tolerance-deg", "0.8",
            "--timeout", "20", "--battery-topic", "/battery_state",
            "--min-voltage", str(self.args.min_voltage),
        ])
        if rotate["exit_code"] != 0:
            raise RuntimeError("terminal_rotation_failed")
        match = re.search(r"final_yaw_deg=([-+0-9.]+)", rotate["console"])
        if match is None:
            raise RuntimeError("terminal_rotation_heading_missing")
        target_heading = float(match.group(1))

        remaining = self.reverse_distance
        backup = None
        for attempt in range(1, 4):
            output = self.report_dir / f"{self.run_id}_backup_{attempt}.json"
            backup = self.child(f"final_reverse_{attempt}", [
                sys.executable, str(self.script_dir / "backup_odom_lidar.py"),
                "--enable-motion", "--distance-m", f"{remaining:.6f}",
                "--target-heading-deg", f"{target_heading:.6f}",
                "--target-rear-median-m", f"{self.rear_reference:.6f}",
                "--output", str(output), "--min-voltage", str(self.args.min_voltage),
                "--min-linear", "0.012", "--max-linear-near", "0.015",
                # The measured 10 cm reverse can legitimately take slightly
                # over 12 s at the precision speed.  Keep a generous stall
                # ceiling; odom/lidar/battery safety gates remain continuous.
                "--max-runtime", "30",
            ], output)
            if backup.get("outcome") == "reached":
                break
            continuation = safe_backup_continuation(backup)
            if continuation is None:
                raise RuntimeError(
                    "final_reverse_failed:" + str(backup.get("reason")))
            remaining = continuation
        if backup is None or backup.get("outcome") != "reached":
            raise RuntimeError("final_reverse_recovery_exhausted")

        self.report["outcome"] = "reached_pending_operator_validation"
        self.report["reason"] = "all_automatic_gates_passed"
        self.report["final_heading_deg_odom"] = target_heading
        emergency_zero_velocity()
        self.write_report()
        print(json.dumps({
            "outcome": self.report["outcome"],
            "elapsed_sec": self.report["elapsed_sec"],
            "report": str(self.report_path),
            "arm_unload_triggered": False,
        }, indent=2))
        return 0

    def close_with_error(self, error: Exception) -> int:
        self.stop_nav2()
        stop_error = None
        try:
            emergency_zero_velocity()
        except Exception as stopping_error:
            stop_error = str(stopping_error)
        self.report["outcome"] = "aborted"
        self.report["reason"] = str(error)
        self.report["emergency_stop"] = {
            "attempted": True,
            "error": stop_error,
        }
        self.write_report()
        print(json.dumps({
            "outcome": "aborted", "reason": str(error),
            "report": str(self.report_path),
        }, indent=2), file=sys.stderr)
        return 2


def emergency_zero_velocity(duration: float = 1.0) -> None:
    started_context = False
    if not rclpy.ok():
        rclpy.init()
        started_context = True
    node = Node("hybrid_docking_emergency_stop")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    try:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if started_context and rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Guarded one-command VP3-to-final-dock sequence")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--start-near-vp3", action="store_true")
    parser.add_argument("--resume-from-a", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument(
        "--localize-only", action="store_true",
        help="apply and verify the VP3 AMCL pose without sending a motion goal")
    parser.add_argument("--min-voltage", type=float, default=11.0)
    parser.add_argument("--base-feedback-timeout", type=float, default=30.0)
    parser.add_argument(
        "--nav-startup-timeout", type=float, default=0.0,
        help="Nav2 activation deadline; 0 waits until lifecycle is active")
    parser.add_argument("--max-start-error-m", type=float, default=0.35)
    parser.add_argument(
        "--nav-timeout", type=float, default=0.0,
        help="Nav2 phase deadline in seconds; 0 disables the external deadline")
    parser.add_argument(
        "--servo-timeout", type=float, default=0.0,
        help="visual-servo motion deadline; 0 keeps calibrating until success")
    parser.add_argument("--report-dir", type=Path)
    parser.add_argument("--world", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--nav-params", type=Path)
    parser.add_argument("--camera-yaml", type=Path)
    parser.add_argument("--tag-reference", type=Path)
    parser.add_argument("--initial-x", type=float)
    parser.add_argument("--initial-y", type=float)
    parser.add_argument("--initial-yaw", type=float)
    args = parser.parse_args()
    if args.check_only and (args.enable_motion or args.localize_only):
        parser.error(
            "--check-only cannot be combined with motion/localization modes")
    if args.localize_only and args.enable_motion:
        parser.error("--localize-only and --enable-motion are mutually exclusive")
    if not args.check_only and not args.enable_motion and not args.localize_only:
        parser.error("select --enable-motion or --localize-only")
    if ((args.enable_motion or args.localize_only) and
            (args.start_near_vp3 == args.resume_from_a)):
        parser.error(
            "select exactly one of --start-near-vp3 or --resume-from-a")
    if args.localize_only and args.resume_from_a:
        parser.error("--localize-only requires --start-near-vp3")
    initial_values = (args.initial_x, args.initial_y, args.initial_yaw)
    if any(value is not None for value in initial_values) and not all(
            value is not None for value in initial_values):
        parser.error(
            "--initial-x, --initial-y and --initial-yaw must be supplied together")
    if args.resume_from_a and any(value is not None for value in initial_values):
        parser.error("runtime initial pose is only valid with --start-near-vp3")
    if args.base_feedback_timeout <= 0.0:
        parser.error("--base-feedback-timeout must be positive")
    if (args.nav_startup_timeout < 0.0 or args.nav_timeout < 0.0 or
            args.servo_timeout < 0.0):
        parser.error("Nav2/servo timeout values cannot be negative")

    run = HybridDockingRun(args)
    try:
        return run.run()
    except KeyboardInterrupt:
        return run.close_with_error(RuntimeError("operator_interrupt"))
    except Exception as error:
        return run.close_with_error(error)


if __name__ == "__main__":
    raise SystemExit(main())
