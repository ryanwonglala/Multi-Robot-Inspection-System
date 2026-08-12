#!/usr/bin/env python3
"""Run the guarded real-TB3 inspection-to-unloading workflow.

The motion sequence is deliberately split by an acknowledged operator gate:

    Home -> VP1/VP2/VP3 inspection -> publish report -> WAIT AT VP3
         -> operator loads tray and calls continue -> hybrid VP4 docking

No timeout automatically crosses the operator gate.  The continue request is
accepted only while the workflow is waiting, the battery is healthy, and the
live map pose is still inside the VP3 handoff radius.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty as EmptyMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml

from run_hybrid_docking import (
    DockingBridge,
    Pose2D,
    emergency_zero_velocity,
    yaw_from_quaternion,
)


CONTINUE_SERVICE = "/inspection_workflow/continue_to_dock"
CONTINUE_TOPIC = "/inspection_workflow/continue"
STATE_TOPIC = "/inspection_workflow/state"


def canonical_vp_label(value) -> str:
    label = str(value or "").strip().lower()
    aliases = {
        "vp1": "vp1", "viewpoint_1": "vp1",
        "vp2": "vp2", "viewpoint_2": "vp2",
        "vp3": "vp3", "viewpoint_3": "vp3",
    }
    return aliases.get(label, label)


def inspection_ready_for_handoff(report: dict) -> tuple[bool, str]:
    """Require complete VP1--VP3 evidence before exposing the dock gate."""
    if report.get("status") != "completed":
        return False, f"inspection_status_{report.get('status')}"
    if report.get("route") != ["arena"]:
        return False, "unexpected_inspection_route"
    areas = report.get("areas") or []
    if len(areas) != 1 or areas[0].get("target_area") != "arena":
        return False, "arena_result_missing"
    area = areas[0]
    if area.get("status") != "checked":
        return False, f"arena_status_{area.get('status')}"
    labels = {
        canonical_vp_label(stop.get("label"))
        for stop in area.get("selected_stops", [])
    }
    if labels != {"vp1", "vp2", "vp3"}:
        return False, "vp1_vp2_vp3_not_all_visited"
    samples = area.get("scan_samples") or []
    sampled_labels = {
        canonical_vp_label(sample.get("stop_label")) for sample in samples
    }
    if sampled_labels != {"vp1", "vp2", "vp3"}:
        return False, "vp1_vp2_vp3_not_all_scanned"
    counts = {
        label: sum(
            canonical_vp_label(sample.get("stop_label")) == label
            for sample in samples)
        for label in ("vp1", "vp2", "vp3")
    }
    if any(count < 6 for count in counts.values()):
        return False, "incomplete_six_direction_scan"
    return True, "inspection_complete_at_vp3"


def summarize_inspection(report: dict) -> dict:
    anomalies = report.get("anomalies") or []
    return {
        "status": report.get("status"),
        "anomaly_count": len(anomalies),
        "anomalies": anomalies,
        "checked_count": (report.get("summary") or {}).get("checked_count"),
        "run_dir": report.get("run_dir"),
    }


def newest_new_report(root: Path, before: set[Path]) -> Path | None:
    candidates = [
        path for path in root.glob("inspection_*/details.yaml")
        if path not in before
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime) \
        if candidates else None


def retryable_docking_reason(reason) -> bool:
    """Only retry a docking failure known to happen before motion starts."""
    return str(reason or "").startswith("base_feedback_timeout")


def build_inspection_command(
        inspection_config: Path, world: Path) -> list[str]:
    return [
        "ros2", "run", "task_layer", "inspection_runner.py",
        "--ros-args", "--params-file", str(inspection_config),
        "-p", f"world_model_path:={world}",
        "-p", "route:=arena",
        "-p", "return_home:=false",
    ]


def build_docking_command(
        report_dir: Path, pose: Pose2D, max_start_error_m: float) -> list[str]:
    return [
        "ros2", "run", "real", "run_hybrid_docking.py",
        "--enable-motion", "--start-near-vp3",
        "--initial-x", f"{pose.x:.9f}",
        "--initial-y", f"{pose.y:.9f}",
        "--initial-yaw", f"{pose.yaw:.9f}",
        "--max-start-error-m", f"{max_start_error_m:.3f}",
        "--base-feedback-timeout", "30.0",
        "--nav-startup-timeout", "0.0",
        "--nav-timeout", "0.0",
        "--servo-timeout", "0.0",
        "--report-dir", str(report_dir),
    ]


def stop_process(process: subprocess.Popen | None, timeout: float = 10.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    process.terminate()
    try:
        process.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


class OperatorGate(Node):
    def __init__(
            self, vp3: Pose2D, max_distance_m: float,
            min_voltage: float, summary: dict) -> None:
        super().__init__("inspection_workflow_gate")
        self.vp3 = vp3
        self.max_distance_m = max_distance_m
        self.min_voltage = min_voltage
        self.summary = summary
        self.stage = "awaiting_operator_load"
        self.continue_requested = False
        self.continue_source = None
        self.accepted_pose: Pose2D | None = None
        self.battery_v = None
        self.battery_present = False
        self.battery_at = None
        self.odom_at = None
        self.last_pose: Pose2D | None = None

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self.state_pub = self.create_publisher(String, STATE_TOPIC, latched)
        self.create_service(Trigger, CONTINUE_SERVICE, self._continue_service)
        self.create_subscription(
            EmptyMessage, CONTINUE_TOPIC, self._continue_topic, 10)
        self.create_subscription(BatteryState, "/battery_state", self._battery, 10)
        self.create_subscription(
            Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self._timer)

    def _battery(self, message: BatteryState) -> None:
        self.battery_v = float(message.voltage)
        self.battery_present = bool(message.present)
        self.battery_at = time.monotonic()

    def _odom(self, _message: Odometry) -> None:
        self.odom_at = time.monotonic()

    def sample_pose(self) -> Pose2D | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", Time())
        except Exception:
            return self.last_pose
        translation = transform.transform.translation
        self.last_pose = Pose2D(
            float(translation.x), float(translation.y),
            yaw_from_quaternion(transform.transform.rotation))
        return self.last_pose

    def state_payload(self) -> dict:
        pose = self.sample_pose()
        return {
            "stage": self.stage,
            "continue_service": CONTINUE_SERVICE,
            "continue_topic": CONTINUE_TOPIC,
            "battery_v": self.battery_v,
            "pose": None if pose is None else pose.__dict__,
            "vp3_distance_m": None if pose is None else math.hypot(
                pose.x - self.vp3.x, pose.y - self.vp3.y),
            "inspection": self.summary,
        }

    def _timer(self) -> None:
        message = String()
        message.data = json.dumps(
            self.state_payload(), ensure_ascii=False, separators=(",", ":"))
        self.state_pub.publish(message)

    def request_continue(self, source: str) -> tuple[bool, str]:
        if self.stage != "awaiting_operator_load":
            return False, f"workflow_stage_is_{self.stage}"
        now = time.monotonic()
        if (self.battery_at is None or now - self.battery_at > 2.0 or
                not self.battery_present or self.battery_v is None or
                self.battery_v < self.min_voltage):
            return False, "battery_feedback_not_safe"
        if self.odom_at is None or now - self.odom_at > 2.0:
            return False, "odometry_not_fresh"
        pose = self.sample_pose()
        if pose is None:
            return False, "map_pose_unavailable"
        distance = math.hypot(pose.x - self.vp3.x, pose.y - self.vp3.y)
        if distance > self.max_distance_m:
            return False, f"robot_not_in_vp3_handoff_radius:{distance:.3f}m"
        self.accepted_pose = pose
        self.continue_requested = True
        self.continue_source = source
        self.stage = "continue_accepted"
        self._timer()
        return True, (
            f"continue accepted at x={pose.x:.3f}, y={pose.y:.3f}, "
            f"yaw={pose.yaw:.3f}")

    def _continue_service(self, _request, response):
        response.success, response.message = self.request_continue("service")
        return response

    def _continue_topic(self, _message: EmptyMessage) -> None:
        accepted, reason = self.request_continue("topic")
        if accepted:
            self.get_logger().info(reason)
        else:
            self.get_logger().warning("continue rejected: " + reason)


class FullWorkflowRun:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.share = Path(get_package_share_directory("real"))
        self.world_path = args.world or (
            self.share / "config/world_model_real_v5.yaml")
        self.map_path = args.map or self.share / "maps/lab_arena_v5.yaml"
        self.nav_params = args.nav_params or (
            self.share / "config/nav2_real.yaml")
        self.inspection_config = args.inspection_config or (
            self.share / "config/inspection_real_v5.yaml")
        self.world = yaml.safe_load(
            self.world_path.read_text(encoding="utf-8"))
        self.inspection_settings = yaml.safe_load(
            self.inspection_config.read_text(encoding="utf-8"))
        home = self.world["robot_start"]["pose"]
        viewpoints = self.world["areas"]["arena"]["viewpoints"]
        vp3 = next(item for item in viewpoints if item["id"] == "vp3")
        self.home = Pose2D(home["x"], home["y"], home["yaw"])
        self.vp3 = Pose2D(vp3["x"], vp3["y"], vp3["yaw"])
        params = self.inspection_settings["inspection_runner"]["ros__parameters"]
        self.inspection_root = Path(params["report_dir"]).expanduser()

        workspace = self.share.parents[4]
        report_root = args.report_root or workspace / "reports/full_workflow"
        self.run_id = datetime.now().strftime("workflow_%Y%m%d_%H%M%S")
        self.run_dir = Path(report_root).expanduser() / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.report_path = self.run_dir / "workflow_report.json"
        self.markdown_path = self.run_dir / "workflow_report.md"
        self.started = time.monotonic()
        self.report = {
            "schema_version": 1,
            "run_id": self.run_id,
            "started_at": datetime.now().astimezone().isoformat(),
            "status": "created",
            "stage": "configuration",
            "configuration": {
                "world": str(self.world_path),
                "map": str(self.map_path),
                "nav_params": str(self.nav_params),
                "inspection_config": str(self.inspection_config),
                "inspection_route": ["vp1", "vp2", "vp3"],
                "return_home_after_inspection": False,
                "operator_gate_service": CONTINUE_SERVICE,
                "operator_gate_topic": CONTINUE_TOPIC,
                "operator_gate_enter": not args.service_only,
                "inspection_only": args.inspection_only,
                "vp3_handoff_radius_m": args.vp3_handoff_radius,
            },
            "phases": [],
            "inspection": None,
            "operator_gate": None,
            "docking": None,
            "arm_unload_triggered": False,
        }
        self.processes: dict[str, tuple[subprocess.Popen, object]] = {}
        self.active_process: subprocess.Popen | None = None
        self.rclpy_started = False
        self.write_report()

    def write_report(self) -> None:
        self.report["elapsed_sec"] = time.monotonic() - self.started
        self.report_path.write_text(
            json.dumps(self.report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        inspection = self.report.get("inspection") or {}
        docking = self.report.get("docking") or {}
        lines = [
            "# TB3 Full Workflow Report",
            "",
            f"- Status: `{self.report.get('status')}`",
            f"- Stage: `{self.report.get('stage')}`",
            f"- Inspection report: `{inspection.get('report_path')}`",
            f"- Anomaly count: `{inspection.get('anomaly_count')}`",
            f"- Docking report: `{docking.get('report_path')}`",
            f"- Arm unload triggered: `{self.report.get('arm_unload_triggered')}`",
        ]
        self.markdown_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def add_phase(self, name: str, started: float, result: dict) -> None:
        self.report["phases"].append({
            "name": name,
            "elapsed_sec": time.monotonic() - started,
            "result": result,
        })
        self.write_report()

    def start_background(self, name: str, command: list[str]) -> None:
        log_path = self.run_dir / f"{name}.log"
        handle = log_path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            command, stdout=handle, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        self.processes[name] = (process, handle)

    def stop_background(self, name: str) -> None:
        item = self.processes.pop(name, None)
        if item is None:
            return
        process, handle = item
        stop_process(process)
        handle.close()

    def run_logged(self, command: list[str], log_path: Path) -> int:
        with log_path.open("w", encoding="utf-8") as handle:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
            self.active_process = process
            assert process.stdout is not None
            try:
                for line in process.stdout:
                    print(line, end="", flush=True)
                    handle.write(line)
                    handle.flush()
                return process.wait()
            finally:
                if process.poll() is None:
                    stop_process(process)
                self.active_process = None

    def validate_configuration(self) -> None:
        for path in (
                self.world_path, self.map_path, self.nav_params,
                self.inspection_config):
            if not path.exists():
                raise FileNotFoundError(path)
        viewpoints = self.world["areas"]["arena"]["viewpoints"]
        if [item["id"] for item in viewpoints] != ["vp1", "vp2", "vp3"]:
            raise RuntimeError("world model patrol route is not VP1,VP2,VP3")

    def start_nav2(self) -> None:
        self.start_background("nav2", [
            "ros2", "launch", "nav2_bringup", "bringup_launch.py",
            f"map:={self.map_path}",
            f"params_file:={self.nav_params}",
            "use_sim_time:=false", "autostart:=true",
            "use_composition:=False",
        ])

    def wait_and_dock(self, summary: dict, inspection_path: Path) -> int:
        """Hold at VP3 until an acknowledged Enter/service, then dock."""
        gate = OperatorGate(
            self.vp3, self.args.vp3_handoff_radius,
            self.args.min_voltage, summary)
        self.report["status"] = "waiting_operator"
        self.report["stage"] = "awaiting_operator_load_at_vp3"
        self.report["operator_gate"] = {
            "status": "waiting",
            "service": CONTINUE_SERVICE,
            "topic": CONTINUE_TOPIC,
            "enter_enabled": (
                not self.args.service_only and sys.stdin.isatty()),
            "continue_command": (
                "ros2 service call " + CONTINUE_SERVICE +
                " std_srvs/srv/Trigger '{}'"),
        }
        self.write_report()
        print("\nTB3 is stationary at VP3. Load the recovered cubes onto the tray.")
        enter_enabled = not self.args.service_only and sys.stdin.isatty()
        if enter_enabled:
            print("After all people leave the arena, press ENTER here to dock.",
                  flush=True)
        else:
            print("This process has no interactive terminal; continue with:")
            print(
                "  ros2 service call " + CONTINUE_SERVICE +
                " std_srvs/srv/Trigger '{}'", flush=True)

        gate_started = time.monotonic()
        deadline = (
            gate_started + self.args.operator_timeout
            if self.args.operator_timeout > 0.0 else None)
        try:
            while rclpy.ok() and not gate.continue_requested:
                rclpy.spin_once(gate, timeout_sec=0.2)
                if enter_enabled:
                    readable, _writable, _exceptional = select.select(
                        [sys.stdin], [], [], 0.0)
                    if readable:
                        sys.stdin.readline()
                        accepted, message = gate.request_continue("enter")
                        print(("CONTINUE_ACCEPTED: " if accepted else
                               "CONTINUE_REJECTED: ") + message, flush=True)
                if deadline is not None and time.monotonic() >= deadline:
                    raise RuntimeError("operator_continue_timeout")
        finally:
            gate_pose = gate.accepted_pose
            gate_source = gate.continue_source
            gate.destroy_node()
        if gate_pose is None:
            raise RuntimeError("continue_accepted_without_map_pose")
        self.report["operator_gate"] = {
            **self.report["operator_gate"],
            "status": "accepted",
            "source": gate_source,
            "wait_elapsed_sec": time.monotonic() - gate_started,
            "handoff_pose": gate_pose.__dict__,
        }
        self.add_phase(
            "operator_load_gate", gate_started, self.report["operator_gate"])

        self.report["status"] = "running"
        self.report["stage"] = "vp3_to_final_dock"
        self.write_report()
        self.stop_background("nav2")
        emergency_zero_velocity()
        if rclpy.ok():
            rclpy.shutdown()
        self.rclpy_started = False

        docking_dir = self.run_dir / "docking"
        docking_dir.mkdir(parents=True, exist_ok=True)
        docking_attempts = []
        docking_code = 2
        docking_path = None
        docking_report = {}
        for attempt in range(1, 4):
            docking_started = time.monotonic()
            reports_before = set(docking_dir.glob("hybrid_*_report.json"))
            docking_code = self.run_logged(
                build_docking_command(
                    docking_dir, gate_pose, self.args.vp3_handoff_radius),
                self.run_dir / f"docking_console_attempt_{attempt}.log")
            reports_after = [
                path for path in docking_dir.glob("hybrid_*_report.json")
                if path not in reports_before
            ]
            docking_path = max(
                reports_after, key=lambda path: path.stat().st_mtime) \
                if reports_after else None
            docking_report = (
                json.loads(docking_path.read_text(encoding="utf-8"))
                if docking_path is not None else {})
            attempt_result = {
                "attempt": attempt,
                "exit_code": docking_code,
                "outcome": docking_report.get("outcome"),
                "reason": docking_report.get("reason"),
                "report_path": (
                    None if docking_path is None else str(docking_path)),
            }
            docking_attempts.append(attempt_result)
            self.add_phase(
                f"vp3_to_final_dock_attempt_{attempt}",
                docking_started, attempt_result)
            if (docking_code == 0 and docking_report.get("outcome") ==
                    "reached_pending_operator_validation"):
                break
            if (attempt < 3 and
                    retryable_docking_reason(docking_report.get("reason"))):
                print(
                    "Docking base discovery was not ready; robot has not "
                    f"moved. Retrying preflight ({attempt + 1}/3)...",
                    flush=True)
                emergency_zero_velocity()
                time.sleep(3.0)
                continue
            break
        self.report["docking"] = {
            "exit_code": docking_code,
            "outcome": docking_report.get("outcome"),
            "reason": docking_report.get("reason"),
            "report_path": None if docking_path is None else str(docking_path),
            "attempts": docking_attempts,
        }
        self.write_report()
        if (docking_code != 0 or
                docking_report.get("outcome") !=
                "reached_pending_operator_validation"):
            raise RuntimeError("hybrid_docking_failed")

        self.report["status"] = "completed"
        self.report["stage"] = "final_stop_reached"
        self.report["arm_unload_triggered"] = False
        self.write_report()
        print(json.dumps({
            "outcome": "completed",
            "workflow_report": str(self.report_path),
            "inspection_report": str(inspection_path),
            "docking_report": str(docking_path),
            "anomaly_count": summary["anomaly_count"],
            "arm_unload_triggered": False,
        }, indent=2, ensure_ascii=False))
        return 0

    def run(self) -> int:
        self.validate_configuration()
        if self.args.check_only:
            self.report["status"] = "check_only_ok"
            self.report["stage"] = "no_motion"
            self.write_report()
            print(json.dumps({
                "outcome": "check_only_ok",
                "workflow_report": str(self.report_path),
                "inspection_command": build_inspection_command(
                    self.inspection_config, self.world_path),
                "continue_command": (
                    "ros2 service call " + CONTINUE_SERVICE +
                    " std_srvs/srv/Trigger '{}'"),
            }, indent=2, ensure_ascii=False))
            return 0

        rclpy.init()
        self.rclpy_started = True
        bridge = DockingBridge()
        try:
            started = time.monotonic()
            bridge.wait_base_health(10.0, self.args.min_voltage)
            bridge.wait_cmd_idle(3.0)
            if bridge.nav_client.wait_for_server(timeout_sec=0.5):
                raise RuntimeError(
                    "nav2_already_running; close the previous Nav2/RViz motion "
                    "session before starting the full workflow")
            self.start_nav2()
            bridge.wait_nav_ready(0.0)
            localized = bridge.initialise_near(self.home)
            home_error = math.hypot(
                localized.x - self.home.x, localized.y - self.home.y)
            if home_error > self.args.home_start_radius:
                raise RuntimeError(f"start_not_at_home:{home_error:.3f}m")
            self.add_phase("home_preflight_and_localization", started, {
                "battery_v": bridge.battery_v,
                "pose": localized.__dict__,
                "home_error_m": home_error,
            })
        finally:
            # Its persistent camera subscription is useful for preflight but
            # would consume the Wi-Fi bandwidth inspection_runner intentionally
            # reserves for momentary photo bursts.
            bridge.destroy_node()

        self.start_background("anomaly_texture", [
            "ros2", "run", "task_layer", "anomaly_texture_node.py"])
        if self.args.use_rviz:
            self.start_background("rviz", [
                "rviz2", "-d", str(self.share / "rviz/localization_view.rviz")])

        before = set(self.inspection_root.glob("inspection_*/details.yaml"))
        self.report["stage"] = "vp1_vp2_vp3_inspection"
        self.write_report()
        started = time.monotonic()
        inspection_code = self.run_logged(
            build_inspection_command(self.inspection_config, self.world_path),
            self.run_dir / "inspection_console.log")
        inspection_path = newest_new_report(self.inspection_root, before)
        if inspection_path is None:
            raise RuntimeError("inspection_report_not_created")
        inspection_report = yaml.safe_load(
            inspection_path.read_text(encoding="utf-8"))
        ready, reason = inspection_ready_for_handoff(inspection_report)
        summary = summarize_inspection(inspection_report)
        self.report["inspection"] = {
            **summary,
            "exit_code": inspection_code,
            "report_path": str(inspection_path),
            "handoff_ready": ready,
            "handoff_reason": reason,
        }
        self.add_phase("vp1_vp2_vp3_inspection", started, self.report["inspection"])
        print("\nINSPECTION_RESULT", flush=True)
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
        if inspection_code != 0 or not ready:
            raise RuntimeError("inspection_not_ready_for_vp3_handoff:" + reason)

        if self.args.inspection_only:
            self.report["status"] = "completed"
            self.report["stage"] = "inspection_complete_at_vp3"
            self.report["operator_gate"] = {"status": "not_requested"}
            self.report["docking"] = {"status": "deferred_to_separate_run"}
            self.write_report()
            print(json.dumps({
                "outcome": "inspection_completed_at_vp3",
                "workflow_report": str(self.report_path),
                "inspection_report": str(inspection_path),
                "anomaly_count": summary["anomaly_count"],
                "next_step": "run_hybrid_docking.py --start-near-vp3",
            }, indent=2, ensure_ascii=False), flush=True)
            return 0

        return self.wait_and_dock(summary, inspection_path)

    def close(self) -> None:
        stop_process(self.active_process)
        for name in list(self.processes):
            self.stop_background(name)
        if self.rclpy_started and rclpy.ok():
            rclpy.shutdown()
            self.rclpy_started = False
        if self.args.enable_motion:
            try:
                emergency_zero_velocity()
            except Exception as error:
                self.report["final_stop_error"] = str(error)

    def close_with_error(self, error: Exception) -> int:
        self.report["status"] = "aborted"
        self.report["reason"] = str(error)
        self.report["stage"] = "stopped"
        self.close()
        self.write_report()
        print(json.dumps({
            "outcome": "aborted",
            "reason": str(error),
            "workflow_report": str(self.report_path),
        }, indent=2, ensure_ascii=False), file=sys.stderr)
        return 2


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="VP1-VP3 inspection, operator load gate and VP4 docking")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--start-at-home", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--use-rviz", action="store_true")
    parser.add_argument(
        "--inspection-only", action="store_true",
        help=("finish after the VP1-VP3 report, stop Nav2, and leave the "
              "robot stationary at VP3 for a separate docking run"))
    parser.add_argument(
        "--service-only", action="store_true",
        help="disable same-terminal ENTER and require the ROS continue service")
    parser.add_argument("--min-voltage", type=float, default=11.0)
    parser.add_argument("--home-start-radius", type=float, default=0.15)
    parser.add_argument("--vp3-handoff-radius", type=float, default=0.25)
    parser.add_argument("--operator-timeout", type=float, default=0.0)
    parser.add_argument("--report-root", type=Path)
    parser.add_argument("--world", type=Path)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--nav-params", type=Path)
    parser.add_argument("--inspection-config", type=Path)
    args = parser.parse_args(argv)
    if args.check_only and args.enable_motion:
        parser.error("--check-only and --enable-motion are mutually exclusive")
    if not args.check_only and not args.enable_motion:
        parser.error("--enable-motion is required for a real run")
    if args.enable_motion and not args.start_at_home:
        parser.error("--start-at-home is required for a real run")
    if args.operator_timeout < 0.0:
        parser.error("--operator-timeout must be zero (unlimited) or positive")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    run = FullWorkflowRun(args)
    try:
        return run.run()
    except KeyboardInterrupt:
        return run.close_with_error(RuntimeError("operator_interrupt"))
    except Exception as error:
        return run.close_with_error(error)
    finally:
        if run.report.get("status") == "completed":
            run.close()


if __name__ == "__main__":
    raise SystemExit(main())
