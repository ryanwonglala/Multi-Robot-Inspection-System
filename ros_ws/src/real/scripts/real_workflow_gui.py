#!/usr/bin/env python3
"""Stage-gated GUI for the physical TB3 inspection and unloading workflow.

The GUI deliberately launches the already validated patrol and docking
programs as separate top-level processes.  It never nests docking inside the
patrol process, which avoids the DDS/Nav2 handoff failure seen at VP3.
"""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox, ttk

from ament_index_python.packages import get_package_share_directory


TEXTS = {
    "zh": {
        "title": "RoboInspect TB3 真机工作流",
        "language": "English",
        "header": "真机工作流状态",
        "current_stage": "当前阶段:", "current_process": "当前进程:",
        "latest_report": "最新报告:", "no_process": "无运行进程",
        "no_report": "尚无报告", "operator_gates": "人工安全确认",
        "home_gate": "TB3 已在 Home，巡检路线净空",
        "vp3_gate": "TB3 已在 VP3 扫描起始朝向（用于独立恢复/调试）",
        "load_gate": "方块已装入托盘，人员已离场，VP3→卸货位路线净空",
        "arm_gate": "TB3 已停稳在最终卸货位，SO-ARM 工作范围净空",
        "return_gate": "卸货完成，TB3 仍在最终停车位，返航路线净空",
        "stages": "阶段按钮", "health": "0. 通信与安全检查",
        "patrol": "1. Home → VP1–VP3 巡检",
        "localize": "2A. VP3 定位检查（不运动）",
        "dock": "2B. VP3 → A → Tag → 最终停车",
        "rgb_open": "2C. RGB 装货检查框", "rgb_close": "关闭 RGB 装货检查框",
        "autoclear": "3. Jetson Auto-Clear 抓取",
        "return_home": "4. 最终停车位 → Home（XY + 起始朝向）",
        "full": "全程工作：巡检 → 人工装货 → 停车 → RGB Gate → Auto-Clear",
        "rviz_open": "当前导航阶段打开 RViz", "rviz_close": "关闭 RViz",
        "reports": "打开报告目录", "reset": "重置 GUI 阶段",
        "stop": "紧急停止", "logs": "实时日志",
        "idle": "空闲", "busy_title": "已有任务",
        "busy_body": "请先等待或停止当前阶段。",
        "missing_confirm": "缺少确认", "missing_jetson": "缺少 Jetson 地址",
        "jetson_body": "请关闭 GUI，在终端 source env_fleet.sh 后重新启动。",
        "home_required": "请确认 TB3 已在 Home。",
        "load_required": "请确认装货完成且人员已离场。",
        "arm_required": "请确认 TB3 已停稳且 SO-ARM 工作范围净空。",
        "return_required": "请确认卸货完成、TB3 仍在最终停车位且返航路线净空。",
        "stage_failed": "阶段失败", "rgb_failed": "RGB Gate 启动失败",
        "rviz_unavailable": "RViz 尚无地图", "rviz_failed": "RViz 启动失败",
        "task_running": "任务运行中",
        "running_health": "正在检查通信与安全",
        "running_patrol": "正在执行 VP1–VP3 巡检",
        "running_localize": "正在进行 VP3 无运动定位检查",
        "running_dock": "正在执行 VP3→A→Tag→最终停车",
        "running_autoclear": "Jetson 正在自动抓取直至托盘清空",
        "running_return": "正在从最终卸货停车位返回 Home 并恢复起始朝向",
        "vp3_ready": "VP3 已就绪，等待人工装货",
        "final_stop": "最终卸货停车位已到达",
        "complete": "完整流程完成：巡检、停车及卸货均成功；RGB Gate 保持监控",
        "autoclear_done": "Auto-Clear 抓取完成",
        "home_ready": "已返回 Home：XY 与起始朝向均已恢复",
        "health_ok": "通信与基础安全检查通过", "stage_done": "阶段完成",
        "stopping": "紧急停止中", "load_gate_starting": "正在启动 RGB 装货检查框",
        "error": "阶段失败", "rgb_paused": "RGB Gate 未启动，流程暂停在最终停车位",
        "rgb_timeout": "RGB Gate 就绪超时，流程暂停在最终停车位",
        "vp3_handoff": "VP3 人工交接",
        "vp3_handoff_body": "巡检报告已生成。请将方块装入 TB3 托盘并离开场地；确认 VP3→卸货位路线净空后点击“是”继续完整停车。\n\n点击“否”会安全暂停在 VP3，可稍后使用 2B。",
        "arm_confirm_title": "SO-ARM 卸货确认",
        "arm_confirm_body": "TB3 已到最终停车位，RGB 装货检查框已打开并收到到位信号。请确认普通 RGB 画面正常、红色方块位于检查框内、停车正确且机械臂工作范围净空，然后点击“是”启动 Jetson Auto-Clear。\n\n点击“否”会安全暂停，可稍后使用按钮 3。",
        "rviz_body": "RViz 需要正在运行的 Nav2。按钮 1 和按钮 4 会自动拉起 RViz；后半段开始后可使用此按钮。",
        "close_body": "关闭窗口将紧急停止当前阶段，是否继续？",
    },
    "en": {
        "title": "RoboInspect TB3 Real Workflow",
        "language": "中文",
        "header": "Real Workflow Status",
        "current_stage": "Current stage:", "current_process": "Current process:",
        "latest_report": "Latest report:", "no_process": "No running process",
        "no_report": "No report yet", "operator_gates": "Operator Safety Gates",
        "home_gate": "TB3 is at Home; patrol route is clear",
        "vp3_gate": "TB3 is at the VP3 scan-start heading (recovery/debug only)",
        "load_gate": "Blocks are loaded; personnel clear; VP3-to-dock route is clear",
        "arm_gate": "TB3 is stable at final dock; SO-ARM workspace is clear",
        "return_gate": "Unload complete; TB3 remains at final dock; return route is clear",
        "stages": "Validated Stage Buttons", "health": "0. Communication & Safety Check",
        "patrol": "1. Home → VP1–VP3 Patrol",
        "localize": "2A. VP3 Localization Check (No Motion)",
        "dock": "2B. VP3 → A → Tag → Final Dock",
        "rgb_open": "2C. RGB Load Check Overlay", "rgb_close": "Close RGB Load Overlay",
        "autoclear": "3. Jetson Auto-Clear Pick",
        "return_home": "4. Final Dock → Home (XY + Start Heading)",
        "full": "Full Workflow: Patrol → Manual Load → Dock → RGB Gate → Auto-Clear",
        "rviz_open": "Open RViz for Current Navigation", "rviz_close": "Close RViz",
        "reports": "Open Report Folder", "reset": "Reset GUI Stage",
        "stop": "EMERGENCY STOP", "logs": "Live Process Log",
        "idle": "Idle", "busy_title": "Task Already Running",
        "busy_body": "Wait for or stop the current stage first.",
        "missing_confirm": "Confirmation Required", "missing_jetson": "Jetson Address Missing",
        "jetson_body": "Close the GUI, source env_fleet.sh in a terminal, then restart it.",
        "home_required": "Confirm that TB3 is at Home.",
        "load_required": "Confirm loading is complete and personnel are clear.",
        "arm_required": "Confirm TB3 is stable and the SO-ARM workspace is clear.",
        "return_required": "Confirm unloading is complete, TB3 remains at the final dock, and the return route is clear.",
        "stage_failed": "Stage Failed", "rgb_failed": "RGB Gate Launch Failed",
        "rviz_unavailable": "RViz Map Unavailable", "rviz_failed": "RViz Launch Failed",
        "task_running": "Task Running",
        "running_health": "Checking communication and safety",
        "running_patrol": "Running VP1–VP3 patrol",
        "running_localize": "Checking VP3 localization without motion",
        "running_dock": "Running VP3 → A → Tag → final docking",
        "running_autoclear": "Jetson Auto-Clear is picking until the tray is empty",
        "running_return": "Returning from final dock to Home and restoring start heading",
        "vp3_ready": "VP3 ready; waiting for manual loading",
        "final_stop": "Final unloading dock reached",
        "complete": "Full workflow complete: patrol, docking, and unloading succeeded; RGB Gate remains active",
        "autoclear_done": "Auto-Clear completed",
        "home_ready": "Home reached: XY and start heading restored",
        "health_ok": "Communication and base safety checks passed", "stage_done": "Stage complete",
        "stopping": "Emergency stop in progress", "load_gate_starting": "Starting RGB load overlay",
        "error": "Stage failed", "rgb_paused": "RGB Gate did not start; workflow paused at final dock",
        "rgb_timeout": "RGB Gate readiness timed out; workflow paused at final dock",
        "vp3_handoff": "VP3 Manual Handoff",
        "vp3_handoff_body": "The patrol report is ready. Load the blocks onto the TB3 tray and leave the arena. After confirming the VP3-to-dock route is clear, click Yes to continue docking.\n\nClick No to pause safely at VP3 and use 2B later.",
        "arm_confirm_title": "SO-ARM Unloading Confirmation",
        "arm_confirm_body": "TB3 is at the final dock and the RGB load overlay has received the arrival signal. Confirm the RGB view is normal, red blocks are inside the check area, parking is correct, and the arm workspace is clear; then click Yes to start Jetson Auto-Clear.\n\nClick No to pause safely and use button 3 later.",
        "rviz_body": "RViz requires a running Nav2 stack. Buttons 1 and 4 open RViz automatically; this button is available during the later navigation stages.",
        "close_body": "Closing the window will emergency-stop the current stage. Continue?",
    },
}


def health_command() -> list[str]:
    return [
        "ros2", "run", "real", "tb3_healthcheck.py",
        "--phase", "base", "--exit-zero-on-warn",
    ]


def patrol_command() -> list[str]:
    return [
        "ros2", "run", "real", "run_full_workflow.py",
        "--enable-motion", "--start-at-home", "--inspection-only",
        "--operator-timeout", "0", "--use-rviz",
    ]


def localization_command() -> list[str]:
    return [
        "ros2", "run", "real", "run_hybrid_docking.py",
        "--localize-only", "--start-near-vp3",
        "--nav-startup-timeout", "0", "--max-start-error-m", "0.35",
    ]


def docking_command() -> list[str]:
    return [
        "ros2", "run", "real", "run_hybrid_docking.py",
        "--enable-motion", "--start-near-vp3",
        "--nav-startup-timeout", "0", "--nav-timeout", "0",
        "--servo-timeout", "0",
    ]


def return_home_command() -> list[str]:
    return [
        "ros2", "run", "real", "run_return_home.py",
        "--enable-motion", "--nav-startup-timeout", "0",
        "--nav-timeout", "0", "--use-rviz",
    ]


JETSON_SOARM_DIR = "/home/nvidia/Multi-Robot-Inspection-System/so-arm101"
# Match only the actual Python worker argv.  A loose ``auto_clear.py``
# search also matches the parent ``bash -c`` command because that command
# contains the future exec text, making every launch look like a duplicate.
AUTO_CLEAR_PROCESS_PATTERN = (
    r"^([^ ]*/)?python3?( -u)? auto_clear\.py( .*)?$"
)


def auto_clear_command(jetson_ip: str) -> list[str]:
    """Run the validated arm clear-until-empty loop on Jetson.

    The wrist-camera server is started only when it is not already healthy.
    Keeping the whole operation behind one SSH process lets the GUI stream the
    arm log and prevents a second Auto-Clear instance from being launched.
    """
    remote = " && ".join([
        f"cd {JETSON_SOARM_DIR}",
        "export SOARM_PORT=/dev/ttyACM0",
        f"(! pgrep -f '{AUTO_CLEAR_PROCESS_PATTERN}' >/dev/null || "
        "(echo 'ERROR: Auto-Clear is already running' >&2; exit 2))",
        "(curl -fsS http://127.0.0.1:8765/status >/dev/null || "
        "(nohup .venv/bin/python -u scripts/camera_server.py --index 0 "
        ">/tmp/soarm_camera_server.log 2>&1 </dev/null & "
        "sleep 4 && curl -fsS http://127.0.0.1:8765/status >/dev/null))",
        "exec .venv/bin/python -u auto_clear.py",
    ])
    return [
        "ssh", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=15",
        "-o", "ServerAliveCountMax=4", f"nvidia@{jetson_ip}", remote,
    ]


def auto_clear_stop_command(jetson_ip: str) -> list[str]:
    """Ask Auto-Clear to return Home; force torque-off only on timeout."""
    pattern = AUTO_CLEAR_PROCESS_PATTERN
    remote = (
        f"cd {JETSON_SOARM_DIR} && export SOARM_PORT=/dev/ttyACM0; "
        f"if pgrep -f '{pattern}' >/dev/null; then "
        f"pkill -INT -f '{pattern}'; "
        "for i in 1 2 3 4 5 6 7 8 9 10 11 12; do "
        f"pgrep -f '{pattern}' >/dev/null || exit 0; sleep 1; done; "
        f"pkill -TERM -f '{pattern}' || true; sleep 1; "
        ".venv/bin/python -u scripts/13_recover.py --free; fi"
    )
    return ["ssh", "-o", "BatchMode=yes", f"nvidia@{jetson_ip}", remote]


JETSON_LOAD_GATE_LAUNCHER = "/home/nvidia/run_turtlebot3_load_arm_gate.sh"
LOAD_GATE_PROCESS_PATTERN = (
    r"^python3 /home/nvidia/turtlebot3_load_arm_gate\.py( .*)?$"
)


def load_gate_command(jetson_ip: str) -> list[str]:
    """Open RGB+ROI gate on the laptop via SSH X11, without starting arm."""
    remote = (
        f"ROS_DOMAIN_ID=30 PYTHONUNBUFFERED=1 "
        f"{JETSON_LOAD_GATE_LAUNCHER} "
        "--arm-shell-command '' "
        "--arm-command-topic /gui/load_arm_gate/disabled "
        "--hold-sec 9999"
    )
    return [
        "ssh", "-Y", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=15",
        "-o", "ServerAliveCountMax=4", f"nvidia@{jetson_ip}", remote,
    ]


def load_gate_arrival_command(jetson_ip: str) -> list[str]:
    remote = (
        "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=30 "
        "ros2 topic pub /turtlebot3/load_unload_arrived std_msgs/msg/String "
        "\"{data: 'Ready, waiting for recognition results'}\" "
        "-r 2 --times 6"
    )
    return ["ssh", "-o", "BatchMode=yes", f"nvidia@{jetson_ip}", remote]


def load_gate_stop_command(jetson_ip: str) -> list[str]:
    remote = f"pkill -INT -f '{LOAD_GATE_PROCESS_PATTERN}' || true"
    return ["ssh", "-o", "BatchMode=yes", f"nvidia@{jetson_ip}", remote]


def rviz_command(share: Path) -> list[str]:
    return ["rviz2", "-d", str(share / "rviz/localization_view.rviz")]


ZERO_COMMAND = [
    "ros2", "topic", "pub", "--rate", "20", "--times", "20",
    "/cmd_vel", "geometry_msgs/msg/Twist",
    "{linear: {x: 0.0}, angular: {z: 0.0}}",
]


class RealWorkflowGui:
    def __init__(self) -> None:
        self.share = Path(get_package_share_directory("real"))
        self.workspace = self.share.parents[4]
        self.log_dir = self.workspace / "reports/real_workflow_gui"
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.root = tk.Tk()
        self.language = "zh"
        self.root.title(TEXTS[self.language]["title"])
        self.root.minsize(980, 700)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.state = "idle"
        self.active_process: subprocess.Popen | None = None
        self.active_name = ""
        self.last_process_code: int | None = None
        self.active_log = None
        self.active_log_path: Path | None = None
        self.active_success_state = "idle"
        self.events: queue.Queue = queue.Queue()
        self.rviz_process: subprocess.Popen | None = None
        self.load_gate_process: subprocess.Popen | None = None
        self.load_gate_ready = False
        self.load_gate_arrival_sent = False
        self.load_gate_log = None
        self.load_gate_log_path: Path | None = None
        self.latest_report: Path | None = None
        self.jetson_ip = os.environ.get("JETSON_IP", "").strip()
        self.full_workflow_active = False

        self.state_text_key = "idle"
        self.state_var = tk.StringVar(value=self._t("idle"))
        self.process_var = tk.StringVar(value=self._t("no_process"))
        self.report_var = tk.StringVar(value=self._t("no_report"))
        self.home_confirm = tk.BooleanVar(value=False)
        self.vp3_confirm = tk.BooleanVar(value=False)
        self.load_confirm = tk.BooleanVar(value=False)
        self.arm_confirm = tk.BooleanVar(value=False)
        self.return_confirm = tk.BooleanVar(value=False)

        self._build()
        self._refresh_buttons()
        self.root.after(100, self._poll_events)

    def _t(self, key: str) -> str:
        return TEXTS[self.language][key]

    def _stage_name(self, name: str) -> str:
        names = {
            "zh": {"Health": "通信检查", "Patrol": "巡检",
                   "Localization": "定位检查", "Docking": "入库停车",
                   "AutoClear": "自动抓取", "ReturnHome": "返回 Home"},
            "en": {"Health": "Health Check", "Patrol": "Patrol",
                   "Localization": "Localization", "Docking": "Docking",
                   "AutoClear": "Auto-Clear", "ReturnHome": "Return Home"},
        }
        return names[self.language].get(name, name)

    def toggle_language(self) -> None:
        self.language = "en" if self.language == "zh" else "zh"
        self._apply_language()

    def _apply_language(self) -> None:
        self.root.title(self._t("title"))
        self.header.configure(text=self._t("header"))
        self.stage_label.configure(text=self._t("current_stage"))
        self.process_label.configure(text=self._t("current_process"))
        self.report_label.configure(text=self._t("latest_report"))
        self.checks.configure(text=self._t("operator_gates"))
        self.home_check.configure(text=self._t("home_gate"))
        self.vp3_check.configure(text=self._t("vp3_gate"))
        self.load_check.configure(text=self._t("load_gate"))
        self.arm_check.configure(text=self._t("arm_gate"))
        self.return_check.configure(text=self._t("return_gate"))
        self.workflow.configure(text=self._t("stages"))
        self.health_button.configure(text=self._t("health"))
        self.patrol_button.configure(text=self._t("patrol"))
        self.localize_button.configure(text=self._t("localize"))
        self.dock_button.configure(text=self._t("dock"))
        self.auto_clear_button.configure(text=self._t("autoclear"))
        self.return_home_button.configure(text=self._t("return_home"))
        self.full_button.configure(text=self._t("full"))
        self.language_button.configure(text=self._t("language"))
        rviz_running = (
            self.rviz_process is not None and self.rviz_process.poll() is None)
        self.rviz_button.configure(
            text=self._t("rviz_close" if rviz_running else "rviz_open"))
        self.report_button.configure(text=self._t("reports"))
        self.reset_button.configure(text=self._t("reset"))
        self.stop_button.configure(text=self._t("stop"))
        self.log_frame.configure(text=self._t("logs"))
        self.state_var.set(self._t(self.state_text_key))
        if self.active_process is not None:
            self.process_var.set(
                f"{self._stage_name(self.active_name)} (PID {self.active_process.pid})")
        elif self.last_process_code is not None and self.active_name:
            suffix = "已退出" if self.language == "zh" else "exited"
            self.process_var.set(
                f"{self._stage_name(self.active_name)} {suffix}, "
                f"code={self.last_process_code}")
        else:
            self.process_var.set(self._t("no_process"))
        if self.latest_report is None:
            self.report_var.set(self._t("no_report"))
        self._refresh_buttons()

    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        self.header = ttk.LabelFrame(main)
        self.header.pack(fill="x")
        self.stage_label = ttk.Label(self.header)
        self.stage_label.grid(
            row=0, column=0, sticky="w", padx=8, pady=5)
        ttk.Label(self.header, textvariable=self.state_var).grid(
            row=0, column=1, sticky="w", padx=8, pady=5)
        self.process_label = ttk.Label(self.header)
        self.process_label.grid(
            row=1, column=0, sticky="w", padx=8, pady=5)
        ttk.Label(self.header, textvariable=self.process_var).grid(
            row=1, column=1, sticky="w", padx=8, pady=5)
        self.report_label = ttk.Label(self.header)
        self.report_label.grid(
            row=2, column=0, sticky="nw", padx=8, pady=5)
        ttk.Label(self.header, textvariable=self.report_var, wraplength=780).grid(
            row=2, column=1, sticky="w", padx=8, pady=5)

        self.checks = ttk.LabelFrame(main)
        self.checks.pack(fill="x", pady=(10, 0))
        self.home_check = ttk.Checkbutton(
            self.checks,
            variable=self.home_confirm, command=self._refresh_buttons,
        )
        self.home_check.pack(anchor="w", padx=8, pady=4)
        self.vp3_check = ttk.Checkbutton(
            self.checks,
            variable=self.vp3_confirm, command=self._refresh_buttons,
        )
        self.vp3_check.pack(anchor="w", padx=8, pady=4)
        self.load_check = ttk.Checkbutton(
            self.checks,
            variable=self.load_confirm, command=self._refresh_buttons,
        )
        self.load_check.pack(anchor="w", padx=8, pady=4)
        self.arm_check = ttk.Checkbutton(
            self.checks,
            variable=self.arm_confirm, command=self._refresh_buttons,
        )
        self.arm_check.pack(anchor="w", padx=8, pady=4)
        self.return_check = ttk.Checkbutton(
            self.checks,
            variable=self.return_confirm, command=self._refresh_buttons,
        )
        self.return_check.pack(anchor="w", padx=8, pady=4)

        self.workflow = ttk.LabelFrame(main)
        self.workflow.pack(fill="x", pady=(10, 0))
        workflow = self.workflow
        self.health_button = ttk.Button(
            workflow, text="0. 通信与安全检查", command=self.start_health)
        self.health_button.grid(row=0, column=0, padx=7, pady=8, sticky="ew")
        self.patrol_button = ttk.Button(
            workflow, text="1. Home → VP1–VP3 巡检", command=self.start_patrol)
        self.patrol_button.grid(row=0, column=1, padx=7, pady=8, sticky="ew")
        self.localize_button = ttk.Button(
            workflow, text="2A. VP3 定位检查（不运动）",
            command=self.start_localization)
        self.localize_button.grid(row=0, column=2, padx=7, pady=8, sticky="ew")
        self.dock_button = ttk.Button(
            workflow, text="2B. VP3 → A → Tag → 最终停车",
            command=self.start_docking)
        self.dock_button.grid(row=0, column=3, padx=7, pady=8, sticky="ew")
        self.load_gate_button = ttk.Button(
            workflow, text="2C. RGB 装货检查框",
            command=self.toggle_load_gate)
        self.load_gate_button.grid(
            row=1, column=0, columnspan=2, padx=7, pady=8, sticky="ew")
        self.auto_clear_button = ttk.Button(
            workflow, text="3. Jetson Auto-Clear 抓取",
            command=self.start_auto_clear)
        self.auto_clear_button.grid(
            row=1, column=2, columnspan=2, padx=7, pady=8, sticky="ew")
        self.return_home_button = ttk.Button(
            workflow, text="4. 最终停车位 → Home（XY + 起始朝向）",
            command=self.start_return_home)
        self.return_home_button.grid(
            row=2, column=0, columnspan=4, padx=7, pady=8, sticky="ew")
        self.full_button = ttk.Button(
            workflow,
            text="全程工作：巡检 → 人工装货 → 停车 → RGB Gate → Auto-Clear",
            command=self.start_full_workflow)
        self.full_button.grid(
            row=3, column=0, columnspan=4, padx=7, pady=8, sticky="ew")
        for column in range(4):
            workflow.columnconfigure(column, weight=1)

        utilities = ttk.Frame(main)
        utilities.pack(fill="x", pady=(10, 0))
        self.language_button = ttk.Button(
            utilities, command=self.toggle_language)
        self.language_button.pack(side="left")
        self.rviz_button = ttk.Button(
            utilities, command=self.toggle_rviz)
        self.rviz_button.pack(side="left", padx=(8, 0))
        self.report_button = ttk.Button(utilities, command=self.open_report_dir)
        self.report_button.pack(side="left", padx=(8, 0))
        self.reset_button = ttk.Button(utilities, command=self.reset_state)
        self.reset_button.pack(side="left", padx=(8, 0))
        self.stop_button = tk.Button(
            utilities, text="紧急停止 / EMERGENCY STOP",
            command=self.emergency_stop, bg="#b00020", fg="white",
            activebackground="#d32f2f", activeforeground="white",
            font=("TkDefaultFont", 10, "bold"), padx=12, pady=5,
        )
        self.stop_button.pack(side="right")

        self.log_frame = ttk.LabelFrame(main)
        self.log_frame.pack(fill="both", expand=True, pady=(10, 0))
        log_frame = self.log_frame
        self.log_text = tk.Text(
            log_frame, height=22, wrap="none", state="disabled",
            background="#111820", foreground="#d6e2ee",
            insertbackground="white")
        yscroll = ttk.Scrollbar(
            log_frame, orient="vertical", command=self.log_text.yview)
        xscroll = ttk.Scrollbar(
            log_frame, orient="horizontal", command=self.log_text.xview)
        self.log_text.configure(
            yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self._apply_language()

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _set_state(self, state: str, text_key: str) -> None:
        self.state = state
        self.state_text_key = text_key
        self.state_var.set(self._t(text_key))
        self._refresh_buttons()

    def _refresh_buttons(self) -> None:
        busy = self.active_process is not None
        normal = "normal"
        disabled = "disabled"
        self.health_button.configure(state=disabled if busy else normal)
        self.patrol_button.configure(
            state=(normal if not busy and self.home_confirm.get()
                   else disabled))
        vp3_ready = self.state == "vp3_ready" or self.vp3_confirm.get()
        self.localize_button.configure(
            state=normal if not busy and vp3_ready else disabled)
        self.dock_button.configure(
            state=(normal if not busy and vp3_ready and self.load_confirm.get()
                   else disabled))
        load_gate_running = (
            self.load_gate_process is not None
            and self.load_gate_process.poll() is None)
        self.load_gate_button.configure(
            text=(self._t("rgb_close") if load_gate_running
                  else self._t("rgb_open")),
            state=(normal if load_gate_running
                   or (not busy and bool(self.jetson_ip))
                   else disabled))
        self.auto_clear_button.configure(
            state=(normal if not busy and bool(self.jetson_ip)
                   and self.arm_confirm.get() else disabled))
        self.return_home_button.configure(
            state=(normal if not busy and self.return_confirm.get()
                   else disabled))
        self.full_button.configure(
            state=(normal if not busy and self.home_confirm.get()
                   and bool(self.jetson_ip) else disabled))
        self.stop_button.configure(state=normal if busy else disabled)
        rviz_running = (
            self.rviz_process is not None and self.rviz_process.poll() is None)
        rviz_available = (
            busy and self.active_name in {"Localization", "Docking"})
        self.rviz_button.configure(
            state=normal if rviz_running or rviz_available else disabled)

    def _start_process(
            self, name: str, command: list[str], success_state: str,
            running_text_key: str) -> None:
        if self.active_process is not None:
            messagebox.showwarning(self._t("busy_title"), self._t("busy_body"))
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = name.lower().replace(" ", "_")
        self.active_log_path = self.log_dir / f"{stamp}_{safe_name}.log"
        self.active_log = self.active_log_path.open("w", encoding="utf-8")
        self._append_log("\n$ " + " ".join(command) + "\n")
        try:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except Exception:
            self.active_log.close()
            self.active_log = None
            raise
        self.active_process = process
        self.active_name = name
        self.last_process_code = None
        self.active_success_state = success_state
        self.process_var.set(f"{self._stage_name(name)} (PID {process.pid})")
        self._set_state("running", running_text_key)
        threading.Thread(
            target=self._read_process, args=(process, name), daemon=True,
        ).start()

    def _read_process(self, process: subprocess.Popen, name: str) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            self.events.put(("line", line))
        code = process.wait()
        self.events.put(("done", process, name, code))

    def _poll_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "line":
                    line = event[1]
                    if self.active_log is not None:
                        self.active_log.write(line)
                        self.active_log.flush()
                    self._append_log(line)
                elif event[0] == "done":
                    self._finish_process(*event[1:])
                elif event[0] == "load_gate_line":
                    self._append_log("[RGB Gate] " + event[1])
                elif event[0] == "load_gate_ready":
                    if event[1] is self.load_gate_process:
                        self.load_gate_ready = True
                        self._append_log(
                            "[GUI] RGB load/arm gate is ready; "
                            "publishing arrival on Domain 30\n")
                        self._publish_load_gate_arrival()
                        self._refresh_buttons()
                elif event[0] == "load_gate_arrival_done":
                    if event[1] is self.load_gate_process:
                        self.load_gate_arrival_sent = event[2] == 0
                        self._append_log(
                            "[GUI] RGB gate arrival signal "
                            f"{'sent' if self.load_gate_arrival_sent else 'failed'}\n")
                elif event[0] == "load_gate_done":
                    self._finish_load_gate_process(*event[1:])
        except queue.Empty:
            pass
        self._poll_rviz()
        self.root.after(100, self._poll_events)

    def _finish_process(
            self, process: subprocess.Popen, name: str, code: int) -> None:
        if process is not self.active_process:
            return
        if self.active_log is not None:
            self.active_log.close()
        self.active_log = None
        self.active_process = None
        self.last_process_code = code
        suffix = "已退出" if self.language == "zh" else "exited"
        self.process_var.set(
            f"{self._stage_name(name)} {suffix}, code={code}")
        report = self._newest_report(name)
        if report is not None:
            self.latest_report = report
            self.report_var.set(str(report))
        if code == 0:
            if self.active_success_state == "vp3_ready":
                self.vp3_confirm.set(True)
                self.home_confirm.set(False)
                self._set_state("vp3_ready", "vp3_ready")
                if self.full_workflow_active:
                    self.root.after(100, self._continue_full_after_patrol)
            elif self.active_success_state == "final_stop":
                self.load_confirm.set(False)
                self._set_state("final_stop", "final_stop")
                if self.full_workflow_active:
                    self.root.after(100, self._continue_full_after_docking)
            elif self.active_success_state == "auto_clear_done":
                self.arm_confirm.set(False)
                if self.full_workflow_active:
                    self.full_workflow_active = False
                    self._set_state(
                        "complete", "complete")
                else:
                    self._set_state("auto_clear_done", "autoclear_done")
            elif self.active_success_state == "home_ready":
                self.home_confirm.set(True)
                self.vp3_confirm.set(False)
                self.load_confirm.set(False)
                self.arm_confirm.set(False)
                self.return_confirm.set(False)
                self._set_state(
                    "home_ready", "home_ready")
            elif self.active_success_state == "health_ok":
                self._set_state("idle", "health_ok")
            else:
                self._set_state(self.active_success_state, "stage_done")
        else:
            self.full_workflow_active = False
            self._set_state("error", "error")
            messagebox.showerror(
                self._t("stage_failed"),
                ((f"{self._stage_name(name)} 退出码 {code}\n日志："
                  f"{self.active_log_path}\n机器人不会自动进入下一阶段。")
                 if self.language == "zh" else
                 (f"{self._stage_name(name)} exited with code {code}\nLog: "
                  f"{self.active_log_path}\nThe robot will not advance automatically.")))
        self._refresh_buttons()

    def _newest_report(self, name: str) -> Path | None:
        patterns: list[str] = []
        if name == "Patrol":
            patterns.append(
                "reports/full_workflow/workflow_*/workflow_report.json")
        elif name in {"Localization", "Docking"}:
            patterns.append(
                "ros_ws/doc/*/apriltag/hybrid_*_report.json")
        elif name == "ReturnHome":
            patterns.append(
                "reports/return_home/return_home_*_report.json")
        candidates = []
        for pattern in patterns:
            candidates.extend(self.workspace.glob(pattern))
        return max(candidates, key=lambda path: path.stat().st_mtime) \
            if candidates else None

    def start_health(self) -> None:
        self._start_process(
            "Health", health_command(), "health_ok", "running_health")

    def start_patrol(self) -> None:
        if not self.home_confirm.get():
            messagebox.showwarning(
                self._t("missing_confirm"), self._t("home_required"))
            return
        self._start_process(
            "Patrol", patrol_command(), "vp3_ready",
            "running_patrol")

    def start_localization(self) -> None:
        self._start_process(
            "Localization", localization_command(), "vp3_ready",
            "running_localize")

    def start_docking(self) -> None:
        if not self.load_confirm.get():
            messagebox.showwarning(
                self._t("missing_confirm"), self._t("load_required"))
            return
        self._start_process(
            "Docking", docking_command(), "final_stop",
            "running_dock")

    def start_auto_clear(self) -> None:
        if not self.jetson_ip:
            messagebox.showerror(
                self._t("missing_jetson"), self._t("jetson_body"))
            return
        if not self.arm_confirm.get():
            messagebox.showwarning(
                self._t("missing_confirm"), self._t("arm_required"))
            return
        self._start_process(
            "AutoClear", auto_clear_command(self.jetson_ip),
            "auto_clear_done", "running_autoclear")

    def start_return_home(self) -> None:
        if not self.return_confirm.get():
            messagebox.showwarning(
                self._t("missing_confirm"), self._t("return_required"))
            return
        self._start_process(
            "ReturnHome", return_home_command(), "home_ready",
            "running_return")

    def toggle_load_gate(self) -> None:
        if (self.load_gate_process is not None
                and self.load_gate_process.poll() is None):
            self.stop_load_gate()
        else:
            self.start_load_gate()

    def start_load_gate(self) -> bool:
        if not self.jetson_ip:
            messagebox.showerror(
                self._t("missing_jetson"), self._t("jetson_body"))
            return False
        if (self.load_gate_process is not None
                and self.load_gate_process.poll() is None):
            return True
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.load_gate_log_path = self.log_dir / f"{stamp}_rgb_load_gate.log"
        self.load_gate_log = self.load_gate_log_path.open(
            "w", encoding="utf-8")
        command = load_gate_command(self.jetson_ip)
        self._append_log("\n$ " + " ".join(command) + "\n")
        try:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except Exception as error:
            self.load_gate_log.close()
            self.load_gate_log = None
            messagebox.showerror(self._t("rgb_failed"), str(error))
            return False
        self.load_gate_process = process
        self.load_gate_ready = False
        self.load_gate_arrival_sent = False
        threading.Thread(
            target=self._read_load_gate_process, args=(process,), daemon=True,
        ).start()
        self._refresh_buttons()
        return True

    def _read_load_gate_process(self, process: subprocess.Popen) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            if self.load_gate_log is not None:
                self.load_gate_log.write(line)
                self.load_gate_log.flush()
            self.events.put(("load_gate_line", line))
            if "Waiting for TurtleBot3 arrival" in line:
                self.events.put(("load_gate_ready", process))
        self.events.put(("load_gate_done", process, process.wait()))

    def _finish_load_gate_process(
            self, process: subprocess.Popen, code: int) -> None:
        if process is not self.load_gate_process:
            return
        if self.load_gate_log is not None:
            self.load_gate_log.close()
        self.load_gate_log = None
        self.load_gate_process = None
        self.load_gate_ready = False
        self.load_gate_arrival_sent = False
        self._append_log(f"[GUI] RGB load/arm gate exited, code={code}\n")
        if self.full_workflow_active and self.state == "load_gate_starting":
            self.full_workflow_active = False
            self._set_state("final_stop", "rgb_paused")
            messagebox.showerror(
                self._t("rgb_failed"),
                ((f"检查窗口退出码 {code}\n日志：{self.load_gate_log_path}")
                 if self.language == "zh" else
                 (f"Overlay exited with code {code}\nLog: {self.load_gate_log_path}")))
        self._refresh_buttons()

    def _publish_load_gate_arrival(self) -> None:
        process = self.load_gate_process
        if process is None:
            return

        def worker() -> None:
            try:
                result = subprocess.run(
                    load_gate_arrival_command(self.jetson_ip),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=10.0, check=False)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 124
            self.events.put(("load_gate_arrival_done", process, code))

        threading.Thread(target=worker, daemon=True).start()

    def stop_load_gate(self) -> None:
        process = self.load_gate_process
        if process is None:
            return
        self._append_log("[GUI] stopping RGB load/arm gate\n")

        def worker() -> None:
            try:
                subprocess.run(
                    load_gate_stop_command(self.jetson_ip),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=6.0, check=False)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

        threading.Thread(target=worker, daemon=True).start()

    def start_full_workflow(self) -> None:
        if not self.home_confirm.get():
            messagebox.showwarning(
                self._t("missing_confirm"), self._t("home_required"))
            return
        if not self.jetson_ip:
            messagebox.showerror(
                self._t("missing_jetson"), self._t("jetson_body"))
            return
        self.full_workflow_active = True
        self._append_log(
            "\n[GUI] FULL WORKFLOW started; operator gates remain active.\n")
        self.start_patrol()

    def _continue_full_after_patrol(self) -> None:
        if not self.full_workflow_active or self.active_process is not None:
            return
        ready = messagebox.askyesno(
            self._t("vp3_handoff"), self._t("vp3_handoff_body"))
        if not ready:
            self.full_workflow_active = False
            self._append_log("[GUI] Full workflow paused at VP3 by operator.\n")
            return
        self.load_confirm.set(True)
        self.start_docking()

    def _continue_full_after_docking(self) -> None:
        if not self.full_workflow_active or self.active_process is not None:
            return
        self._set_state("load_gate_starting", "load_gate_starting")
        if not self.start_load_gate():
            self.full_workflow_active = False
            self._set_state("final_stop", "rgb_paused")
            return
        self._wait_load_gate_for_full(0)

    def _wait_load_gate_for_full(self, attempt: int) -> None:
        if not self.full_workflow_active:
            return
        if self.load_gate_ready and self.load_gate_arrival_sent:
            self._prompt_full_auto_clear()
            return
        if (self.load_gate_process is None
                or self.load_gate_process.poll() is not None):
            return
        if attempt >= 150:
            self.full_workflow_active = False
            self._set_state("final_stop", "rgb_timeout")
            messagebox.showerror(
                self._t("rgb_failed"),
                ((f"日志：{self.load_gate_log_path}") if self.language == "zh"
                 else f"Log: {self.load_gate_log_path}"))
            return
        self.root.after(100, self._wait_load_gate_for_full, attempt + 1)

    def _prompt_full_auto_clear(self) -> None:
        ready = messagebox.askyesno(
            self._t("arm_confirm_title"), self._t("arm_confirm_body"))
        if not ready:
            self.full_workflow_active = False
            self._append_log(
                "[GUI] Full workflow paused before Auto-Clear by operator.\n")
            return
        self.arm_confirm.set(True)
        self.start_auto_clear()

    def emergency_stop(self) -> None:
        process = self.active_process
        if process is None:
            return
        self._append_log("\n[GUI] EMERGENCY STOP requested\n")
        self._set_state("stopping", "stopping")

        active_name = self.active_name
        jetson_ip = self.jetson_ip

        def stop_worker() -> None:
            if active_name == "AutoClear" and jetson_ip:
                self.events.put((
                    "line",
                    "[GUI] stopping remote Auto-Clear and releasing arm torque\n"))
                try:
                    subprocess.run(
                        auto_clear_stop_command(jetson_ip),
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        timeout=18.0, check=False)
                except subprocess.TimeoutExpired:
                    self.events.put((
                        "line", "[GUI] WARNING: remote arm stop timed out\n"))
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            subprocess.run(
                ZERO_COMMAND, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, timeout=8.0, check=False)
            self.events.put(("line", "[GUI] independent zero velocity sent\n"))

        self.full_workflow_active = False

        threading.Thread(target=stop_worker, daemon=True).start()

    def toggle_rviz(self) -> None:
        if self.rviz_process is not None and self.rviz_process.poll() is None:
            try:
                os.killpg(self.rviz_process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            self.rviz_process = None
            self.rviz_button.configure(text=self._t("rviz_open"))
            return
        if (self.active_process is None or
                self.active_name not in {"Localization", "Docking"}):
            messagebox.showinfo(
                self._t("rviz_unavailable"), self._t("rviz_body"))
            return
        try:
            self.rviz_process = subprocess.Popen(
                rviz_command(self.share), start_new_session=True)
            self.rviz_button.configure(text=self._t("rviz_close"))
        except Exception as error:
            messagebox.showerror(self._t("rviz_failed"), str(error))

    def _poll_rviz(self) -> None:
        if self.rviz_process is not None and self.rviz_process.poll() is not None:
            self.rviz_process = None
            self.rviz_button.configure(text=self._t("rviz_open"))

    def open_report_dir(self) -> None:
        target = self.latest_report.parent if self.latest_report else self.log_dir
        subprocess.Popen(["xdg-open", str(target)], start_new_session=True)

    def reset_state(self) -> None:
        if self.active_process is not None:
            messagebox.showwarning(
                self._t("task_running"), self._t("busy_body"))
            return
        self.home_confirm.set(False)
        self.vp3_confirm.set(False)
        self.load_confirm.set(False)
        self.arm_confirm.set(False)
        self.return_confirm.set(False)
        self.full_workflow_active = False
        self._set_state("idle", "idle")

    def close(self) -> None:
        if self.active_process is not None:
            if not messagebox.askyesno(
                    self._t("task_running"), self._t("close_body")):
                return
            self.emergency_stop()
        if self.rviz_process is not None and self.rviz_process.poll() is None:
            try:
                os.killpg(self.rviz_process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        if (self.load_gate_process is not None
                and self.load_gate_process.poll() is None):
            self.stop_load_gate()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    gui = RealWorkflowGui()
    gui.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
