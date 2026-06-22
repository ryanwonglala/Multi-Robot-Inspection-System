#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
import yaml

from task_layer.model_spawner import (
    area_center,
    area_random,
    list_builtin_models,
    make_spawn_command,
    resolve_model_file,
    run_spawn,
    unique_entity_name,
)


STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}

# Task 2.1: Areas that are structurally available in the map but must NOT be
# added to an inspection route (GUI-only restriction; world_model.yaml unchanged).
INSPECT_DISABLED_AREAS = {'mother_base', 'charging_station', 'server_door'}

# Task 1.2: The only models that can be selected in the Scene tab.
# Spawnable shapes for robustness validation. Boxes keep all three sizes
# (0.25 / 0.45 / 0.70 m). Cylinder/cone/sphere are large-only (0.20 m) — the
# 2.5 / 8 cm medium/small variants were too small to be a useful visual
# reference and were removed. Cone is an STL mesh under models/primitives/.
ALLOWED_SCENE_MODELS = {
    'small_box', 'medium_box', 'large_box',
    'large_cylinder', 'large_cone', 'large_sphere',
}


def default_world_model_path() -> str:
    share_dir = get_package_share_directory('task_layer')
    return str(Path(share_dir) / 'config' / 'world_model.yaml')


def default_report_dir() -> str:
    return str(Path.home() / 'roboinspec_ws' / 'reports')


def load_robot_registry() -> dict:
    """robots.yaml is optional for the GUI: used to send each robot back to
    its own dock; missing file/entries just fall back to runner defaults."""
    try:
        share_dir = get_package_share_directory('task_layer')
        with (Path(share_dir) / 'config' / 'robots.yaml').open(encoding='utf-8') as f:
            return (yaml.safe_load(f) or {}).get('robots', {})
    except Exception:  # noqa: BLE001
        return {}


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def normalize_text(value: str) -> str:
    return value.strip().lower().replace(' ', '_').replace('-', '_')


class TaskGuiNode(Node):
    def __init__(self):
        super().__init__('task_gui')
        self.declare_parameter('world_model_path', default_world_model_path())
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('yaw', 0.0)
        # Robot namespaces this GUI can command. [''] = legacy single robot
        # in the root namespace.
        self.declare_parameter('robots', ['tb3', 'arm'])
        self.declare_parameter('server_timeout_sec', 2.0)
        self.declare_parameter('world', 'map')
        self.declare_parameter('report_dir', default_report_dir())
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass

        self.world_model = self.load_world_model()
        self.areas = self.world_model.get('areas', {})
        # Accessible areas first; walled-off ones (accessible: false) sink to
        # the end of every list and are not selectable in the GUI.
        self.area_items = sorted(
            self.areas.items(),
            key=lambda item: not item[1].get('accessible', True))
        self.models = list_builtin_models()
        robots = [str(ns).strip().strip('/') for ns in
                  (self.get_parameter('robots').value or [])]
        if not robots:
            robots = ['']
        self.robot_namespaces = robots
        self.nav_clients = {}
        for ns in robots:
            action_name = f'/{ns}/navigate_to_pose' if ns else 'navigate_to_pose'
            self.nav_clients[ns] = ActionClient(self, NavigateToPose, action_name)
        self.active_robot = robots[0]
        self.robot_registry = load_robot_registry()
        self.goal_handle = None
        # Fleet-wide anomaly bus: inspection_runner publishes every photo-diff
        # detection as a latched JSON event on /anomaly_events (transient_local
        # so events fired before the GUI opened are replayed on subscribe). We
        # only collect them here; the red Tk label is refreshed from the main
        # thread in poll_inspection.
        self.anomaly_events: list[dict] = []
        anomaly_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/anomaly_events', self._on_anomaly_event, anomaly_qos)

    def _on_anomaly_event(self, msg: String):
        try:
            self.anomaly_events.append(json.loads(msg.data))
        except ValueError:
            self.get_logger().warn(
                'Unparseable anomaly event: %s' % msg.data[:120])

    @property
    def client(self):
        # All existing call sites keep working: always the active robot's client.
        return self.nav_clients[self.active_robot]

    def load_world_model(self) -> dict:
        path = Path(self.get_parameter('world_model_path').value).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'world_model_path does not exist: {path}')
        with path.open('r', encoding='utf-8') as file:
            return yaml.safe_load(file) or {}

    def resolve_target(self, text: str) -> tuple[str, dict]:
        query = text.strip()
        if not query:
            raise ValueError('Enter a number or area name')

        if query.isdigit():
            index = int(query)
            if index < 1 or index > len(self.area_items):
                raise ValueError(f'Area number must be 1..{len(self.area_items)}')
            key, area = self.area_items[index - 1]
            if not area.get('accessible', True):
                raise ValueError(f'Area {key} is walled off in the current map')
            return key, area

        normalized = normalize_text(query)
        for key, area in self.area_items:
            names = {
                normalize_text(key),
                normalize_text(area.get('display_name', key)),
                normalize_text(area.get('marker_model', '')),
            }
            if normalized in names:
                if not area.get('accessible', True):
                    raise ValueError(
                        f'Area {key} is walled off in the current map')
                return key, area

        raise ValueError(f'Unknown area: {query}')

    def build_goal(self, area: dict) -> NavigateToPose.Goal:
        center = area.get('center')
        if not center or len(center) < 2:
            raise ValueError('Selected area is missing center: [x, y]')

        yaw = float(self.get_parameter('yaw').value)
        qz, qw = yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('goal_frame').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(center[0])
        pose.pose.position.y = float(center[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal


class TaskGui:
    def __init__(self, node: TaskGuiNode):
        self.node = node
        self.pending_goal_key = None
        self.result_future = None
        self.send_future = None
        self.inspect_processes = {}   # ns -> Popen
        self.inspect_logs = {}        # ns -> (file handle, Path)
        self.spawn_count = 0
        # Task 4: track whether we have already shown the allocation block in
        # the Inspection Status text so we only inject it once per run.
        self._allocation_shown = False

        self.root = tk.Tk()
        self.root.title('RoboInspect Task GUI')
        self.root.minsize(860, 620)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.status_var = tk.StringVar(value='Ready')
        self.detail_var = tk.StringVar(value='')
        self.target_var = tk.StringVar()
        # Task 2.3: the editable Target Rooms tk.Text (_route_text) is the canonical
        # route store; get_route()/set_route() are the only accessors.
        self.inspect_mode_var = tk.StringVar(value='auto')
        self.max_attempts_var = tk.StringVar(value='2')
        self.spread_ratio_var = tk.StringVar(value='0.35')
        self.return_home_var = tk.BooleanVar(value=True)
        self.inspect_status_var = tk.StringVar(value='Ready')
        self.latest_report_var = tk.StringVar(value='')
        # Red anomaly banner (count + last few detections); driven by the
        # node's /anomaly_events subscription, refreshed in poll_inspection.
        self.anomaly_var = tk.StringVar(value='')
        self._anomaly_seen = 0
        self.model_var = tk.StringVar(value=self.node.models[0]['key'] if self.node.models else '')
        self.name_var = tk.StringVar(value='')
        self.area_var = tk.StringVar()
        self.placement_var = tk.StringVar(value='center')
        self.x_var = tk.StringVar(value='0.000')
        self.y_var = tk.StringVar(value='0.000')
        self.z_var = tk.StringVar(value='0.000')  # Task 1.4: default to floor level
        self.yaw_var = tk.StringVar(value='0.000')
        self.margin_var = tk.StringVar(value='0.200')
        self.allow_renaming_var = tk.BooleanVar(value=False)
        self.scene_status_var = tk.StringVar(value='Ready')

        self._build()
        self.root.after(50, self.spin_ros)
        self.root.after(500, self.poll_inspection)

    def _build(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        robot_row = ttk.Frame(main)
        robot_row.pack(fill='x', pady=(0, 6))
        ttk.Label(robot_row, text='Robot').pack(side='left')
        self.robot_var = tk.StringVar(value=self.node.active_robot or '(root)')
        robot_box = ttk.Combobox(
            robot_row, textvariable=self.robot_var, state='readonly', width=12,
            values=[ns or '(root)' for ns in self.node.robot_namespaces])
        robot_box.pack(side='left', padx=8)
        robot_box.bind('<<ComboboxSelected>>', self.on_robot_select)

        notebook = ttk.Notebook(main)
        notebook.pack(fill='both', expand=True)

        self.nav_tab = ttk.Frame(notebook, padding=8)
        self.inspect_tab = ttk.Frame(notebook, padding=8)
        self.scene_tab = ttk.Frame(notebook, padding=8)
        notebook.add(self.nav_tab, text='Navigate')
        notebook.add(self.inspect_tab, text='Inspect')
        notebook.add(self.scene_tab, text='Scene')

        self._build_nav_tab()
        self._build_inspect_tab()
        self._build_scene_tab()

        status = ttk.LabelFrame(main, text='Status')
        status.pack(fill='x', pady=(8, 0))
        ttk.Label(status, textvariable=self.status_var).pack(anchor='w')
        ttk.Label(status, textvariable=self.detail_var).pack(anchor='w')

    def _area_available(self, key, area, force_available_keys, disabled_keys):
        """Per-tab availability: force_available wins; otherwise accessible and
        not in the tab's disabled set."""
        if key in force_available_keys:
            return True
        return area.get('accessible', True) and key not in disabled_keys

    def _build_area_list(self, parent, select_callback=None, double_callback=None,
                         force_available_keys=None, disabled_keys=None,
                         selectmode='browse'):
        """Build an area Listbox.

        force_available_keys: area keys rendered as available (not grey) and
            selectable even when accessible==False (Scene tab: restricted_zone).
        disabled_keys: accessible area keys rendered unavailable/grey for this
            tab's action (Inspect tab: INSPECT_DISABLED_AREAS).
        selectmode: 'browse' (single) or 'extended' (multi, Inspect tab).

        Ordering rule: all available rows first, then all unavailable rows —
        never interleaved (order within each group preserves node.area_items).
        A per-list row -> area_items index map is stored on the widget as
        area_list._row_to_gidx so callbacks resolve selections after reordering.
        """
        if force_available_keys is None:
            force_available_keys = set()
        if disabled_keys is None:
            disabled_keys = set()
        frame = ttk.LabelFrame(parent, text='Areas')
        area_list = tk.Listbox(frame, activestyle='dotbox', exportselection=False,
                               selectmode=selectmode)
        area_list.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(frame, orient='vertical', command=area_list.yview)
        scrollbar.pack(side='right', fill='y')
        area_list.configure(yscrollcommand=scrollbar.set)
        if select_callback:
            area_list.bind('<<ListboxSelect>>', select_callback)
        if double_callback:
            area_list.bind('<Double-Button-1>', double_callback)

        # Available first, unavailable last (stable within each group).
        indexed = list(enumerate(self.node.area_items))
        avail = [t for t in indexed
                 if self._area_available(t[1][0], t[1][1], force_available_keys, disabled_keys)]
        unavail = [t for t in indexed
                   if not self._area_available(t[1][0], t[1][1], force_available_keys, disabled_keys)]
        area_list._row_to_gidx = []
        for row, (gidx, (key, area)) in enumerate(avail + unavail, start=1):
            name = area.get('display_name', key)
            # Task 3: name only (no coordinate tail).
            area_list.insert('end', f'{row:02d}. {key} | {name}')
            if not self._area_available(key, area, force_available_keys, disabled_keys):
                area_list.itemconfig('end', foreground='gray60')
            area_list._row_to_gidx.append(gidx)
        return frame, area_list

    def _build_nav_tab(self):
        input_row = ttk.Frame(self.nav_tab)
        input_row.pack(fill='x', pady=(0, 8))
        ttk.Label(input_row, text='Target').pack(side='left')
        entry = ttk.Entry(input_row, textvariable=self.target_var)
        entry.pack(side='left', fill='x', expand=True, padx=8)
        entry.bind('<Return>', lambda _event: self.send_goal())
        ttk.Button(input_row, text='Go', command=self.send_goal).pack(side='left')
        ttk.Button(input_row, text='Cancel', command=self.cancel_goal).pack(side='left', padx=(6, 0))

        body = ttk.Frame(self.nav_tab)
        body.pack(fill='both', expand=True)
        list_frame, self.nav_area_list = self._build_area_list(
            body,
            select_callback=self.on_nav_area_select,
            double_callback=lambda _event: self.send_goal(),
        )
        list_frame.pack(side='left', fill='both', expand=True)
        info = ttk.LabelFrame(body, text='Selected')
        info.pack(side='left', fill='both', expand=True, padx=(10, 0))
        self.nav_info_text = tk.Text(info, height=12, wrap='word')
        self.nav_info_text.pack(fill='both', expand=True)
        self.nav_info_text.configure(state='disabled')

    def _build_inspect_tab(self):
        body = ttk.Frame(self.inspect_tab)
        body.pack(fill='both', expand=True)
        # Task 2.4 multi-select; Task 2.1 disabled areas (greyed + sunk to the end
        # by _build_area_list's ordering rule, so no available/disabled interleave).
        list_frame, self.inspect_area_list = self._build_area_list(
            body,
            select_callback=None,
            double_callback=lambda _event: self.add_selected_area_to_route(),
            disabled_keys=INSPECT_DISABLED_AREAS,
            selectmode='extended',
        )
        list_frame.pack(side='left', fill='both', expand=True)

        # Task 2.4: "Select All" button beneath the area list.
        select_all_btn = ttk.Button(list_frame, text='Select All',
                                    command=self._inspect_select_all)
        select_all_btn.pack(fill='x', padx=4, pady=(2, 4))

        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True, padx=(10, 0))

        # Task 2.3: replace single-line Entry with multi-line Text + Scrollbar.
        route_frame = ttk.LabelFrame(right, text='Target Rooms')
        route_frame.pack(fill='x')
        route_text_frame = ttk.Frame(route_frame)
        route_text_frame.pack(fill='x', padx=6, pady=6)
        self._route_text = tk.Text(route_text_frame, height=3, wrap='word', width=30)
        self._route_text.pack(side='left', fill='x', expand=True)
        route_text_scroll = ttk.Scrollbar(route_text_frame, orient='vertical',
                                           command=self._route_text.yview)
        route_text_scroll.pack(side='right', fill='y')
        self._route_text.configure(yscrollcommand=route_text_scroll.set)
        # Task 2.3: the Text stays editable (hand-typed routes are honoured via
        # get_route()); Add Selected / Clear route through set_route().
        route_buttons = ttk.Frame(route_frame)
        route_buttons.pack(fill='x', padx=6, pady=(0, 6))
        ttk.Button(route_buttons, text='Add Selected',
                   command=self.add_selected_area_to_route).pack(side='left')
        ttk.Button(route_buttons, text='Clear',
                   command=lambda: self.set_route('')).pack(
                       side='left', padx=(6, 0))

        mode_frame = ttk.LabelFrame(right, text='Dispatch Mode')
        mode_frame.pack(fill='x', pady=(10, 0))
        ttk.Radiobutton(
            mode_frame, text='Auto allocate (split route across all robots)',
            variable=self.inspect_mode_var, value='auto').pack(anchor='w', padx=6)
        ttk.Radiobutton(
            mode_frame, text='Manual (active robot runs the whole route)',
            variable=self.inspect_mode_var, value='manual').pack(anchor='w', padx=6)

        params = ttk.LabelFrame(right, text='Parameters')
        params.pack(fill='x', pady=(10, 0))
        # Task 2.2: bilingual description labels next to each parameter entry.
        small_font = ('TkDefaultFont', 8)
        grey = '#555555'
        self._entry_row(params, 0, 'Max Attempts', self.max_attempts_var)
        ttk.Label(params, text='每个区域最多尝试的候选观测点数 / max candidate viewpoints tried per area',
                  font=small_font, foreground=grey).grid(
                      row=0, column=2, sticky='w', padx=(0, 4), pady=3)
        self._entry_row(params, 1, 'Spread Ratio', self.spread_ratio_var)
        ttk.Label(params, text='候选点偏离区域中心的扩散比例 / how far candidates spread from area center',
                  font=small_font, foreground=grey).grid(
                      row=1, column=2, sticky='w', padx=(0, 4), pady=3)
        return_home_row = ttk.Frame(params)
        return_home_row.grid(row=2, column=0, columnspan=3, sticky='w', pady=4)
        ttk.Checkbutton(return_home_row, text='Return Home',
                        variable=self.return_home_var).pack(side='left')
        ttk.Label(return_home_row,
                  text='  巡检后机器人是否自动返回各自充电桩 / whether robots auto-return to docks after inspection',
                  font=small_font, foreground=grey).pack(side='left')

        buttons = ttk.Frame(right)
        buttons.pack(fill='x', pady=(10, 0))
        ttk.Button(buttons, text='Start Inspection',
                   command=self.start_inspection).pack(side='left')
        ttk.Button(buttons, text='Abort & Reset to Dock',
                   command=self.abort_and_reset_to_dock).pack(side='left', padx=(8, 0))
        ttk.Button(buttons, text='Open Report Dir',
                   command=self.set_report_dir_status).pack(side='left', padx=(8, 0))

        # Task 2.3: replace stacked Labels with one scrollable Text widget.
        report = ttk.LabelFrame(right, text='Inspection Status')
        report.pack(fill='both', expand=True, pady=(10, 0))
        status_text_frame = ttk.Frame(report)
        status_text_frame.pack(fill='both', expand=True, padx=6, pady=6)
        self._status_text = tk.Text(status_text_frame, height=7, wrap='word',
                                    state='disabled')
        self._status_text.pack(side='left', fill='both', expand=True)
        status_scroll = ttk.Scrollbar(status_text_frame, orient='vertical',
                                      command=self._status_text.yview)
        status_scroll.pack(side='right', fill='y')
        self._status_text.configure(yscrollcommand=status_scroll.set)
        # Red tag for anomaly lines.
        self._status_text.tag_configure('anomaly', foreground='#cc0000')
        # Task 2.3/4: single source of truth for the status box; every refresh
        # re-renders from this snapshot so there is exactly one render path.
        self._status_state = {
            'status': 'Ready', 'report': '', 'allocation_lines': None, 'anomaly': ''}

    def _build_scene_tab(self):
        top = ttk.Frame(self.scene_tab)
        top.pack(fill='both', expand=True)

        # Task 1.2(b): build model list; grey and block non-allowed models.
        model_frame = ttk.LabelFrame(top, text='Models')
        model_frame.pack(side='left', fill='both', expand=True)
        self.model_list = tk.Listbox(model_frame, exportselection=False, height=10)
        self.model_list.pack(side='left', fill='both', expand=True)
        model_scroll = ttk.Scrollbar(model_frame, orient='vertical', command=self.model_list.yview)
        model_scroll.pack(side='right', fill='y')
        self.model_list.configure(yscrollcommand=model_scroll.set)
        # Ordering rule: allowed (importable) models first, greyed non-allowed
        # last — never interleaved. _row_to_midx maps display rows back to
        # node.models indices for on_model_select / default selection.
        indexed = list(enumerate(self.node.models))
        ordered = ([t for t in indexed if t[1]['key'] in ALLOWED_SCENE_MODELS]
                   + [t for t in indexed if t[1]['key'] not in ALLOWED_SCENE_MODELS])
        self.model_list._row_to_midx = []
        for midx, model in ordered:
            self.model_list.insert('end', model['key'])
            if model['key'] not in ALLOWED_SCENE_MODELS:
                self.model_list.itemconfig('end', foreground='gray60')
            self.model_list._row_to_midx.append(midx)
        self.model_list.bind('<<ListboxSelect>>', self.on_model_select)

        # Task 1.1: pass force_available_keys so restricted_zone renders normally.
        area_frame, self.scene_area_list = self._build_area_list(
            top,
            select_callback=self.on_scene_area_select,
            force_available_keys={'restricted_zone'},
        )
        area_frame.pack(side='left', fill='both', expand=True, padx=(10, 0))

        controls = ttk.LabelFrame(self.scene_tab, text='Placement')
        controls.pack(fill='x', pady=(10, 0))
        self._entry_row(controls, 0, 'Entity Name', self.name_var)
        self._entry_row(controls, 1, 'X', self.x_var)
        self._entry_row(controls, 1, 'Y', self.y_var, column=2)
        self._entry_row(controls, 1, 'Z', self.z_var, column=4)
        self._entry_row(controls, 2, 'Yaw', self.yaw_var)
        self._entry_row(controls, 2, 'Random Margin', self.margin_var, column=2)

        # Task 1.3: only Area Center and Random In Area; Manual + Allow Renaming
        # + Dry Run widgets are intentionally omitted.
        mode = ttk.Frame(controls)
        mode.grid(row=3, column=0, columnspan=6, sticky='w', pady=(8, 0))
        ttk.Radiobutton(mode, text='Area Center', variable=self.placement_var, value='center',
                        command=self.apply_placement).pack(side='left')
        ttk.Radiobutton(mode, text='Random In Area', variable=self.placement_var, value='random',
                        command=self.apply_placement).pack(side='left', padx=(10, 0))
        # allow_renaming_var remains (default False); no widget is shown.
        # placement_var 'manual' value still works if set programmatically.

        buttons = ttk.Frame(self.scene_tab)
        buttons.pack(fill='x', pady=(10, 0))
        ttk.Button(buttons, text='Use Center', command=self.use_center).pack(side='left')
        ttk.Button(buttons, text='Randomize', command=self.use_random).pack(side='left', padx=(8, 0))
        # Task 1.3: Dry Run button is hidden (dry_run_spawn method kept but unused).
        ttk.Button(buttons, text='Spawn', command=self.spawn).pack(side='right', padx=(0, 8))
        ttk.Label(self.scene_tab, textvariable=self.scene_status_var).pack(fill='x', pady=(8, 0))

        # Task 1.2(b): default to the first allowed (box) row. With allowed-first
        # ordering that is the leading row whenever any allowed model exists.
        first_allowed_row = next(
            (row for row, midx in enumerate(self.model_list._row_to_midx)
             if self.node.models[midx]['key'] in ALLOWED_SCENE_MODELS), None)
        if first_allowed_row is not None:
            midx = self.model_list._row_to_midx[first_allowed_row]
            self.model_list.selection_set(first_allowed_row)
            self.model_var.set(self.node.models[midx]['key'])
            self.name_var.set(unique_entity_name(
                self.node.models[midx]['key'], self.spawn_count + 1))
        elif self.node.models:
            self.model_list.selection_set(0)
            self.on_model_select()
        if self.node.area_items:
            self.scene_area_list.selection_set(0)
            self.on_scene_area_select()

    def _entry_row(self, parent, row, label, variable, column=0):
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky='w', padx=(0, 6), pady=3)
        ttk.Entry(parent, textvariable=variable, width=18).grid(
            row=row, column=column + 1, sticky='w', padx=(0, 14), pady=3)

    def selected_area_from_list(self, area_list):
        selection = area_list.curselection()
        if not selection:
            raise ValueError('Select a semantic area')
        # Display order is reordered (available-first); map the row back to the
        # global area_items index. Returns the global index so nav's typed-number
        # contract (target_var -> resolve_target) stays correct.
        gidx = area_list._row_to_gidx[selection[0]]
        return gidx, self.node.area_items[gidx]

    def on_nav_area_select(self, _event=None):
        try:
            index, (key, area) = self.selected_area_from_list(self.nav_area_list)
        except ValueError:
            return
        self.target_var.set(str(index + 1))
        self.show_area(key, area)

    def show_area(self, key: str, area: dict):
        bounds = area.get('bounds', {})
        lines = [
            f'key: {key}',
            f"name: {area.get('display_name', key)}",
            f"type: {area.get('type', 'unknown')}",
            f"marker: {area.get('marker_model', 'unknown')}",
            f"center: {area.get('center')}",
            f"size: {area.get('size')}",
            f"bounds: x[{bounds.get('x_min')}, {bounds.get('x_max')}], y[{bounds.get('y_min')}, {bounds.get('y_max')}]",
        ]
        self.nav_info_text.configure(state='normal')
        self.nav_info_text.delete('1.0', 'end')
        self.nav_info_text.insert('1.0', '\n'.join(lines) + '\n')
        self.nav_info_text.configure(state='disabled')

    def send_goal(self):
        try:
            key, area = self.node.resolve_target(self.target_var.get())
            goal = self.node.build_goal(area)
        except Exception as exc:  # noqa: BLE001
            self.status_var.set('Input error')
            self.detail_var.set(str(exc))
            return

        if not self.node.client.wait_for_server(timeout_sec=float(self.node.get_parameter('server_timeout_sec').value)):
            self.status_var.set('Nav2 unavailable')
            self.detail_var.set('NavigateToPose action server is not available yet')
            return

        self.pending_goal_key = key
        display_name = area.get('display_name', key)
        x = goal.pose.pose.position.x
        y = goal.pose.pose.position.y
        self.status_var.set(f'Sending goal: {display_name}')
        self.detail_var.set(f'{key} -> x={x:.3f}, y={y:.3f}')
        self.send_future = self.node.client.send_goal_async(goal)
        self.send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_var.set('Goal rejected')
            self.detail_var.set(self.pending_goal_key or '')
            return
        self.node.goal_handle = goal_handle
        self.status_var.set('Goal accepted')
        self.detail_var.set(self.pending_goal_key or '')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        result = future.result()
        status = STATUS_TEXT.get(result.status, str(result.status))
        self.status_var.set('Goal succeeded' if result.status == GoalStatus.STATUS_SUCCEEDED else 'Goal finished')
        self.detail_var.set(f'{self.pending_goal_key}: {status}')
        self.node.goal_handle = None

    def cancel_goal(self):
        goal_handle = self.node.goal_handle
        if goal_handle is None:
            self.status_var.set('No active goal')
            return
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _future: self.status_var.set('Cancel requested'))

    def add_selected_area_to_route(self):
        """Add all currently-selected inspect-area rows to the route.

        Task 2.1: rejects INSPECT_DISABLED_AREAS with a messagebox.
        Task 2.4: iterates curselection() so all rows selected in extended mode
                  are appended in one call; no duplicates.
        """
        selection = self.inspect_area_list.curselection()
        if not selection:
            messagebox.showerror('Route Error', 'Select a semantic area')
            return
        current = [item.strip() for item in self.get_route().split(',') if item.strip()]
        blocked = []
        for row in selection:
            gidx = self.inspect_area_list._row_to_gidx[row]
            key, area = self.node.area_items[gidx]
            if not area.get('accessible', True):
                blocked.append(f'{key} (walled off)')
                continue
            if key in INSPECT_DISABLED_AREAS:
                # Task 2.1: show one combined error at the end rather than per-item.
                blocked.append(f'{key} (not inspectable)')
                continue
            if key not in current:
                current.append(key)
        if blocked:
            messagebox.showerror(
                'Route Error',
                'The following areas cannot be added:\n' + '\n'.join(blocked))
        self.set_route(','.join(current))

    # ------------------------------------------------------------------
    # Task 2.3: Target Rooms route accessors (_route_text is canonical, so a
    # hand-typed route is honoured as well as one built via the buttons).
    # ------------------------------------------------------------------

    def get_route(self) -> str:
        """Canonical, normalized route string read from the editable Text.

        Newlines and ';' are treated as ',' so a multi-line/hand-typed route
        still resolves to a clean comma-separated list."""
        raw = self._route_text.get('1.0', 'end')
        parts = [p.strip() for p in
                 raw.replace('\n', ',').replace(';', ',').split(',') if p.strip()]
        return ','.join(parts)

    def set_route(self, value: str):
        """Replace the Target Rooms Text content (canonical route store)."""
        self._route_text.delete('1.0', 'end')
        if value:
            self._route_text.insert('1.0', value)

    # ------------------------------------------------------------------
    # Task 2.3/4: single Inspection Status render path. Callers pass only the
    # parts they changed; unspecified parts (None) keep their prior value, and
    # the whole box is re-rendered from self._status_state every time.
    # ------------------------------------------------------------------

    def _update_inspect_status(self, status: str | None = None,
                               report: str | None = None,
                               allocation_lines: list | None = None,
                               anomaly_text: str | None = None):
        st = self._status_state
        if status is not None:
            st['status'] = status
            self.inspect_status_var.set(status)
        if report is not None:
            st['report'] = report
            self.latest_report_var.set(report)
        if allocation_lines is not None:
            st['allocation_lines'] = allocation_lines
        if anomaly_text is not None:
            st['anomaly'] = anomaly_text
            self.anomaly_var.set(anomaly_text)

        self._status_text.configure(state='normal')
        self._status_text.delete('1.0', 'end')
        if st['status']:
            self._status_text.insert('end', st['status'] + '\n')
        if st['report']:
            self._status_text.insert('end', st['report'] + '\n')
        if st['allocation_lines']:
            self._status_text.insert('end', '本次分配 / Allocation:\n')
            for line in st['allocation_lines']:
                self._status_text.insert('end', '  ' + line + '\n')
        if st['anomaly']:
            self._status_text.insert('end', st['anomaly'] + '\n', 'anomaly')
        self._status_text.see('end')
        self._status_text.configure(state='disabled')

    # ------------------------------------------------------------------
    # Task 2.4: Select All helper for Inspect tab
    # ------------------------------------------------------------------

    def _inspect_select_all(self):
        """Select every inspectable row (accessible and not in INSPECT_DISABLED_AREAS).

        With available-first ordering these are the leading rows, but we map each
        display row via _row_to_gidx so it is correct regardless of order."""
        self.inspect_area_list.selection_clear(0, 'end')
        for row, gidx in enumerate(self.inspect_area_list._row_to_gidx):
            key, area = self.node.area_items[gidx]
            if area.get('accessible', True) and key not in INSPECT_DISABLED_AREAS:
                self.inspect_area_list.selection_set(row)

    def start_inspection(self):
        route = self.get_route().strip()
        if not route:
            messagebox.showerror('Inspection Error', 'Route is empty')
            return
        if self.inspect_mode_var.get() == 'auto':
            self.start_auto_inspection(route)
        else:
            self.start_manual_inspection(route)

    def start_auto_inspection(self, route: str):
        """Default dispatch: the operator names the rooms, the system splits
        the route across robots (task_allocator) and runs them concurrently."""
        if any(p and p.poll() is None for p in self.inspect_processes.values()):
            messagebox.showinfo('Inspection Running',
                                'An inspection is already running')
            return
        report_root = Path(self.node.get_parameter('report_dir').value)
        report_root.mkdir(parents=True, exist_ok=True)
        log_path = report_root / 'allocator_last_run.log'
        command = [
            'ros2', 'run', 'task_layer', 'task_allocator.py', '--ros-args',
            '-p', f'use_sim_time:={str(bool(self.node.get_parameter("use_sim_time").value)).lower()}',
            '-p', f'route:={route}',
            '-p', f'return_home:={str(bool(self.return_home_var.get())).lower()}',
            '-p', f'report_dir:={report_root}',
        ]
        try:
            log_file = open(log_path, 'w', encoding='utf-8')
            # start_new_session: own process group, so Abort can kill the
            # allocator AND the runner subprocesses it spawns (os.killpg).
            # Task 4: PYTHONUNBUFFERED=1 so allocation lines appear in the log
            # without waiting for the process to flush.
            self.inspect_processes['__auto__'] = subprocess.Popen(
                command, stdout=log_file, stderr=subprocess.STDOUT, text=True,
                start_new_session=True,
                env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            self.inspect_logs['__auto__'] = (log_file, log_path)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror('Inspection Error', str(exc))
            return
        # Task 4: reset the allocation-shown flag for this new run.
        self._allocation_shown = False
        self._update_inspect_status(
            status='[auto] allocating route across robots', report='')

    def start_manual_inspection(self, route: str):
        ns = self.node.active_robot
        label = ns or 'root'
        running = self.inspect_processes.get(ns)
        if running and running.poll() is None:
            messagebox.showinfo('Inspection Running',
                                f'[{label}] is already inspecting')
            return
        # Per-robot report dir; stdout goes to a file instead of a PIPE so a
        # long run can never deadlock on a full pipe buffer.
        report_dir = Path(self.node.get_parameter('report_dir').value) / (ns or 'single')
        report_dir.mkdir(parents=True, exist_ok=True)
        log_path = report_dir / 'last_run.log'
        command = ['ros2', 'run', 'task_layer', 'inspection_runner.py', '--ros-args']
        if ns:
            command += ['-r', f'__ns:=/{ns}']  # whole process joins the robot namespace
        command += [
            '-p', f'use_sim_time:={str(bool(self.node.get_parameter("use_sim_time").value)).lower()}',
            '-p', f'route:={route}',
            '-p', f'max_candidate_attempts_per_area:={self.max_attempts_var.get().strip()}',
            '-p', f'candidate_spread_ratio:={self.spread_ratio_var.get().strip()}',
            '-p', f'return_home:={str(bool(self.return_home_var.get())).lower()}',
            '-p', f'report_dir:={report_dir}',
        ]
        # Send the robot back to its own dock (see robots.yaml home_pose);
        # without this the runner falls back to the shared robot_start.
        home = (self.node.robot_registry.get(ns) or {}).get('home_pose') or {}
        if {'x', 'y'} <= home.keys():
            command += [
                '-p', f'home_x:={float(home["x"])}',
                '-p', f'home_y:={float(home["y"])}',
                '-p', f'home_yaw:={float(home.get("yaw", 0.0))}',
            ]
        try:
            log_file = open(log_path, 'w', encoding='utf-8')
            self.inspect_processes[ns] = subprocess.Popen(
                command, stdout=log_file, stderr=subprocess.STDOUT, text=True,
                start_new_session=True)
            self.inspect_logs[ns] = (log_file, log_path)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror('Inspection Error', str(exc))
            return
        # Task 4: immediately show the robot -> route allocation in manual mode.
        self._update_inspect_status(
            status=f'[{label}] inspection running',
            report='',
            allocation_lines=[f'{label} -> {route}'])

    def poll_inspection(self):
        # -- Anomaly updates (Task 2.3: rendered in red via Text tag) ----------
        events = self.node.anomaly_events
        anomaly_text = ''
        if events:
            lines = [
                '⚠ %s @ %s (%.2f, %.2f)' % (
                    e.get('robot', '?'), e.get('area', '?'),
                    float(e.get('x', 0.0)), float(e.get('y', 0.0)))
                for e in events[-3:]
            ]
            anomaly_text = 'Anomalies: %d\n%s' % (len(events), '\n'.join(lines))
        if len(events) != self._anomaly_seen:
            self._anomaly_seen = len(events)
            # Single render path: update only the anomaly part of the snapshot.
            self._update_inspect_status(anomaly_text=anomaly_text)

        # -- Task 4: while __auto__ is still running, show allocation lines ----
        auto_proc = self.inspect_processes.get('__auto__')
        if auto_proc and auto_proc.poll() is None and not self._allocation_shown:
            _lf, log_path = self.inspect_logs.get('__auto__', (None, None))
            if log_path and log_path.exists():
                try:
                    raw = log_path.read_text(encoding='utf-8', errors='replace')
                    alloc_lines = [
                        line.split('Allocation:', 1)[1].strip()
                        for line in raw.splitlines() if 'Allocation:' in line
                    ]
                    if alloc_lines:
                        self._allocation_shown = True
                        self._update_inspect_status(
                            status='[auto] inspection running — allocation received',
                            allocation_lines=alloc_lines,
                            anomaly_text=anomaly_text)
                except OSError:
                    pass

        # -- Process completion -----------------------------------------------
        for ns, process in list(self.inspect_processes.items()):
            if process is None:
                continue
            return_code = process.poll()
            if return_code is None:
                continue
            log_file, log_path = self.inspect_logs.pop(ns, (None, None))
            if log_file:
                log_file.close()
            output = ''
            if log_path and log_path.exists():
                output = log_path.read_text(encoding='utf-8', errors='replace')
            label = 'auto' if ns == '__auto__' else (ns or 'root')
            status_msg = f'[{label}] inspection finished: code {return_code}'
            if ns == '__auto__':
                mission_report = ''
                for line in output.splitlines():
                    if 'Mission report written:' in line:
                        mission_report = line.split(
                            'Mission report written:', 1)[1].strip()
                alloc_lines = [
                    line.split('Allocation:', 1)[1].strip()
                    for line in output.splitlines() if 'Allocation:' in line
                ]
                report_str = (mission_report or ' | '.join(alloc_lines)
                              or output.strip()[-300:])
                self._update_inspect_status(
                    status=status_msg,
                    report=report_str,
                    allocation_lines=alloc_lines if alloc_lines else None,
                    anomaly_text=anomaly_text)
            else:
                report_line = self.extract_report_line(output)
                self._update_inspect_status(
                    status=status_msg,
                    report=report_line or output.strip()[-300:],
                    anomaly_text=anomaly_text)
            self.inspect_processes[ns] = None
        self.root.after(500, self.poll_inspection)

    def extract_report_line(self, output: str) -> str:
        for line in output.splitlines():
            if 'Inspection report written:' in line:
                return line.split('Inspection report written:', 1)[1].strip()
        return ''

    def set_report_dir_status(self):
        report_dir = str(self.node.get_parameter('report_dir').value)
        self.latest_report_var.set(report_dir)
        self._update_inspect_status(report=report_dir)

    def selected_scene_area(self):
        _index, pair = self.selected_area_from_list(self.scene_area_list)
        return pair

    def on_model_select(self, _event=None):
        selection = self.model_list.curselection()
        if not selection:
            return
        model_key = self.node.models[self.model_list._row_to_midx[selection[0]]]['key']
        # Task 1.2(b): silently reject selection of non-allowed models.
        if model_key not in ALLOWED_SCENE_MODELS:
            self.model_list.selection_clear(0, 'end')
            return
        self.model_var.set(model_key)
        self.name_var.set(unique_entity_name(model_key, self.spawn_count + 1))

    def on_scene_area_select(self, _event=None):
        if self.placement_var.get() != 'manual':
            self.apply_placement()

    def apply_placement(self):
        if self.placement_var.get() == 'center':
            self.use_center()
        elif self.placement_var.get() == 'random':
            self.use_random()

    def use_center(self):
        _key, area = self.selected_scene_area()
        x, y = area_center(area)
        self.set_xy(x, y)
        self.placement_var.set('center')

    def use_random(self):
        _key, area = self.selected_scene_area()
        x, y = area_random(area, float(self.margin_var.get()))
        self.set_xy(x, y)
        self.placement_var.set('random')

    def set_xy(self, x: float, y: float):
        self.x_var.set('%.3f' % x)
        self.y_var.set('%.3f' % y)

    def build_spawn_params(self) -> dict:
        model_key = self.model_var.get()
        model_file = resolve_model_file(model_key)
        entity_name = self.name_var.get().strip() or unique_entity_name(model_key, self.spawn_count + 1)
        return {
            'world': self.node.get_parameter('world').value,
            'file': model_file,
            'name': entity_name,
            'allow_renaming': bool(self.allow_renaming_var.get()),
            'x': float(self.x_var.get()),
            'y': float(self.y_var.get()),
            'z': float(self.z_var.get()),
            'R': 0.0,
            'P': 0.0,
            'Y': float(self.yaw_var.get()),
        }

    def dry_run_spawn(self):
        try:
            params = self.build_spawn_params()
            self.scene_status_var.set(' '.join(make_spawn_command(params)))
        except Exception as exc:
            messagebox.showerror('Dry Run Error', str(exc))

    def spawn(self):
        try:
            params = self.build_spawn_params()
            self.spawn_count += 1
            if not self.allow_renaming_var.get():
                params['name'] = unique_entity_name(self.model_var.get(), self.spawn_count)
                self.name_var.set(params['name'])
            self.scene_status_var.set('Spawning %s at x=%.3f y=%.3f' % (params['name'], params['x'], params['y']))
            return_code = run_spawn(params)
            if return_code == 0:
                self.scene_status_var.set('Spawned %s' % params['name'])
                self.name_var.set(unique_entity_name(self.model_var.get(), self.spawn_count + 1))
            else:
                self.scene_status_var.set('Spawn failed with code %d' % return_code)
        except Exception as exc:
            messagebox.showerror('Spawn Error', str(exc))

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.root.after(50, self.spin_ros)

    def on_robot_select(self, _event=None):
        value = self.robot_var.get()
        self.node.active_robot = '' if value == '(root)' else value
        self.status_var.set(f'Active robot: {value}')

    def _kill_process_group(self, process) -> None:
        """SIGINT the whole process group (allocator + its runner children),
        escalating to SIGKILL if it does not exit promptly."""
        if not process or process.poll() is not None:
            return
        try:
            pgid = os.getpgid(process.pid)
        except (ProcessLookupError, OSError):
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(pgid, sig)
            except (ProcessLookupError, OSError):
                return
            try:
                process.wait(timeout=3.0)
                return
            except subprocess.TimeoutExpired:
                continue

    def _latest_run_dir(self) -> Path | None:
        """Newest mission_*/inspection_* directory under report_dir, to drop an
        abort marker into the run we just killed."""
        root = Path(self.node.get_parameter('report_dir').value)
        if not root.exists():
            return None
        candidates = [p for p in root.glob('**/mission_*') if p.is_dir()]
        candidates += [p for p in root.glob('**/inspection_*') if p.is_dir()]
        if not candidates:
            return None
        return max(candidates, key=lambda p: p.stat().st_mtime)

    def abort_and_reset_to_dock(self):
        """Abandon the running inspection and hard-reset every robot to its dock
        (teleport + re-seed AMCL + clear costmaps). Recovers from wedge / loop /
        AMCL drift without restarting the stack. SIMULATION ONLY."""
        running = [p for p in self.inspect_processes.values()
                   if p and p.poll() is None]
        if not messagebox.askyesno(
                'Abort & Reset',
                'Abandon the current inspection (if any) and hard-reset all '
                'robots to their docks?\n\n(Teleport + relocalise + clear '
                'costmaps. Simulation only.)'):
            return

        # 1. Kill the running inspection process group(s).
        for process in running:
            self._kill_process_group(process)
        self.inspect_processes.clear()

        # 2. Mark the interrupted run as aborted (do not fake a full report).
        run_dir = self._latest_run_dir()
        if run_dir is not None:
            try:
                stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                (run_dir / 'ABORTED.txt').write_text(
                    f'Mission manually aborted at {stamp} via GUI '
                    '"Abort & Reset to Dock".\n'
                    'Robots were teleported back to their docks; this run is '
                    'INCOMPLETE -- any partial photos/yaml here are not a valid '
                    'inspection result.\n', encoding='utf-8')
            except OSError:
                pass

        # 3. Hard-reset every robot to its dock.
        report_root = Path(self.node.get_parameter('report_dir').value)
        report_root.mkdir(parents=True, exist_ok=True)
        log_path = report_root / 'reset_to_dock_last_run.log'
        command = [
            'ros2', 'run', 'task_layer', 'reset_to_dock.py', '--ros-args',
            '-p', f'use_sim_time:={str(bool(self.node.get_parameter("use_sim_time").value)).lower()}',
        ]
        try:
            log_file = open(log_path, 'w', encoding='utf-8')
            subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT,
                             text=True, start_new_session=True)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror('Reset Error', str(exc))
            return
        marked = f' (marked {run_dir.name} aborted)' if run_dir is not None else ''
        self._update_inspect_status(
            status=f'Aborted; resetting all robots to docks…{marked}')

    def close(self):
        for process in self.inspect_processes.values():
            self._kill_process_group(process)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TaskGuiNode()
    gui = TaskGui(node)
    try:
        gui.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
