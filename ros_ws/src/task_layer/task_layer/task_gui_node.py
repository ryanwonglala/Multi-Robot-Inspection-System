#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
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

        self.root = tk.Tk()
        self.root.title('RoboInspect Task GUI')
        self.root.minsize(860, 620)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.status_var = tk.StringVar(value='Ready')
        self.detail_var = tk.StringVar(value='')
        self.target_var = tk.StringVar()
        self.inspect_route_var = tk.StringVar()
        self.inspect_mode_var = tk.StringVar(value='auto')
        self.max_attempts_var = tk.StringVar(value='2')
        self.spread_ratio_var = tk.StringVar(value='0.35')
        self.return_home_var = tk.BooleanVar(value=True)
        self.inspect_status_var = tk.StringVar(value='Ready')
        self.latest_report_var = tk.StringVar(value='')
        self.model_var = tk.StringVar(value=self.node.models[0]['key'] if self.node.models else '')
        self.name_var = tk.StringVar(value='')
        self.area_var = tk.StringVar()
        self.placement_var = tk.StringVar(value='center')
        self.x_var = tk.StringVar(value='0.000')
        self.y_var = tk.StringVar(value='0.000')
        self.z_var = tk.StringVar(value='0.250')
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

    def _build_area_list(self, parent, select_callback=None, double_callback=None):
        frame = ttk.LabelFrame(parent, text='Areas')
        area_list = tk.Listbox(frame, activestyle='dotbox', exportselection=False)
        area_list.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(frame, orient='vertical', command=area_list.yview)
        scrollbar.pack(side='right', fill='y')
        area_list.configure(yscrollcommand=scrollbar.set)
        if select_callback:
            area_list.bind('<<ListboxSelect>>', select_callback)
        if double_callback:
            area_list.bind('<Double-Button-1>', double_callback)
        for index, (key, area) in enumerate(self.node.area_items, start=1):
            name = area.get('display_name', key)
            center = area.get('center', ['?', '?'])
            if area.get('accessible', True):
                label = f'{index:02d}. {key} | {name} | ({center[0]}, {center[1]})'
                area_list.insert('end', label)
            else:
                area_list.insert('end', f'{index:02d}. {key} | {name} | (walled off)')
                area_list.itemconfig('end', foreground='gray60')
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
        list_frame, self.inspect_area_list = self._build_area_list(
            body,
            select_callback=None,
            double_callback=lambda _event: self.add_selected_area_to_route(),
        )
        list_frame.pack(side='left', fill='both', expand=True)

        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True, padx=(10, 0))
        route_frame = ttk.LabelFrame(right, text='Target Rooms')
        route_frame.pack(fill='x')
        ttk.Entry(route_frame, textvariable=self.inspect_route_var).pack(fill='x', padx=6, pady=6)
        route_buttons = ttk.Frame(route_frame)
        route_buttons.pack(fill='x', padx=6, pady=(0, 6))
        ttk.Button(route_buttons, text='Add Selected', command=self.add_selected_area_to_route).pack(side='left')
        ttk.Button(route_buttons, text='Clear', command=lambda: self.inspect_route_var.set('')).pack(side='left', padx=(6, 0))

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
        self._entry_row(params, 0, 'Max Attempts', self.max_attempts_var)
        self._entry_row(params, 1, 'Spread Ratio', self.spread_ratio_var)
        ttk.Checkbutton(params, text='Return Home', variable=self.return_home_var).grid(
            row=2, column=0, columnspan=2, sticky='w', pady=4)

        buttons = ttk.Frame(right)
        buttons.pack(fill='x', pady=(10, 0))
        ttk.Button(buttons, text='Start Inspection', command=self.start_inspection).pack(side='left')
        ttk.Button(buttons, text='Abort & Reset to Dock',
                   command=self.abort_and_reset_to_dock).pack(side='left', padx=(8, 0))
        ttk.Button(buttons, text='Open Report Dir', command=self.set_report_dir_status).pack(side='left', padx=(8, 0))

        report = ttk.LabelFrame(right, text='Inspection Status')
        report.pack(fill='both', expand=True, pady=(10, 0))
        ttk.Label(report, textvariable=self.inspect_status_var).pack(anchor='w', padx=6, pady=(6, 0))
        ttk.Label(report, textvariable=self.latest_report_var, wraplength=420).pack(anchor='w', padx=6, pady=(4, 6))

    def _build_scene_tab(self):
        top = ttk.Frame(self.scene_tab)
        top.pack(fill='both', expand=True)

        model_frame = ttk.LabelFrame(top, text='Models')
        model_frame.pack(side='left', fill='both', expand=True)
        self.model_list = tk.Listbox(model_frame, exportselection=False, height=10)
        self.model_list.pack(side='left', fill='both', expand=True)
        model_scroll = ttk.Scrollbar(model_frame, orient='vertical', command=self.model_list.yview)
        model_scroll.pack(side='right', fill='y')
        self.model_list.configure(yscrollcommand=model_scroll.set)
        for model in self.node.models:
            self.model_list.insert('end', model['key'])
        self.model_list.bind('<<ListboxSelect>>', self.on_model_select)

        area_frame, self.scene_area_list = self._build_area_list(top, select_callback=self.on_scene_area_select)
        area_frame.pack(side='left', fill='both', expand=True, padx=(10, 0))

        controls = ttk.LabelFrame(self.scene_tab, text='Placement')
        controls.pack(fill='x', pady=(10, 0))
        self._entry_row(controls, 0, 'Entity Name', self.name_var)
        self._entry_row(controls, 1, 'X', self.x_var)
        self._entry_row(controls, 1, 'Y', self.y_var, column=2)
        self._entry_row(controls, 1, 'Z', self.z_var, column=4)
        self._entry_row(controls, 2, 'Yaw', self.yaw_var)
        self._entry_row(controls, 2, 'Random Margin', self.margin_var, column=2)

        mode = ttk.Frame(controls)
        mode.grid(row=3, column=0, columnspan=6, sticky='w', pady=(8, 0))
        ttk.Radiobutton(mode, text='Area Center', variable=self.placement_var, value='center',
                        command=self.apply_placement).pack(side='left')
        ttk.Radiobutton(mode, text='Random In Area', variable=self.placement_var, value='random',
                        command=self.apply_placement).pack(side='left', padx=(10, 0))
        ttk.Radiobutton(mode, text='Manual', variable=self.placement_var, value='manual').pack(side='left', padx=(10, 0))
        ttk.Checkbutton(mode, text='Allow Renaming', variable=self.allow_renaming_var).pack(side='left', padx=(20, 0))

        buttons = ttk.Frame(self.scene_tab)
        buttons.pack(fill='x', pady=(10, 0))
        ttk.Button(buttons, text='Use Center', command=self.use_center).pack(side='left')
        ttk.Button(buttons, text='Randomize', command=self.use_random).pack(side='left', padx=(8, 0))
        ttk.Button(buttons, text='Dry Run', command=self.dry_run_spawn).pack(side='right')
        ttk.Button(buttons, text='Spawn', command=self.spawn).pack(side='right', padx=(0, 8))
        ttk.Label(self.scene_tab, textvariable=self.scene_status_var).pack(fill='x', pady=(8, 0))

        if self.node.models:
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
        return selection[0], self.node.area_items[selection[0]]

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
        try:
            _index, (key, area) = self.selected_area_from_list(self.inspect_area_list)
        except ValueError as exc:
            messagebox.showerror('Route Error', str(exc))
            return
        if not area.get('accessible', True):
            messagebox.showerror('Route Error', f'{key} is walled off in the current map')
            return
        current = [item.strip() for item in self.inspect_route_var.get().split(',') if item.strip()]
        current.append(key)
        self.inspect_route_var.set(','.join(current))

    def start_inspection(self):
        route = self.inspect_route_var.get().strip()
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
            self.inspect_processes['__auto__'] = subprocess.Popen(
                command, stdout=log_file, stderr=subprocess.STDOUT, text=True,
                start_new_session=True)
            self.inspect_logs['__auto__'] = (log_file, log_path)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror('Inspection Error', str(exc))
            return
        self.inspect_status_var.set('[auto] allocating route across robots')
        self.latest_report_var.set('')

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
        self.inspect_status_var.set(f'[{label}] inspection running')
        self.latest_report_var.set('')

    def poll_inspection(self):
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
            self.inspect_status_var.set(f'[{label}] inspection finished: code {return_code}')
            if ns == '__auto__':
                mission_report = ''
                for line in output.splitlines():
                    if 'Mission report written:' in line:
                        mission_report = line.split(
                            'Mission report written:', 1)[1].strip()
                allocation = [line.split('Allocation:', 1)[1].strip()
                              for line in output.splitlines() if 'Allocation:' in line]
                self.latest_report_var.set(
                    mission_report or ' | '.join(allocation) or output.strip()[-300:])
            else:
                report_line = self.extract_report_line(output)
                self.latest_report_var.set(report_line or output.strip()[-300:])
            self.inspect_processes[ns] = None
        self.root.after(500, self.poll_inspection)

    def extract_report_line(self, output: str) -> str:
        for line in output.splitlines():
            if 'Inspection report written:' in line:
                return line.split('Inspection report written:', 1)[1].strip()
        return ''

    def set_report_dir_status(self):
        self.latest_report_var.set(str(self.node.get_parameter('report_dir').value))

    def selected_scene_area(self):
        _index, pair = self.selected_area_from_list(self.scene_area_list)
        return pair

    def on_model_select(self, _event=None):
        selection = self.model_list.curselection()
        if not selection:
            return
        model_key = self.node.models[selection[0]]['key']
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
        self.inspect_status_var.set(
            f'Aborted; resetting all robots to docks…{marked}')

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
