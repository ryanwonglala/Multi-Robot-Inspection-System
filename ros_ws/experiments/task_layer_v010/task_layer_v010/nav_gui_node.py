#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


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
    share_dir = get_package_share_directory('task_layer_v010')
    return str(Path(share_dir) / 'config' / 'world_model.yaml')


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def normalize_text(value: str) -> str:
    return value.strip().lower().replace(' ', '_').replace('-', '_')


class NavGuiNode(Node):
    def __init__(self):
        super().__init__('nav_gui')
        self.declare_parameter('world_model_path', default_world_model_path())
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('server_timeout_sec', 2.0)

        self.world_model = self.load_world_model()
        self.areas = self.world_model.get('areas', {})
        self.area_items = list(self.areas.items())
        action_name = self.get_parameter('action_name').value
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.goal_handle = None

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
            return self.area_items[index - 1]

        normalized = normalize_text(query)
        for key, area in self.area_items:
            names = {
                normalize_text(key),
                normalize_text(area.get('display_name', key)),
                normalize_text(area.get('marker_model', '')),
            }
            if normalized in names:
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


class NavGui:
    def __init__(self, node: NavGuiNode):
        self.node = node
        self.pending_goal_key = None
        self.result_future = None
        self.send_future = None

        self.root = tk.Tk()
        self.root.title('Task Layer Nav GUI')
        self.root.minsize(620, 460)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.target_var = tk.StringVar()
        self.status_var = tk.StringVar(value='Ready')
        self.detail_var = tk.StringVar(value='Select an area or type number/name')

        self._build()
        self.root.after(50, self.spin_ros)

    def _build(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        input_row = ttk.Frame(main)
        input_row.pack(fill='x', pady=(0, 8))
        ttk.Label(input_row, text='Target').pack(side='left')
        entry = ttk.Entry(input_row, textvariable=self.target_var)
        entry.pack(side='left', fill='x', expand=True, padx=8)
        entry.bind('<Return>', lambda _event: self.send_goal())
        ttk.Button(input_row, text='Go', command=self.send_goal).pack(side='left')
        ttk.Button(input_row, text='Cancel', command=self.cancel_goal).pack(side='left', padx=(6, 0))

        body = ttk.Frame(main)
        body.pack(fill='both', expand=True)

        list_frame = ttk.LabelFrame(body, text='Areas')
        list_frame.pack(side='left', fill='both', expand=True)
        self.area_list = tk.Listbox(list_frame, activestyle='dotbox', exportselection=False)
        self.area_list.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(list_frame, orient='vertical', command=self.area_list.yview)
        scrollbar.pack(side='right', fill='y')
        self.area_list.configure(yscrollcommand=scrollbar.set)
        self.area_list.bind('<<ListboxSelect>>', self.on_area_select)
        self.area_list.bind('<Double-Button-1>', lambda _event: self.send_goal())

        for index, (key, area) in enumerate(self.node.area_items, start=1):
            name = area.get('display_name', key)
            center = area.get('center', ['?', '?'])
            self.area_list.insert('end', f'{index:02d}. {key}  |  {name}  |  ({center[0]}, {center[1]})')

        info = ttk.LabelFrame(body, text='Selected')
        info.pack(side='left', fill='both', expand=True, padx=(10, 0))
        self.info_text = tk.Text(info, height=12, wrap='word')
        self.info_text.pack(fill='both', expand=True)
        self.info_text.configure(state='disabled')

        status = ttk.LabelFrame(main, text='Status')
        status.pack(fill='x', pady=(8, 0))
        ttk.Label(status, textvariable=self.status_var).pack(anchor='w')
        ttk.Label(status, textvariable=self.detail_var).pack(anchor='w')

    def on_area_select(self, _event=None):
        selection = self.area_list.curselection()
        if not selection:
            return
        index = selection[0]
        key, area = self.node.area_items[index]
        self.target_var.set(str(index + 1))
        self.show_area(key, area)

    def show_area(self, key: str, area: dict):
        bounds = area.get('bounds', {})
        lines = [
            f"key: {key}",
            f"name: {area.get('display_name', key)}",
            f"type: {area.get('type', 'unknown')}",
            f"marker: {area.get('marker_model', 'unknown')}",
            f"center: {area.get('center')}",
            f"size: {area.get('size')}",
            f"bounds: x[{bounds.get('x_min')}, {bounds.get('x_max')}], y[{bounds.get('y_min')}, {bounds.get('y_max')}]",
        ]
        text = '\n'.join(lines) + '\n'
        self.info_text.configure(state='normal')
        self.info_text.delete('1.0', 'end')
        self.info_text.insert('1.0', text)
        self.info_text.configure(state='disabled')

    def send_goal(self):
        try:
            key, area = self.node.resolve_target(self.target_var.get())
            goal = self.node.build_goal(area)
        except Exception as exc:  # noqa: BLE001 - display prototype errors directly.
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
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.status_var.set('Goal succeeded')
        else:
            self.status_var.set('Goal finished')
        self.detail_var.set(f'{self.pending_goal_key}: {status}')
        self.node.goal_handle = None

    def cancel_goal(self):
        goal_handle = self.node.goal_handle
        if goal_handle is None:
            self.status_var.set('No active goal')
            return
        future = goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _future: self.status_var.set('Cancel requested'))

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.root.after(50, self.spin_ros)

    def close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = NavGuiNode()
    gui = NavGui(node)
    try:
        gui.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
