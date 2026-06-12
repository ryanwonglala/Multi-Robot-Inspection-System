from __future__ import annotations

from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node

from task_layer.model_spawner import (
    area_center,
    area_items,
    area_random,
    list_builtin_models,
    load_world_model,
    make_spawn_command,
    resolve_model_file,
    run_spawn,
    unique_entity_name,
)


class SceneBuilderGuiNode(Node):
    def __init__(self):
        super().__init__('scene_builder_gui')
        self.declare_parameter('world', 'map')
        self.declare_parameter('world_model_path', '')
        path = self.get_parameter('world_model_path').value or None
        self.world_model = load_world_model(path)
        self.areas = area_items(self.world_model)
        self.models = list_builtin_models()


class SceneBuilderGui:
    def __init__(self, node: SceneBuilderGuiNode):
        self.node = node
        self.spawn_count = 0

        self.root = tk.Tk()
        self.root.title('Gazebo Scene Builder')
        self.root.minsize(760, 520)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        default_model = self.node.models[0]['key'] if self.node.models else ''
        self.model_var = tk.StringVar(value=default_model)
        self.name_var = tk.StringVar(value='')
        self.area_var = tk.StringVar()
        self.placement_var = tk.StringVar(value='center')
        self.x_var = tk.StringVar(value='0.000')
        self.y_var = tk.StringVar(value='0.000')
        self.z_var = tk.StringVar(value='0.250')
        self.yaw_var = tk.StringVar(value='0.000')
        self.margin_var = tk.StringVar(value='0.200')
        self.allow_renaming_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value='Ready')

        self._build()
        self._select_first_area()
        self.root.after(50, self.spin_ros)

    def _build(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        top = ttk.Frame(main)
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

        area_frame = ttk.LabelFrame(top, text='Semantic Areas')
        area_frame.pack(side='left', fill='both', expand=True, padx=(10, 0))
        self.area_list = tk.Listbox(area_frame, exportselection=False, height=10)
        self.area_list.pack(side='left', fill='both', expand=True)
        area_scroll = ttk.Scrollbar(area_frame, orient='vertical', command=self.area_list.yview)
        area_scroll.pack(side='right', fill='y')
        self.area_list.configure(yscrollcommand=area_scroll.set)
        for index, (key, area) in enumerate(self.node.areas, start=1):
            name = area.get('display_name', key)
            self.area_list.insert('end', f'{index:02d}. {key} | {name}')
        self.area_list.bind('<<ListboxSelect>>', self.on_area_select)

        controls = ttk.LabelFrame(main, text='Placement')
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
        ttk.Radiobutton(mode, text='Manual', variable=self.placement_var, value='manual').pack(
            side='left', padx=(10, 0))
        ttk.Checkbutton(mode, text='Allow Renaming', variable=self.allow_renaming_var).pack(
            side='left', padx=(20, 0))

        buttons = ttk.Frame(main)
        buttons.pack(fill='x', pady=(10, 0))
        ttk.Button(buttons, text='Use Center', command=self.use_center).pack(side='left')
        ttk.Button(buttons, text='Randomize', command=self.use_random).pack(side='left', padx=(8, 0))
        ttk.Button(buttons, text='Dry Run', command=self.dry_run).pack(side='right')
        ttk.Button(buttons, text='Spawn', command=self.spawn).pack(side='right', padx=(0, 8))

        detail = ttk.LabelFrame(main, text='Area Detail')
        detail.pack(fill='both', expand=True, pady=(10, 0))
        self.detail_text = tk.Text(detail, height=7, wrap='word')
        self.detail_text.pack(fill='both', expand=True)
        self.detail_text.configure(state='disabled')

        ttk.Label(main, textvariable=self.status_var).pack(fill='x', pady=(8, 0))

    def _entry_row(self, parent, row, label, variable, column=0):
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky='w', padx=(0, 6), pady=3)
        ttk.Entry(parent, textvariable=variable, width=18).grid(
            row=row, column=column + 1, sticky='w', padx=(0, 14), pady=3)

    def _select_first_area(self):
        if self.node.models:
            self.model_list.selection_set(0)
            self.on_model_select()
        if self.node.areas:
            self.area_list.selection_set(0)
            self.on_area_select()

    def selected_area(self):
        selection = self.area_list.curselection()
        if not selection:
            raise ValueError('Select a semantic area')
        return self.node.areas[selection[0]]

    def on_model_select(self, _event=None):
        selection = self.model_list.curselection()
        if not selection:
            return
        model_key = self.node.models[selection[0]]['key']
        self.model_var.set(model_key)
        self.name_var.set(unique_entity_name(model_key, self.spawn_count + 1))

    def on_area_select(self, _event=None):
        key, area = self.selected_area()
        self.area_var.set(key)
        bounds = area.get('bounds', {})
        lines = [
            f'key: {key}',
            f"name: {area.get('display_name', key)}",
            f"type: {area.get('type', 'unknown')}",
            f'center: {area.get("center")}',
            f'size: {area.get("size")}',
            'bounds: x[{x_min}, {x_max}], y[{y_min}, {y_max}]'.format(**bounds),
        ]
        self.detail_text.configure(state='normal')
        self.detail_text.delete('1.0', 'end')
        self.detail_text.insert('1.0', '\n'.join(lines))
        self.detail_text.configure(state='disabled')
        if self.placement_var.get() != 'manual':
            self.apply_placement()

    def apply_placement(self):
        if self.placement_var.get() == 'center':
            self.use_center()
        elif self.placement_var.get() == 'random':
            self.use_random()

    def use_center(self):
        _key, area = self.selected_area()
        x, y = area_center(area)
        self.set_xy(x, y)
        self.placement_var.set('center')

    def use_random(self):
        _key, area = self.selected_area()
        x, y = area_random(area, float(self.margin_var.get()))
        self.set_xy(x, y)
        self.placement_var.set('random')

    def set_xy(self, x: float, y: float):
        self.x_var.set('%.3f' % x)
        self.y_var.set('%.3f' % y)

    def build_params(self) -> dict:
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

    def dry_run(self):
        try:
            params = self.build_params()
            self.status_var.set(' '.join(make_spawn_command(params)))
        except Exception as exc:
            messagebox.showerror('Dry Run Error', str(exc))

    def spawn(self):
        try:
            params = self.build_params()
            self.spawn_count += 1
            if not self.allow_renaming_var.get():
                params['name'] = unique_entity_name(self.model_var.get(), self.spawn_count)
                self.name_var.set(params['name'])
            self.status_var.set(
                'Spawning %s at x=%.3f y=%.3f' % (params['name'], params['x'], params['y']))
            return_code = run_spawn(params)
            if return_code == 0:
                self.status_var.set('Spawned %s' % params['name'])
                self.name_var.set(unique_entity_name(self.model_var.get(), self.spawn_count + 1))
            else:
                self.status_var.set('Spawn failed with code %d' % return_code)
        except Exception as exc:
            messagebox.showerror('Spawn Error', str(exc))

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(50, self.spin_ros)

    def close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = SceneBuilderGuiNode()
    try:
        gui = SceneBuilderGui(node)
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
