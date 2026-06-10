#!/usr/bin/env python3
"""Press-and-hold Tkinter drive GUI for manual SLAM mapping with TurtleBot3."""
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

HZ = 10
WIN_W, WIN_H = 320, 480


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class MappingCtlNode(Node):
    def __init__(self):
        super().__init__('mapping_ctl')
        self.declare_parameter('use_sim_time', True)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        # Drive state — written by GUI thread, read by timer
        self.lin = 0.0
        self.ang = 0.0

        # Display state — written by ROS callbacks, read by GUI thread
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.map_w = 0
        self.map_h = 0
        self.map_res = 0.0

        self._lock = threading.Lock()
        self.create_timer(1.0 / HZ, self._publish)

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = math.atan2(siny, cosy)

    def _map_cb(self, msg):
        with self._lock:
            self.map_w = msg.info.width
            self.map_h = msg.info.height
            self.map_res = msg.info.resolution

    def _publish(self):
        msg = Twist()
        with self._lock:
            msg.linear.x = self.lin
            msg.angular.z = self.ang
        self.pub.publish(msg)

    def stop(self):
        with self._lock:
            self.lin = 0.0
            self.ang = 0.0
        self.pub.publish(Twist())

    def set_vel(self, lin, ang):
        with self._lock:
            self.lin = lin
            self.ang = ang

    def get_pose(self):
        with self._lock:
            return self.x, self.y, self.yaw

    def get_map_info(self):
        with self._lock:
            return self.map_w, self.map_h, self.map_res

    def get_vel(self):
        with self._lock:
            return self.lin, self.ang


# ── Tkinter GUI ───────────────────────────────────────────────────────────────

class MappingCtlGUI:
    def __init__(self, node: MappingCtlNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title('TB3 Mapping Controller')
        self.root.resizable(False, False)
        self.root.geometry(f'{WIN_W}x{WIN_H}')
        self.root.attributes('-topmost', True)

        self._build_ui()
        self._bind_keys()

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.after(200, self._refresh)

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self):
        pad = {'padx': 6, 'pady': 4}

        # ── D-pad ────────────────────────────────────────────────────────
        dpad = tk.LabelFrame(self.root, text='Drive  (hold = move, release = stop)')
        dpad.pack(fill='x', **pad)

        btn = dict(width=8, height=3, relief='raised')
        stop_btn = dict(width=5, height=3, bg='#c0392b', fg='white',
                        activebackground='#e74c3c', activeforeground='white',
                        relief='raised')

        self._make_hold_btn(dpad, '↑\nForward', btn, 0, 1,
                            lin=self._lin_val, ang=lambda: 0.0)
        self._make_hold_btn(dpad, '← Left', btn, 1, 0,
                            lin=lambda: 0.0, ang=self._ang_val)
        self._make_stop_btn(dpad, '■\nSTOP', stop_btn, 1, 1)
        self._make_hold_btn(dpad, 'Right →', btn, 1, 2,
                            lin=lambda: 0.0, ang=lambda: -self._ang_val())
        self._make_hold_btn(dpad, '↓\nBackward', btn, 2, 1,
                            lin=lambda: -self._lin_val(), ang=lambda: 0.0)

        # ── Spin row ──────────────────────────────────────────────────────
        spin_frame = tk.Frame(dpad)
        spin_frame.grid(row=3, column=0, columnspan=3, pady=(4, 2))

        spin_btn = dict(width=10, height=2, relief='raised')
        self._make_hold_btn(spin_frame, '↺  Spin Left', spin_btn, 0, 0,
                            lin=lambda: 0.0, ang=self._ang_val, pack=True)
        self._make_hold_btn(spin_frame, 'Spin Right  ↻', spin_btn, 0, 1,
                            lin=lambda: 0.0, ang=lambda: -self._ang_val(), pack=True)

        # ── Speed sliders ─────────────────────────────────────────────────
        sliders = tk.LabelFrame(self.root, text='Speed')
        sliders.pack(fill='x', **pad)

        self.lin_var = tk.DoubleVar(value=0.15)
        self.ang_var = tk.DoubleVar(value=1.0)

        self._lin_lbl = tk.StringVar(value='Linear:  0.15 m/s')
        self._ang_lbl = tk.StringVar(value='Angular: 1.00 rad/s')

        tk.Label(sliders, textvariable=self._lin_lbl, width=20,
                 anchor='w').grid(row=0, column=0, padx=6)
        tk.Scale(sliders, from_=0.05, to=0.22, resolution=0.01,
                 orient='horizontal', variable=self.lin_var,
                 command=self._update_lin_lbl,
                 showvalue=False, length=140).grid(row=0, column=1, padx=4)

        tk.Label(sliders, textvariable=self._ang_lbl, width=20,
                 anchor='w').grid(row=1, column=0, padx=6)
        tk.Scale(sliders, from_=0.3, to=2.84, resolution=0.05,
                 orient='horizontal', variable=self.ang_var,
                 command=self._update_ang_lbl,
                 showvalue=False, length=140).grid(row=1, column=1, padx=4)

        # ── Status display ────────────────────────────────────────────────
        status = tk.LabelFrame(self.root, text='Status')
        status.pack(fill='x', **pad)

        self._vel_var = tk.StringVar(value='v: +0.00 m/s   ω: +0.00 rad/s')
        self._pose_var = tk.StringVar(value='x: 0.000   y: 0.000   yaw: 0.0°')
        self._map_var = tk.StringVar(value='Map: waiting...')

        for var in (self._vel_var, self._pose_var, self._map_var):
            tk.Label(status, textvariable=var, font=('Courier', 9),
                     anchor='w').pack(fill='x', padx=6, pady=1)

    def _make_hold_btn(self, parent, text, cfg, row, col,
                       lin, ang, pack=False):
        btn = tk.Button(parent, text=text, **cfg)
        btn.bind('<ButtonPress-1>',   lambda _: self.node.set_vel(lin(), ang()))
        btn.bind('<ButtonRelease-1>', lambda _: self.node.stop())
        if pack:
            btn.grid(row=row, column=col, padx=4, pady=2)
        else:
            btn.grid(row=row, column=col, padx=4, pady=2)

    def _make_stop_btn(self, parent, text, cfg, row, col):
        btn = tk.Button(parent, text=text, **cfg,
                        command=self.node.stop)
        btn.grid(row=row, column=col, padx=4, pady=2)

    # ── Keyboard bindings ─────────────────────────────────────────────────

    def _bind_keys(self):
        self.root.focus_set()
        self._held = set()

        bindings = {
            ('w', 'W', 'Up'):    (self._lin_val, lambda: 0.0),
            ('s', 'S', 'Down'):  (lambda: -self._lin_val(), lambda: 0.0),
            ('a', 'A', 'Left'):  (lambda: 0.0, self._ang_val),
            ('d', 'D', 'Right'): (lambda: 0.0, lambda: -self._ang_val()),
            ('q', 'Q'):          (lambda: 0.0, self._ang_val),
            ('e', 'E'):          (lambda: 0.0, lambda: -self._ang_val()),
        }

        for keys, (lin_fn, ang_fn) in bindings.items():
            for k in keys:
                # closure-capture via default args
                self.root.bind(f'<KeyPress-{k}>',
                               (lambda e, lf=lin_fn, af=ang_fn:
                                self._key_press(e, lf, af)))
                self.root.bind(f'<KeyRelease-{k}>',
                               lambda e: self._key_release(e))

        self.root.bind('<space>', lambda _: self.node.stop())

    def _key_press(self, event, lin_fn, ang_fn):
        key = event.keysym
        if key not in self._held:
            self._held.add(key)
            self.node.set_vel(lin_fn(), ang_fn())

    def _key_release(self, event):
        self._held.discard(event.keysym)
        if not self._held:
            self.node.stop()

    # ── Slider callbacks ──────────────────────────────────────────────────

    def _lin_val(self):
        return self.lin_var.get()

    def _ang_val(self):
        return self.ang_var.get()

    def _update_lin_lbl(self, val):
        self._lin_lbl.set(f'Linear:  {float(val):.2f} m/s')

    def _update_ang_lbl(self, val):
        self._ang_lbl.set(f'Angular: {float(val):.2f} rad/s')

    # ── Periodic display refresh ──────────────────────────────────────────

    def _refresh(self):
        lin, ang = self.node.get_vel()
        self._vel_var.set(f'v: {lin:+.2f} m/s   ω: {ang:+.2f} rad/s')

        x, y, yaw = self.node.get_pose()
        self._pose_var.set(
            f'x: {x:.3f}   y: {y:.3f}   yaw: {math.degrees(yaw):.1f}°')

        w, h, res = self.node.get_map_info()
        if w > 0:
            self._map_var.set(f'Map: {w}×{h} px | res: {res:.3f} m/px')

        self.root.after(200, self._refresh)

    # ── Close ─────────────────────────────────────────────────────────────

    def _on_close(self):
        self.node.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = MappingCtlNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    gui = MappingCtlGUI(node)
    gui.run()
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
