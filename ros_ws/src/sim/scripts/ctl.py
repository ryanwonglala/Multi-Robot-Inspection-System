#!/usr/bin/env python3
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


HZ = 10
LIN_MAX = 0.7
ANG_MAX = 1.8
LIN_DEFAULT = 0.2
ANG_DEFAULT = 0.6


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class CtlNode(Node):
    def __init__(self):
        super().__init__('tb4_ctl')
        self.declare_parameter('cmd', '/cmd_vel')
        self.declare_parameter('odom', '/odom')
        cmd = self.get_parameter('cmd').value
        odom = self.get_parameter('odom').value
        self.pub = self.create_publisher(TwistStamped, cmd, 10)
        self.sub = self.create_subscription(Odometry, odom, self._odom_cb, 10)
        self.odom = None
        self.cmd_topic = cmd
        self.odom_topic = odom

    def _odom_cb(self, msg):
        self.odom = msg

    def send(self, lin, ang):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.pub.publish(msg)

    def stop(self):
        self.send(0.0, 0.0)


class CtlGui:
    def __init__(self, node):
        self.node = node
        self.lin_dir = 0.0
        self.ang_dir = 0.0
        self.active_keys = set()
        self.running = True

        self.root = tk.Tk()
        self.root.title('TB4 ctl')
        self.root.minsize(360, 300)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.lin = tk.DoubleVar(value=LIN_DEFAULT)
        self.ang = tk.DoubleVar(value=ANG_DEFAULT)
        self.state = tk.StringVar(value='idle')
        self.odom = tk.StringVar(value='odom: --')
        self.topics = tk.StringVar(value=f'cmd: {node.cmd_topic}   odom: {node.odom_topic}')

        self._build()
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        self.root.after(int(1000 / HZ), self.tick)
        self.root.after(100, self.update_odom)

    def _build(self):
        pad = {'padx': 8, 'pady': 6}
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill='both', expand=True)

        grid = ttk.LabelFrame(main, text='Motion')
        grid.pack(fill='x', **pad)

        btns = [
            ('W', 0, 1, lambda: self.set_motion(1, 0)),
            ('A', 1, 0, lambda: self.set_motion(0, 1)),
            ('STOP', 1, 1, self.stop),
            ('D', 1, 2, lambda: self.set_motion(0, -1)),
            ('S', 2, 1, lambda: self.set_motion(-1, 0)),
        ]
        for text, row, col, cb in btns:
            b = ttk.Button(grid, text=text)
            b.grid(row=row, column=col, sticky='nsew', padx=4, pady=4)
            if text == 'STOP':
                b.configure(command=cb)
            else:
                b.bind('<ButtonPress-1>', lambda _e, fn=cb: fn())
                b.bind('<ButtonRelease-1>', lambda _e: self.stop())
        for i in range(3):
            grid.columnconfigure(i, weight=1)

        speed = ttk.LabelFrame(main, text='Speed')
        speed.pack(fill='x', **pad)
        ttk.Label(speed, text='linear').grid(row=0, column=0, sticky='w')
        ttk.Scale(speed, from_=0.0, to=LIN_MAX, variable=self.lin).grid(row=0, column=1, sticky='ew')
        ttk.Label(speed, textvariable=self.lin).grid(row=0, column=2, sticky='e')
        ttk.Label(speed, text='angular').grid(row=1, column=0, sticky='w')
        ttk.Scale(speed, from_=0.0, to=ANG_MAX, variable=self.ang).grid(row=1, column=1, sticky='ew')
        ttk.Label(speed, textvariable=self.ang).grid(row=1, column=2, sticky='e')
        speed.columnconfigure(1, weight=1)

        status = ttk.LabelFrame(main, text='Status')
        status.pack(fill='x', **pad)
        ttk.Label(status, textvariable=self.state).pack(anchor='w')
        ttk.Label(status, textvariable=self.odom).pack(anchor='w')
        ttk.Label(status, textvariable=self.topics).pack(anchor='w')

    def set_motion(self, lin_dir, ang_dir):
        self.lin_dir = float(lin_dir)
        self.ang_dir = float(ang_dir)
        self.publish()

    def publish(self):
        lin = self.lin_dir * self.lin.get()
        ang = self.ang_dir * self.ang.get()
        self.node.send(lin, ang)
        self.state.set(f'cmd: lin {lin:.2f} m/s   ang {ang:.2f} rad/s')

    def stop(self):
        self.lin_dir = 0.0
        self.ang_dir = 0.0
        self.node.stop()
        self.state.set('cmd: stopped')

    def tick(self):
        if self.running:
            if self.lin_dir or self.ang_dir:
                self.publish()
            self.root.after(int(1000 / HZ), self.tick)

    def update_odom(self):
        odom = self.node.odom
        if odom is not None:
            p = odom.pose.pose.position
            yaw = yaw_from_quat(odom.pose.pose.orientation)
            self.odom.set(f'odom: x {p.x:.2f}   y {p.y:.2f}   yaw {yaw:.2f}')
        if self.running:
            self.root.after(100, self.update_odom)

    def key_press(self, event):
        key = event.keysym.lower()
        self.active_keys.add(key)
        if key == 'w':
            self.set_motion(1, 0)
        elif key == 's':
            self.set_motion(-1, 0)
        elif key == 'a':
            self.set_motion(0, 1)
        elif key == 'd':
            self.set_motion(0, -1)
        elif key == 'space':
            self.stop()

    def key_release(self, event):
        key = event.keysym.lower()
        self.active_keys.discard(key)
        if key in {'w', 'a', 's', 'd'}:
            self.stop()

    def close(self):
        self.running = False
        self.node.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def spin_node(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = CtlNode()
    stop_event = threading.Event()
    thread = threading.Thread(target=spin_node, args=(node, stop_event), daemon=True)
    thread.start()
    gui = CtlGui(node)
    try:
        gui.run()
    finally:
        stop_event.set()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
