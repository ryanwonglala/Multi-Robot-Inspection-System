#!/usr/bin/env python3
"""Manual Tkinter drive GUI for TurtleBot3 Burger + Gazebo Classic."""
import math
import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

LIN_MAX = 0.22   # TB3 Burger max linear  (m/s)
ANG_MAX = 2.84   # TB3 Burger max angular (rad/s)
HZ = 10
STEP_LIN = 0.02
STEP_ANG = 0.1


class CtlNode(Node):
    def __init__(self):
        super().__init__('manual_controller')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self.lin = 0.0
        self.ang = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.create_timer(1.0 / HZ, self._publish)

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)


class CtlGUI:
    def __init__(self, node: CtlNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('TB3 Burger Manual Controller')
        self.root.resizable(False, False)

        pad = {'padx': 6, 'pady': 4}

        # ── Speed display ──────────────────────────────────────────
        speed_frame = tk.LabelFrame(self.root, text='Speed', **pad)
        speed_frame.grid(row=0, column=0, columnspan=3, sticky='ew', **pad)

        tk.Label(speed_frame, text='Linear (m/s):').grid(row=0, column=0, sticky='w')
        self.lin_var = tk.StringVar(value='0.00')
        tk.Label(speed_frame, textvariable=self.lin_var, width=6,
                 relief='sunken').grid(row=0, column=1)

        tk.Label(speed_frame, text='Angular (rad/s):').grid(row=1, column=0, sticky='w')
        self.ang_var = tk.StringVar(value='0.00')
        tk.Label(speed_frame, textvariable=self.ang_var, width=6,
                 relief='sunken').grid(row=1, column=1)

        # ── Odometry display ───────────────────────────────────────
        odom_frame = tk.LabelFrame(self.root, text='Odometry', **pad)
        odom_frame.grid(row=1, column=0, columnspan=3, sticky='ew', **pad)

        for col, label in enumerate(('x (m)', 'y (m)', 'yaw (°)')):
            tk.Label(odom_frame, text=label).grid(row=0, column=col, **pad)
        self.x_var = tk.StringVar(value='0.00')
        self.y_var = tk.StringVar(value='0.00')
        self.yaw_var = tk.StringVar(value='0.0')
        for col, var in enumerate((self.x_var, self.y_var, self.yaw_var)):
            tk.Label(odom_frame, textvariable=var, width=7,
                     relief='sunken').grid(row=1, column=col, **pad)

        # ── D-pad buttons ──────────────────────────────────────────
        btn_frame = tk.LabelFrame(self.root, text='Controls  (WASD / ↑↓←→)', **pad)
        btn_frame.grid(row=2, column=0, columnspan=3, **pad)

        btn_cfg = {'width': 4, 'height': 2}
        tk.Button(btn_frame, text='↑\nFwd', command=self._fwd, **btn_cfg).grid(row=0, column=1)
        tk.Button(btn_frame, text='←\nLeft', command=self._left, **btn_cfg).grid(row=1, column=0)
        tk.Button(btn_frame, text='■\nStop', command=self._stop, **btn_cfg).grid(row=1, column=1)
        tk.Button(btn_frame, text='→\nRight', command=self._right, **btn_cfg).grid(row=1, column=2)
        tk.Button(btn_frame, text='↓\nBack', command=self._back, **btn_cfg).grid(row=2, column=1)

        # ── Keyboard bindings ──────────────────────────────────────
        for key in ('w', 'W', 'Up'):
            self.root.bind(f'<{key}>', lambda _: self._fwd())
        for key in ('s', 'S', 'Down'):
            self.root.bind(f'<{key}>', lambda _: self._back())
        for key in ('a', 'A', 'Left'):
            self.root.bind(f'<{key}>', lambda _: self._left())
        for key in ('d', 'D', 'Right'):
            self.root.bind(f'<{key}>', lambda _: self._right())
        self.root.bind('<space>', lambda _: self._stop())

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._update_display()

    def _clamp_lin(self, v):
        return max(-LIN_MAX, min(LIN_MAX, v))

    def _clamp_ang(self, v):
        return max(-ANG_MAX, min(ANG_MAX, v))

    def _fwd(self):
        self.node.lin = self._clamp_lin(self.node.lin + STEP_LIN)
        self.node.ang = 0.0

    def _back(self):
        self.node.lin = self._clamp_lin(self.node.lin - STEP_LIN)
        self.node.ang = 0.0

    def _left(self):
        self.node.ang = self._clamp_ang(self.node.ang + STEP_ANG)

    def _right(self):
        self.node.ang = self._clamp_ang(self.node.ang - STEP_ANG)

    def _stop(self):
        self.node.lin = 0.0
        self.node.ang = 0.0

    def _update_display(self):
        self.lin_var.set(f'{self.node.lin:+.2f}')
        self.ang_var.set(f'{self.node.ang:+.2f}')
        self.x_var.set(f'{self.node.x:.2f}')
        self.y_var.set(f'{self.node.y:.2f}')
        self.yaw_var.set(f'{math.degrees(self.node.yaw):.1f}')
        self.root.after(100, self._update_display)

    def _on_close(self):
        self._stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = CtlNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    gui = CtlGUI(node)
    gui.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
