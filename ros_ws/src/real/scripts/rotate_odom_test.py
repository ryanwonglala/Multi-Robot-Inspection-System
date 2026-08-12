#!/usr/bin/env python3
"""Safely rotate one ROS 2 base by an odometry-measured angle."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_message(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RotationMonitor(Node):
    def __init__(self, odom_topic, cmd_topic, battery_topic):
        super().__init__('rotate_odom_test')
        self.cmd_topic = cmd_topic
        self.publisher = None
        # BEST_EFFORT matches both the production UGV sensor QoS and the
        # reliable simplified/TB3 odometry publishers.
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, battery_topic, self._on_battery, 10)
        self.last_yaw = None
        self.accumulated = 0.0
        self.last_odom_at = None
        self.last_progress_at = None
        self.last_progress_angle = 0.0
        self.battery_at = None
        self.battery_v = None
        self.battery_present = False

    def _on_battery(self, message):
        self.battery_at = time.monotonic()
        self.battery_v = float(message.voltage)
        self.battery_present = bool(message.present)

    def _on_odom(self, message):
        now = time.monotonic()
        yaw = yaw_from_message(message)
        if self.last_yaw is None:
            self.last_yaw = yaw
            self.last_progress_at = now
            self.last_progress_angle = self.accumulated
        else:
            self.accumulated += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
            if abs(self.accumulated - self.last_progress_angle) >= math.radians(1):
                self.last_progress_at = now
                self.last_progress_angle = self.accumulated
        self.last_odom_at = now

    def command(self, angular):
        if self.publisher is None:
            return
        message = Twist()
        message.angular.z = angular
        self.publisher.publish(message)

    def stop(self):
        for _ in range(10):
            self.command(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def enable_publisher(self):
        existing = self.get_publishers_info_by_topic(self.cmd_topic)
        if existing:
            names = ','.join(sorted({item.node_name for item in existing}))
            raise RuntimeError(f'existing_cmd_vel_publishers:{names}')
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', required=True)
    parser.add_argument('--cmd-topic', required=True)
    parser.add_argument('--degrees', type=float, default=90.0)
    parser.add_argument('--max-speed', type=float, default=0.35)
    parser.add_argument('--min-speed', type=float, default=0.10)
    parser.add_argument('--tolerance-deg', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--battery-topic', default='/battery_state')
    parser.add_argument('--min-voltage', type=float, default=10.5)
    args = parser.parse_args()

    rclpy.init()
    node = RotationMonitor(args.odom_topic, args.cmd_topic, args.battery_topic)
    target = math.radians(args.degrees)
    direction = 1.0 if target >= 0.0 else -1.0
    tolerance = math.radians(args.tolerance_deg)
    started = time.monotonic()
    moving_started = None
    result = 3

    try:
        while rclpy.ok() and (node.last_yaw is None or node.battery_at is None):
            if time.monotonic() - started > 5.0:
                print('ABORT: no odometry/battery received within 5 s', file=sys.stderr)
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.battery_present or node.battery_v < args.min_voltage:
            print(f'ABORT: battery_not_safe:{node.battery_v}', file=sys.stderr)
            return 2
        try:
            node.enable_publisher()
        except RuntimeError as error:
            print(f'ABORT: {error}', file=sys.stderr)
            return 2

        initial_yaw = node.last_yaw
        moving_started = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
            remaining = target - node.accumulated

            if direction * remaining <= tolerance:
                result = 0
                break
            if now - moving_started > args.timeout:
                print('ABORT: rotation timeout', file=sys.stderr)
                result = 2
                break
            if node.last_odom_at is None or now - node.last_odom_at > 0.6:
                print('ABORT: odometry stream lost', file=sys.stderr)
                result = 2
                break
            if (node.battery_at is None or now - node.battery_at > 1.0 or
                    not node.battery_present or
                    node.battery_v < args.min_voltage):
                print('ABORT: battery feedback unsafe', file=sys.stderr)
                result = 2
                break
            if (now - moving_started > 1.5 and
                    abs(node.accumulated) < math.radians(3)):
                print('ABORT: no odometry progress after motion command',
                      file=sys.stderr)
                result = 2
                break
            if (node.last_progress_at is not None and
                    now - node.last_progress_at > 1.5):
                print('ABORT: odometry stopped progressing', file=sys.stderr)
                result = 2
                break

            speed = min(args.max_speed, max(
                args.min_speed, 1.2 * abs(remaining)))
            node.command(direction * speed)
            time.sleep(0.03)

        node.stop()
        settle_until = time.monotonic() + 1.0
        while time.monotonic() < settle_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(
            f'initial_yaw_deg={math.degrees(initial_yaw):.2f} '
            f'rotated_deg={math.degrees(node.accumulated):.2f} '
            f'final_yaw_deg={math.degrees(node.last_yaw):.2f} '
            f'target_deg={args.degrees:.2f}')
        return result
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
