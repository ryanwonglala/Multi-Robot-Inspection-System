#!/usr/bin/env python3
"""Publishes /initialpose once AMCL is ready, then shuts down.

Ported from task_layer_v020 and adapted for TB3 Burger:
  - Default pose: x=-4.8, y=-3.825, yaw=-1.5708 (charging_station center)
  - Covariance: diagonal [0.25, 0.25, 0.068] (standard AMCL values)
  - Publishes repeat_count times at repeat_period_sec intervals, then exits
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class SetInitialPoseNode(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0325)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('delay_sec', 0.0)
        self.declare_parameter('repeat_count', 10)
        self.declare_parameter('repeat_period_sec', 0.5)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._sent = 0
        self._repeat_count = int(self.get_parameter('repeat_count').value)

        delay = float(self.get_parameter('delay_sec').value)
        if delay > 0.0:
            self.get_logger().info(f'Initial pose will publish after {delay:.1f} s')
            self._delay_timer = self.create_timer(delay, self._start)
        else:
            self._start()

    def _start(self):
        if hasattr(self, '_delay_timer'):
            self._delay_timer.cancel()
        period = float(self.get_parameter('repeat_period_sec').value)
        self._timer = self.create_timer(period, self._publish_once)
        self._publish_once()

    def _publish_once(self):
        if self._sent >= self._repeat_count:
            self._timer.cancel()
            self.get_logger().info('Initial pose publishing complete — node exiting')
            rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(self.get_parameter('x').value)
        msg.pose.pose.position.y = float(self.get_parameter('y').value)
        msg.pose.pose.position.z = float(self.get_parameter('z').value)

        yaw = float(self.get_parameter('yaw').value)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        # Standard AMCL diagonal covariance: x, y, z, roll, pitch, yaw
        msg.pose.covariance[0] = 0.25     # x variance
        msg.pose.covariance[7] = 0.25     # y variance
        msg.pose.covariance[35] = 0.068   # yaw variance (~±15°)

        self._pub.publish(msg)
        self._sent += 1
        self.get_logger().info(
            f'[{self._sent}/{self._repeat_count}] initialpose → '
            f'x={msg.pose.pose.position.x:.3f} '
            f'y={msg.pose.pose.position.y:.3f} '
            f'yaw={math.degrees(yaw):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPoseNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if node.handle:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
