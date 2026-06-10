#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class SetInitialPoseNode(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', -4.8)
        self.declare_parameter('y', -3.5)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', -1.5708)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('delay_sec', 5.0)
        self.declare_parameter('repeat_count', 10)
        self.declare_parameter('repeat_period_sec', 0.5)

        self._publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self._sent = 0
        self._repeat_count = int(self.get_parameter('repeat_count').value)
        delay = float(self.get_parameter('delay_sec').value)
        self._delay_timer = self.create_timer(delay, self._start_publishing)
        self.get_logger().info(f'Will publish initial pose after {delay:.1f}s')

    def _start_publishing(self):
        self._delay_timer.cancel()
        period = float(self.get_parameter('repeat_period_sec').value)
        self._timer = self.create_timer(period, self._publish_once)
        self._publish_once()

    def _publish_once(self):
        if self._sent >= self._repeat_count:
            self.get_logger().info('Initial pose publish complete')
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

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685389

        self._publisher.publish(msg)
        self._sent += 1
        self.get_logger().info(
            f"Published initial pose {self._sent}/{self._repeat_count}: "
            f"x={msg.pose.pose.position.x:.3f} y={msg.pose.pose.position.y:.3f} yaw={yaw:.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
