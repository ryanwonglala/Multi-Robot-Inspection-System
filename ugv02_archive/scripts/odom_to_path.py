#!/usr/bin/env python3
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class OdomToPath(Node):
    def __init__(self):
        super().__init__("odom_to_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/odom_path")
        self.declare_parameter("max_poses", 3000)
        self.declare_parameter("min_distance", 0.0)
        self.declare_parameter("publish_rate", 2.0)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.max_poses = int(self.get_parameter("max_poses").value)
        self.min_distance = float(self.get_parameter("min_distance").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.poses = deque(maxlen=self.max_poses)
        self.last_x = None
        self.last_y = None
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.create_timer(1.0 / max(0.1, publish_rate), self.publish_path)
        self.get_logger().info(f"Publishing {self.path_topic} from {self.odom_topic}")

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        if self.last_x is not None:
            dx = pose.x - self.last_x
            dy = pose.y - self.last_y
            if dx * dx + dy * dy < self.min_distance * self.min_distance:
                return

        self.last_x = pose.x
        self.last_y = pose.y

        stamped = PoseStamped()
        stamped.header = msg.header
        stamped.pose = msg.pose.pose
        self.poses.append(stamped)

        self.publish_path()

    def publish_path(self):
        if not self.poses:
            return
        path = Path()
        path.header = self.poses[-1].header
        path.poses = list(self.poses)
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
