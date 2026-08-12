#!/usr/bin/env python3
import json
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class UGV02Node(Node):
    def __init__(self):
        super().__init__("wave_rover_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_linear", 0.45)
        self.declare_parameter("max_angular", 1.20)
        self.declare_parameter("deadman_timeout", 0.60)
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("track_width", 0.26)
        self.declare_parameter("meters_per_tick", 0.001)

        # 用来修正小车真实前进方向和 ROS odom/base_link 方向不一致的问题。
        # 你现在需要先用 90.0。如果方向反了，再改成 -90.0 或 180.0。
        self.declare_parameter("odom_yaw_offset_deg", 90.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.meters_per_tick = float(self.get_parameter("meters_per_tick").value)
        self.odom_yaw_offset = math.radians(
            float(self.get_parameter("odom_yaw_offset_deg").value)
        )

        self.running = True
        self.write_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.odom_lock = threading.Lock()

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = 0.0
        self.last_cmd_log = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_l = None
        self.last_odom_r = None
        self.last_odom_time = None
        self.last_feedback_time = 0.0
        self.last_feedback_warning = 0.0

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.05,
            write_timeout=0.20,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 20)

        self.reader_thread = threading.Thread(
            target=self.read_loop,
            name="ugv02-serial-reader",
            daemon=True,
        )
        self.reader_thread.start()

        command_period = 1.0 / float(self.get_parameter("command_rate").value)
        self.command_timer = self.create_timer(command_period, self.command_loop)
        self.feedback_timer = self.create_timer(1.0, self.feedback_watchdog)

        time.sleep(1.0)
        self.enable_feedback()
        self.send_motion(0.0, 0.0)

        self.get_logger().info(f"UGV02 driver with odom started")
        self.get_logger().info(f"Subscribed to /cmd_vel")
        self.get_logger().info(f"Publishing /odom and TF odom -> base_link")
        self.get_logger().info(f"Serial port: {self.port}, baud: {self.baud}")
        self.get_logger().info(
            f"Odometry yaw offset: {math.degrees(self.odom_yaw_offset):.1f} deg"
        )

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def send_json(self, payload):
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        with self.write_lock:
            self.ser.write(line.encode("utf-8"))
            self.ser.flush()

    def enable_feedback(self):
        self.send_json({"T": 131, "cmd": 1})

    def disable_feedback(self):
        self.send_json({"T": 131, "cmd": 0})

    def send_motion(self, linear, angular):
        linear = self.clamp(float(linear), self.max_linear)
        angular = self.clamp(float(angular), self.max_angular)
        self.send_json({"T": 13, "X": linear, "Z": angular})

    def cmd_vel_callback(self, msg):
        with self.command_lock:
            self.target_linear = self.clamp(msg.linear.x, self.max_linear)
            self.target_angular = self.clamp(msg.angular.z, self.max_angular)
            self.last_cmd_time = time.monotonic()

    def command_loop(self):
        now = time.monotonic()
        with self.command_lock:
            if now - self.last_cmd_time > self.deadman_timeout:
                linear = 0.0
                angular = 0.0
            else:
                linear = self.target_linear
                angular = self.target_angular

        try:
            self.send_motion(linear, angular)
        except Exception as exc:
            if now - self.last_cmd_log > 1.0:
                self.get_logger().warn(f"Command send failed: {exc}")
                self.last_cmd_log = now

    def read_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", "ignore").strip()
                if not line:
                    continue

                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue

                self.last_feedback_time = time.monotonic()

                if "odl" in data and "odr" in data:
                    self.handle_odom(data)

            except Exception as exc:
                now = time.monotonic()
                if now - self.last_feedback_warning > 1.0:
                    self.get_logger().warn(f"Feedback parse failed: {exc}")
                    self.last_feedback_warning = now
                time.sleep(0.02)

    def handle_odom(self, data):
        stamp = self.get_clock().now()
        now = time.monotonic()

        odom_l = int(data["odl"])
        odom_r = int(data["odr"])

        if self.last_odom_l is None:
            self.last_odom_l = odom_l
            self.last_odom_r = odom_r
            self.last_odom_time = now
            return

        dl_ticks = odom_l - self.last_odom_l
        dr_ticks = odom_r - self.last_odom_r
        dt = now - self.last_odom_time

        self.last_odom_l = odom_l
        self.last_odom_r = odom_r
        self.last_odom_time = now

        if dt <= 0.0:
            return

        if abs(dl_ticks) > 10000 or abs(dr_ticks) > 10000:
            return

        dl = dl_ticks * self.meters_per_tick
        dr = dr_ticks * self.meters_per_tick

        ds = (dl + dr) * 0.5
        dyaw = (dr - dl) / self.track_width

        with self.odom_lock:
            mid_yaw = self.yaw + dyaw * 0.5
            self.x += ds * math.cos(mid_yaw)
            self.y += ds * math.sin(mid_yaw)
            self.yaw = math.atan2(
                math.sin(self.yaw + dyaw),
                math.cos(self.yaw + dyaw),
            )

            self.publish_odom(stamp, ds / dt, dyaw / dt)

    def publish_odom(self, stamp, linear, angular):
        yaw_for_frames = math.atan2(
            math.sin(self.yaw + self.odom_yaw_offset),
            math.cos(self.yaw + self.odom_yaw_offset),
        )

        qz = math.sin(yaw_for_frames * 0.5)
        qw = math.cos(yaw_for_frames * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        odom.twist.twist.linear.x = float(linear)
        odom.twist.twist.angular.z = float(angular)

        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(transform)

    def feedback_watchdog(self):
        if self.last_feedback_time <= 0.0:
            return

        age = time.monotonic() - self.last_feedback_time
        if age > 2.0:
            self.get_logger().warn(f"No feedback from lower controller for {age:.1f}s")
            try:
                self.enable_feedback()
            except Exception:
                pass

    def stop_robot(self):
        try:
            self.send_motion(0.0, 0.0)
            time.sleep(0.05)
            self.send_motion(0.0, 0.0)
        except Exception:
            pass

    def shutdown(self):
        self.running = False
        self.stop_robot()
        try:
            self.disable_feedback()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = UGV02Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
