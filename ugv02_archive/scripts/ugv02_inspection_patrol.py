#!/usr/bin/env python3
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.exceptions import RCLError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


def min_valid(*values):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    return min(vals) if vals else None


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Ugv02InspectionPatrol(Node):
    def __init__(self):
        super().__init__("ugv02_inspection_patrol")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        self.declare_parameter("base_speed", 0.075)
        self.declare_parameter("creep_speed", 0.035)
        self.declare_parameter("reverse_speed", -0.070)
        self.declare_parameter("max_speed", 0.130)
        self.declare_parameter("turn_speed", 0.38)
        self.declare_parameter("max_turn", 0.65)

        self.declare_parameter("front_stop", 0.42)
        self.declare_parameter("front_slow", 0.82)
        self.declare_parameter("side_safe", 0.38)
        self.declare_parameter("depth_stop", 0.38)
        self.declare_parameter("depth_slow", 0.75)

        self.declare_parameter("front_angle_deg", 24.0)
        self.declare_parameter("lidar_max_range", 6.0)
        self.declare_parameter("backup_time", 0.85)
        self.declare_parameter("turn_time", 1.15)
        self.declare_parameter("stuck_time", 2.2)
        self.declare_parameter("stuck_distance", 0.025)

        self.declare_parameter("heading_hold_gain", 0.9)
        self.declare_parameter("side_gain", 0.42)
        self.declare_parameter("cmd_rate", 10.0)
        self.declare_parameter("publish_debug", True)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, "/inspection/status", 10)

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

        self.scan = {}
        self.scan_stamp = 0.0
        self.depth = {}
        self.depth_stamp = 0.0
        self.odom_stamp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.state = "FORWARD"
        self.state_until = 0.0
        self.turn_dir = 1.0
        self.target_yaw = None

        self.current_vx = 0.0
        self.current_wz = 0.0
        self.last_loop_time = time.monotonic()
        self.last_log_time = 0.0
        self.progress_x = 0.0
        self.progress_y = 0.0
        self.progress_time = time.monotonic()

        period = 1.0 / float(self.get_parameter("cmd_rate").value)
        self.timer = self.create_timer(period, self.loop)
        self.get_logger().info("UGV02 inspection patrol started: lidar + depth + odom -> cmd_vel")

    def scan_callback(self, msg):
        front_deg = float(self.get_parameter("front_angle_deg").value)
        self.scan = {
            "front": self.scan_sector(msg, -front_deg, front_deg),
            "wide_front": self.scan_sector(msg, -45.0, 45.0),
            "front_left": self.scan_sector(msg, 12.0, 58.0),
            "front_right": self.scan_sector(msg, -58.0, -12.0),
            "left": self.scan_sector(msg, 45.0, 115.0),
            "right": self.scan_sector(msg, -115.0, -45.0),
        }
        self.scan_stamp = time.monotonic()

    def scan_sector(self, msg, deg_min, deg_max):
        vals = []
        max_range = float(self.get_parameter("lidar_max_range").value)
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < max(msg.range_min, 0.08) or r > min(msg.range_max, max_range):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(normalize_angle(angle))
            if deg_min <= deg <= deg_max:
                vals.append(float(r))
        if not vals:
            return None
        return float(np.percentile(np.array(vals, dtype=np.float32), 18))

    def depth_callback(self, msg):
        try:
            depth_m = self.image_to_depth_m(msg)
        except Exception as exc:
            self.get_logger().warn(f"Depth image parse failed: {exc}")
            return

        if depth_m is None:
            return

        h, w = depth_m.shape
        mid = self.roi_distance(depth_m, w, h, 0.36, 0.64, 0.30, 0.56)
        left = self.roi_distance(depth_m, w, h, 0.08, 0.38, 0.34, 0.62)
        right = self.roi_distance(depth_m, w, h, 0.62, 0.92, 0.34, 0.62)

        low_roi = self.roi_values(depth_m, w, h, 0.30, 0.70, 0.56, 0.78)
        low = None
        floor_suspect = False
        if low_roi.size > 30:
            low_close_ratio = float(np.mean(low_roi < float(self.get_parameter("depth_stop").value)))
            low_p20 = float(np.percentile(low_roi, 20))
            # If almost the whole lower image is close, it is usually floor, not an obstacle.
            if 0.06 <= low_close_ratio <= 0.42:
                low = low_p20
            elif low_close_ratio > 0.42:
                floor_suspect = True

        self.depth = {
            "front": min_valid(mid, low),
            "left": left,
            "right": right,
            "mid": mid,
            "low": low,
            "floor_suspect": floor_suspect,
        }
        self.depth_stamp = time.monotonic()

    def image_to_depth_m(self, msg):
        enc = msg.encoding.lower()
        if enc in ("16uc1", "mono16", "z16"):
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape((msg.height, msg.step))
            usable = rows[:, : msg.width * 2]
            depth_u16 = usable.view(np.uint16).reshape((msg.height, msg.width))
            depth_m = depth_u16.astype(np.float32) * 0.001
            return depth_m
        if enc == "32fc1":
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape((msg.height, msg.step))
            usable = rows[:, : msg.width * 4]
            return usable.view(np.float32).reshape((msg.height, msg.width))
        return None

    def roi_values(self, depth_m, w, h, x1, x2, y1, y2):
        xs = int(w * x1)
        xe = int(w * x2)
        ys = int(h * y1)
        ye = int(h * y2)
        roi = depth_m[ys:ye, xs:xe]
        vals = roi[np.isfinite(roi)]
        vals = vals[(vals > 0.15) & (vals < 4.0)]
        return vals

    def roi_distance(self, depth_m, w, h, x1, x2, y1, y2):
        vals = self.roi_values(depth_m, w, h, x1, x2, y1, y2)
        if vals.size < 30:
            return None
        return float(np.percentile(vals, 18))

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quat(pose.orientation)
        self.odom_stamp = time.monotonic()

    def loop(self):
        now = time.monotonic()
        dt = max(0.02, now - self.last_loop_time)
        self.last_loop_time = now

        scan_age = now - self.scan_stamp if self.scan_stamp else 999.0
        odom_age = now - self.odom_stamp if self.odom_stamp else 999.0
        depth_age = now - self.depth_stamp if self.depth_stamp else 999.0

        if scan_age > 1.5:
            self.publish_cmd(0.0, 0.0, dt)
            self.log_status(now, "NO_SCAN", None, None, None, depth_age, odom_age)
            return

        front = min_valid(self.scan.get("front"), self.scan.get("wide_front"), self.depth.get("front"))
        left = min_valid(self.scan.get("left"), self.scan.get("front_left"), self.depth.get("left"))
        right = min_valid(self.scan.get("right"), self.scan.get("front_right"), self.depth.get("right"))

        if front is None:
            self.publish_cmd(0.0, 0.0, dt)
            self.log_status(now, "NO_FRONT_DISTANCE", front, left, right, depth_age, odom_age)
            return

        front_stop = float(self.get_parameter("front_stop").value)
        front_slow = float(self.get_parameter("front_slow").value)
        side_safe = float(self.get_parameter("side_safe").value)

        if self.is_stuck(now):
            self.start_backup(now, left, right, reason="STUCK")

        if self.state not in ("BACKUP", "TURN") and front < front_stop:
            self.start_backup(now, left, right, reason="OBSTACLE")

        if self.state == "BACKUP":
            if now < self.state_until:
                self.publish_cmd(float(self.get_parameter("reverse_speed").value), 0.0, dt)
                self.log_status(now, self.state, front, left, right, depth_age, odom_age)
                return
            self.state = "TURN"
            self.state_until = now + float(self.get_parameter("turn_time").value)
            self.target_yaw = None

        if self.state == "TURN":
            if now < self.state_until or front < front_slow:
                self.publish_cmd(0.0, self.turn_dir * float(self.get_parameter("turn_speed").value), dt)
                self.log_status(now, self.state, front, left, right, depth_age, odom_age)
                return
            self.state = "FORWARD"
            self.target_yaw = self.yaw
            self.reset_progress(now)

        vx, wz = self.forward_policy(front, left, right, side_safe, front_slow)
        self.publish_cmd(vx, wz, dt)
        self.log_status(now, self.state, front, left, right, depth_age, odom_age)

    def forward_policy(self, front, left, right, side_safe, front_slow):
        base_speed = float(self.get_parameter("base_speed").value)
        creep_speed = float(self.get_parameter("creep_speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        max_turn = float(self.get_parameter("max_turn").value)
        side_gain = float(self.get_parameter("side_gain").value)
        heading_gain = float(self.get_parameter("heading_hold_gain").value)

        if self.target_yaw is None:
            self.target_yaw = self.yaw

        left_v = left if left is not None else 2.0
        right_v = right if right is not None else 2.0
        side_balance = clamp(left_v - right_v, -1.0, 1.0)
        wz = side_gain * side_balance

        if front < front_slow:
            vx = creep_speed
            self.turn_dir = self.choose_turn_dir(left, right)
            wz += self.turn_dir * 0.22
            self.target_yaw = None
        else:
            clear_bonus = clamp((front - front_slow) * 0.04, 0.0, 0.05)
            vx = min(max_speed, base_speed + clear_bonus)
            if min(left_v, right_v) > side_safe + 0.25:
                yaw_error = normalize_angle(self.target_yaw - self.yaw)
                wz += clamp(heading_gain * yaw_error, -0.16, 0.16)
            else:
                self.target_yaw = self.yaw

        return clamp(vx, -0.12, max_speed), clamp(wz, -max_turn, max_turn)

    def choose_turn_dir(self, left, right):
        left_v = left if left is not None else 0.0
        right_v = right if right is not None else 0.0
        if abs(left_v - right_v) < 0.08:
            return self.turn_dir
        return 1.0 if left_v > right_v else -1.0

    def start_backup(self, now, left, right, reason):
        self.state = "BACKUP"
        self.state_until = now + float(self.get_parameter("backup_time").value)
        self.turn_dir = self.choose_turn_dir(left, right)
        self.target_yaw = None
        self.reset_progress(now)
        self.get_logger().warn(f"{reason}: backup then turn {'left' if self.turn_dir > 0 else 'right'}")

    def is_stuck(self, now):
        if self.current_vx < 0.03:
            self.reset_progress(now)
            return False
        if now - self.progress_time < float(self.get_parameter("stuck_time").value):
            return False
        moved = math.hypot(self.x - self.progress_x, self.y - self.progress_y)
        if moved < float(self.get_parameter("stuck_distance").value):
            return True
        self.reset_progress(now)
        return False

    def reset_progress(self, now):
        self.progress_x = self.x
        self.progress_y = self.y
        self.progress_time = now

    def publish_cmd(self, target_vx, target_wz, dt):
        max_accel = 0.16
        max_ang_accel = 1.20
        self.current_vx += clamp(target_vx - self.current_vx, -max_accel * dt, max_accel * dt)
        self.current_wz += clamp(target_wz - self.current_wz, -max_ang_accel * dt, max_ang_accel * dt)

        cmd = Twist()
        cmd.linear.x = float(self.current_vx)
        cmd.angular.z = float(self.current_wz)
        try:
            self.cmd_pub.publish(cmd)
        except RCLError:
            pass

    def stop(self):
        self.current_vx = 0.0
        self.current_wz = 0.0
        cmd = Twist()
        try:
            for _ in range(3):
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)
        except RCLError:
            pass

    def log_status(self, now, state, front, left, right, depth_age, odom_age):
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        status = {
            "state": state,
            "front": None if front is None else round(front, 2),
            "left": None if left is None else round(left, 2),
            "right": None if right is None else round(right, 2),
            "depth_front": None if self.depth.get("front") is None else round(self.depth.get("front"), 2),
            "floor_suspect": bool(self.depth.get("floor_suspect", False)),
            "odom_age": round(odom_age, 2),
            "depth_age": round(depth_age, 2),
            "vx": round(self.current_vx, 3),
            "wz": round(self.current_wz, 3),
        }
        if bool(self.get_parameter("publish_debug").value):
            msg = String()
            msg.data = json.dumps(status, separators=(",", ":"))
            self.status_pub.publish(msg)
        self.get_logger().info(msg.data if bool(self.get_parameter("publish_debug").value) else str(status))


def main():
    rclpy.init()
    node = Ugv02InspectionPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
