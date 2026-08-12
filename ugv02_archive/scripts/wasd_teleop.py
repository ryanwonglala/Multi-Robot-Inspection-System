#!/usr/bin/env python3
import select
import math
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


HELP = """
UGV02 WASD control
  W: forward       S: backward
  A: rotate left   D: rotate right
  Space/X: stop    Q: quit
  J/L: trim left/right angular correction
  K: reset trim
  [: slow left side if robot drifts right
  ]: slow right side if robot drifts left
  \\: reset left/right speed rates
  P: save current speed rates in UGV02

Hold W/A/S/D to move. Release the key to stop.
Arrow keys also work.
"""


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WasdTeleop(Node):
    def __init__(self):
        super().__init__("wasd_teleop")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("straight_bias_z", 0.0)
        self.declare_parameter("heading_hold", True)
        self.declare_parameter("heading_kp", 1.4)
        self.declare_parameter("heading_deadband", 0.01)
        self.declare_parameter("heading_max_correction", 0.28)
        self.declare_parameter("heading_correction_sign", 1.0)
        self.declare_parameter("yaw_rate_hold", True)
        self.declare_parameter("yaw_rate_kp", 0.9)
        self.declare_parameter("yaw_rate_deadband", 0.015)
        self.declare_parameter("yaw_rate_max_correction", 0.22)
        self.declare_parameter("trim_step", 0.01)
        self.declare_parameter("speed_rate_step", 0.03)
        self.declare_parameter("left_speed_rate", 1.0)
        self.declare_parameter("right_speed_rate", 1.0)
        self.declare_parameter("deadman_timeout", 1.5)
        self.declare_parameter("publish_rate", 20.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.straight_bias_z = float(self.get_parameter("straight_bias_z").value)
        self.heading_hold = bool(self.get_parameter("heading_hold").value)
        self.heading_kp = float(self.get_parameter("heading_kp").value)
        self.heading_deadband = float(self.get_parameter("heading_deadband").value)
        self.heading_max_correction = float(
            self.get_parameter("heading_max_correction").value
        )
        self.heading_correction_sign = float(
            self.get_parameter("heading_correction_sign").value
        )
        self.yaw_rate_hold = bool(self.get_parameter("yaw_rate_hold").value)
        self.yaw_rate_kp = float(self.get_parameter("yaw_rate_kp").value)
        self.yaw_rate_deadband = float(self.get_parameter("yaw_rate_deadband").value)
        self.yaw_rate_max_correction = float(
            self.get_parameter("yaw_rate_max_correction").value
        )
        self.trim_step = float(self.get_parameter("trim_step").value)
        self.speed_rate_step = float(self.get_parameter("speed_rate_step").value)
        self.left_speed_rate = float(self.get_parameter("left_speed_rate").value)
        self.right_speed_rate = float(self.get_parameter("right_speed_rate").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.speed_rate_pub = self.create_publisher(String, "/ugv02/speed_rate", 10)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, sensor_qos)
        self.create_subscription(Imu, "/imu", self.imu_callback, sensor_qos)

        self.linear = 0.0
        self.angular = 0.0
        self.mode = "stop"
        self.straight_direction = 0
        self.current_yaw = None
        self.imu_yaw = None
        self.imu_yaw_rate = 0.0
        self.last_imu_time = 0.0
        self.target_yaw = None
        self.last_odom_time = 0.0
        self.last_key_time = 0.0
        self.quit_requested = False
        self.create_timer(1.0 / publish_rate, self.publish_command)

    def odom_callback(self, msg):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_time = time.monotonic()

    def imu_callback(self, msg):
        if msg.orientation_covariance[0] == -1.0:
            self.imu_yaw = None
        else:
            self.imu_yaw = yaw_from_quaternion(msg.orientation)
        self.imu_yaw_rate = float(msg.angular_velocity.z)
        self.last_imu_time = time.monotonic()

    def handle_key(self, key):
        if key == "\x1b":
            key = self.read_escape_sequence()

        key = key.lower()
        self.last_key_time = time.monotonic()

        if key in ("w", "up"):
            self.start_straight(1)
        elif key in ("s", "down"):
            self.start_straight(-1)
        elif key in ("a", "left"):
            self.start_rotate(1)
        elif key in ("d", "right"):
            self.start_rotate(-1)
        elif key == "j":
            self.straight_bias_z += self.trim_step
            self.refresh_straight_fallback()
        elif key == "l":
            self.straight_bias_z -= self.trim_step
            self.refresh_straight_fallback()
        elif key == "k":
            self.straight_bias_z = 0.0
            self.refresh_straight_fallback()
        elif key in (" ", "x"):
            self.stop()
        elif key in ("q", "\x03"):
            self.stop()
            self.quit_requested = True
        elif key == "[":
            self.left_speed_rate = clamp(
                self.left_speed_rate - self.speed_rate_step, 0.5, 1.0
            )
            self.publish_speed_rate()
        elif key == "]":
            self.right_speed_rate = clamp(
                self.right_speed_rate - self.speed_rate_step, 0.5, 1.0
            )
            self.publish_speed_rate()
        elif key == "\\":
            self.left_speed_rate = 1.0
            self.right_speed_rate = 1.0
            self.publish_speed_rate()
        elif key == "p":
            self.publish_speed_rate(save=True)

        print(
            f"\rcommand: vx={self.linear:+.2f} m/s, "
            f"wz={self.commanded_angular():+.2f} rad/s, "
            f"mode={self.mode}, trim={self.straight_bias_z:+.3f}, "
            f"rate L/R={self.left_speed_rate:.2f}/{self.right_speed_rate:.2f}      ",
            end="",
            flush=True,
        )

    def start_straight(self, direction):
        if self.mode != "straight" or self.straight_direction != direction:
            self.target_yaw = self.heading_yaw()
        self.mode = "straight"
        self.straight_direction = direction
        self.linear = direction * self.linear_speed
        self.angular = self.directional_trim()

    def start_rotate(self, direction):
        self.mode = "rotate"
        self.straight_direction = 0
        self.target_yaw = None
        self.linear = 0.0
        self.angular = direction * self.angular_speed

    def read_escape_sequence(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.02)
        if not readable:
            return "\x1b"
        second = sys.stdin.read(1)
        readable, _, _ = select.select([sys.stdin], [], [], 0.02)
        if not readable:
            return "\x1b" + second
        third = sys.stdin.read(1)
        if second == "[" and third == "A":
            return "up"
        if second == "[" and third == "B":
            return "down"
        if second == "[" and third == "C":
            return "right"
        if second == "[" and third == "D":
            return "left"
        return "\x1b" + second + third

    def publish_command(self):
        if (
            self.linear != 0.0 or self.angular != 0.0
        ) and time.monotonic() - self.last_key_time > self.deadman_timeout:
            self.stop()

        message = Twist()
        message.linear.x = float(self.linear)
        message.angular.z = float(self.commanded_angular())
        self.publisher.publish(message)

    def refresh_straight_fallback(self):
        if self.mode != "straight":
            return
        self.angular = self.directional_trim()

    def directional_trim(self):
        if self.straight_direction < 0:
            return -self.straight_bias_z
        return self.straight_bias_z

    def commanded_angular(self):
        current_heading = self.heading_yaw()
        rate_correction = 0.0
        if (
            self.yaw_rate_hold
            and self.mode == "straight"
            and time.monotonic() - self.last_imu_time < 1.0
        ):
            yaw_rate = self.imu_yaw_rate
            if abs(yaw_rate) >= self.yaw_rate_deadband:
                rate_correction = -self.yaw_rate_kp * yaw_rate
                rate_correction = clamp(
                    rate_correction,
                    -self.yaw_rate_max_correction,
                    self.yaw_rate_max_correction,
                )

        heading_correction = 0.0
        if (
            self.heading_hold
            and self.mode == "straight"
            and self.target_yaw is None
            and current_heading is not None
        ):
            self.target_yaw = current_heading

        if (
            self.heading_hold
            and self.mode == "straight"
            and self.target_yaw is not None
            and current_heading is not None
        ):
            error = wrap_pi(self.target_yaw - current_heading)
            if abs(error) < self.heading_deadband:
                heading_correction = 0.0
            else:
                heading_correction = self.heading_correction_sign * self.heading_kp * error
            heading_correction = clamp(
                heading_correction,
                -self.heading_max_correction,
                self.heading_max_correction,
            )
        if self.mode == "straight":
            return self.directional_trim() + heading_correction + rate_correction
        return self.angular

    def heading_yaw(self):
        now = time.monotonic()
        if self.imu_yaw is not None and now - self.last_imu_time < 1.0:
            return self.imu_yaw
        if self.current_yaw is not None and now - self.last_odom_time < 1.0:
            return self.current_yaw
        return None

    def publish_speed_rate(self, save=False):
        message = String()
        message.data = (
            f'{{"L":{self.left_speed_rate:.3f},'
            f'"R":{self.right_speed_rate:.3f},'
            f'"save":{str(bool(save)).lower()}}}'
        )
        for _ in range(3):
            self.speed_rate_pub.publish(message)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        self.mode = "stop"
        self.straight_direction = 0
        self.target_yaw = None

        message = Twist()
        for _ in range(3):
            self.publisher.publish(message)


def main():
    if not sys.stdin.isatty():
        print("Run this program in an interactive terminal.")
        return 1

    rclpy.init()
    node = WasdTeleop()
    terminal_settings = termios.tcgetattr(sys.stdin)

    print(HELP)
    print(
        f"linear={node.linear_speed:.2f} m/s, "
        f"angular={node.angular_speed:.2f} rad/s, "
        f"straight_bias_z={node.straight_bias_z:+.3f}, "
        f"heading_hold={node.heading_hold}"
    )
    print(
        f"heading_kp={node.heading_kp:.2f}, "
        f"max_correction={node.heading_max_correction:.2f} rad/s, "
        f"correction_sign={node.heading_correction_sign:+.0f}"
    )
    print(
        f"yaw_rate_hold={node.yaw_rate_hold}, "
        f"yaw_rate_kp={node.yaw_rate_kp:.2f}, "
        f"speed_rate L/R={node.left_speed_rate:.2f}/{node.right_speed_rate:.2f}"
    )
    print(f"deadman timeout={node.deadman_timeout:.1f}s")

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not node.quit_requested:
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                node.handle_key(sys.stdin.read(1))
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nRobot stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
