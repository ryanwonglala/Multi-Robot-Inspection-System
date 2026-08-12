#!/usr/bin/env python3
#
# UGV02 ROS2 driver based on public UGV02/Wave Rover ROS2 work:
# - aimeesmallbeck/AimeeCloud, src/aimee_ugv02_controller:
#   JSON protocol, continuous feedback, sensor scales, tuned ticks_per_meter.
# - ArshamFN/WaveShare-Jetson-ROS2-Rover:
#   Jetson /dev/rover practice, encoder field swap, gyrodometry, acceleration ramp.
#
# SPDX-License-Identifier: MPL-2.0

import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import serial
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def ramp_value(current: float, target: float, limit: float, dt: float) -> float:
    if limit <= 0.0:
        return target
    return current + clamp(target - current, -limit * dt, limit * dt)


def directional_angular_floor(
    requested: float,
    turning_detected: bool,
    start_ccw: float,
    start_cw: float,
    hold_ccw: float,
    hold_cw: float,
) -> float:
    """Apply an independently calibrated CW/CCW start-or-hold floor.

    Zero values disable compensation.  This function intentionally knows
    nothing about the vehicle: A2 calibration supplies the four floor values.
    """
    if abs(requested) < 1e-9:
        return 0.0
    ccw = requested > 0.0
    floor = (
        (hold_ccw if turning_detected else start_ccw)
        if ccw else
        (hold_cw if turning_detected else start_cw)
    )
    floor = max(0.0, float(floor))
    return math.copysign(max(abs(requested), floor), requested)


def firmware_boot_error_from_line(current: str, line: str) -> str:
    """Track the ESP32 setup-loop failure without treating it as JSON."""
    if "Initialization of the sensor returned:" in line:
        return line.strip()
    if "Device connected!" in line:
        return ""
    return current


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class UGV02OpenSourceNode(Node):
    CMD_WHEEL_SPEED = 1
    CMD_VELOCITY = 13
    CMD_IMU = 126
    CMD_ODOM_REQUEST = 130
    CMD_CONTINUOUS_FEEDBACK = 131
    CMD_ECHO = 143
    FEEDBACK_CONTINUOUS = 1001

    def __init__(self) -> None:
        super().__init__("ugv02_opensource_node")

        self.declare_parameter("serial_port", "/dev/rover")
        self.declare_parameter("fallback_ports", "/dev/ttyACM0,/dev/ttyUSB0,/dev/ttyTHS1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)

        # Defaults come from the public UGV02 examples noted in the file header.
        self.declare_parameter("control_mode", "velocity")
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("wheel_separation", 0.172)
        self.declare_parameter("odom_feedback_units", "centimeters")
        self.declare_parameter("ticks_per_meter", 106.0)
        self.declare_parameter("encoder_swap", False)
        self.declare_parameter("encoder_jump_limit_ticks", 5000)
        self.declare_parameter("odom_jump_limit_m", 1.0)
        # Skid steering makes encoder-only yaw drift badly during large turns.
        # Use the UGV02 IMU gyro for heading and keep encoders for translation.
        self.declare_parameter("use_gyro_heading", True)

        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("heartbeat_interval", 0.1)
        self.declare_parameter("configure_ugv_type", True)
        self.declare_parameter("ugv_main_type", 2)
        self.declare_parameter("ugv_module_type", 0)
        self.declare_parameter("left_speed_rate", 1.0)
        self.declare_parameter("right_speed_rate", 1.0)
        self.declare_parameter("linear_accel_limit", 0.8)
        self.declare_parameter("angular_accel_limit", 2.0)
        self.declare_parameter("min_motor_power", 0.0)
        # Directional velocity-mode floors are populated only after A2.
        # Keeping them at zero preserves raw command-response measurements.
        self.declare_parameter("angular_start_floor_ccw", 0.0)
        self.declare_parameter("angular_start_floor_cw", 0.0)
        self.declare_parameter("angular_hold_floor_ccw", 0.0)
        self.declare_parameter("angular_hold_floor_cw", 0.0)
        self.declare_parameter("turn_detect_start_wz", 0.08)
        self.declare_parameter("turn_detect_stop_wz", 0.03)

        self.declare_parameter("accel_scale", 0.001197)
        self.declare_parameter("gyro_scale", 0.001066)
        self.declare_parameter("gyro_z_sign", 1.0)
        self.declare_parameter("gyro_bias_alpha_stationary", 0.02)
        self.declare_parameter("stationary_linear_threshold", 0.02)
        self.declare_parameter("stationary_angular_threshold", 0.05)
        self.declare_parameter("stationary_distance_threshold", 0.002)
        self.declare_parameter("voltage_scale", 0.01)
        self.declare_parameter("battery_low_threshold", 11.5)
        self.declare_parameter("motion_start_min_voltage", 12.0)
        self.declare_parameter("motion_stop_min_voltage", 11.5)
        self.declare_parameter("motion_feedback_timeout", 0.6)
        self.declare_parameter("require_fresh_feedback_for_motion", True)
        self.declare_parameter("hard_stop_on_cmd_timeout", True)
        self.declare_parameter("hard_stop_on_zero_command", True)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.fallback_ports = [
            item.strip()
            for item in str(self.get_parameter("fallback_ports").value).split(",")
            if item.strip()
        ]
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.control_mode = str(self.get_parameter("control_mode").value)

        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.odom_feedback_units = str(self.get_parameter("odom_feedback_units").value)
        self.ticks_per_meter = float(self.get_parameter("ticks_per_meter").value)
        self.encoder_swap = bool(self.get_parameter("encoder_swap").value)
        self.encoder_jump_limit_ticks = int(self.get_parameter("encoder_jump_limit_ticks").value)
        self.odom_jump_limit_m = float(self.get_parameter("odom_jump_limit_m").value)
        self.use_gyro_heading = bool(self.get_parameter("use_gyro_heading").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.heartbeat_interval = float(self.get_parameter("heartbeat_interval").value)
        self.configure_ugv_type = bool(self.get_parameter("configure_ugv_type").value)
        self.ugv_main_type = int(self.get_parameter("ugv_main_type").value)
        self.ugv_module_type = int(self.get_parameter("ugv_module_type").value)
        self.left_speed_rate = float(self.get_parameter("left_speed_rate").value)
        self.right_speed_rate = float(self.get_parameter("right_speed_rate").value)
        self.linear_accel_limit = float(self.get_parameter("linear_accel_limit").value)
        self.angular_accel_limit = float(self.get_parameter("angular_accel_limit").value)
        self.min_motor_power = float(self.get_parameter("min_motor_power").value)
        self.angular_start_floor_ccw = float(
            self.get_parameter("angular_start_floor_ccw").value
        )
        self.angular_start_floor_cw = float(
            self.get_parameter("angular_start_floor_cw").value
        )
        self.angular_hold_floor_ccw = float(
            self.get_parameter("angular_hold_floor_ccw").value
        )
        self.angular_hold_floor_cw = float(
            self.get_parameter("angular_hold_floor_cw").value
        )
        self.turn_detect_start_wz = float(
            self.get_parameter("turn_detect_start_wz").value
        )
        self.turn_detect_stop_wz = float(
            self.get_parameter("turn_detect_stop_wz").value
        )

        self.accel_scale = float(self.get_parameter("accel_scale").value)
        self.gyro_scale = float(self.get_parameter("gyro_scale").value)
        self.gyro_z_sign = float(self.get_parameter("gyro_z_sign").value)
        self.gyro_bias_alpha_stationary = float(
            self.get_parameter("gyro_bias_alpha_stationary").value
        )
        self.stationary_linear_threshold = float(
            self.get_parameter("stationary_linear_threshold").value
        )
        self.stationary_angular_threshold = float(
            self.get_parameter("stationary_angular_threshold").value
        )
        self.stationary_distance_threshold = float(
            self.get_parameter("stationary_distance_threshold").value
        )
        self.voltage_scale = float(self.get_parameter("voltage_scale").value)
        self.battery_low_threshold = float(self.get_parameter("battery_low_threshold").value)
        self.motion_start_min_voltage = float(
            self.get_parameter("motion_start_min_voltage").value
        )
        self.motion_stop_min_voltage = float(
            self.get_parameter("motion_stop_min_voltage").value
        )
        self.motion_feedback_timeout = float(
            self.get_parameter("motion_feedback_timeout").value
        )
        self.require_fresh_feedback_for_motion = bool(
            self.get_parameter("require_fresh_feedback_for_motion").value
        )
        self.hard_stop_on_cmd_timeout = bool(
            self.get_parameter("hard_stop_on_cmd_timeout").value
        )
        self.hard_stop_on_zero_command = bool(
            self.get_parameter("hard_stop_on_zero_command").value
        )
        if self.motion_stop_min_voltage > self.motion_start_min_voltage:
            raise ValueError(
                "motion_stop_min_voltage must not exceed "
                "motion_start_min_voltage"
            )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        # Nav2's odometry subscriptions request RELIABLE.  BEST_EFFORT here
        # makes the graph visible but delivers no odometry to the controller.
        self.odom_pub = self.create_publisher(Odometry, "/odom", reliable_qos)
        self.imu_pub = self.create_publisher(Imu, "/imu", sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, "/battery", reliable_qos)
        self.status_pub = self.create_publisher(String, "/ugv02/status", reliable_qos)
        self.speed_rate_sub = self.create_subscription(
            String, "/ugv02/speed_rate", self.speed_rate_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None

        self.serial_handle: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.read_thread: Optional[threading.Thread] = None
        self.running = False
        self.connected_port = ""
        self.last_connect_attempt = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_left_odom: Optional[float] = None
        self.last_right_odom: Optional[float] = None
        self.last_odom_time = time.monotonic()
        self.last_feedback_time = 0.0
        self.gyro_bias_z = 0.0
        self.latest_voltage: Optional[float] = None
        self.battery_low = False
        self.motion_active = False
        self.motion_block_reason = "waiting_for_command"
        self.turning_detected = False
        self.turning_direction = 0
        self.firmware_boot_error = ""
        self.last_non_json_line = ""

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_control_time = time.monotonic()
        self.last_send_time = 0.0
        self.last_status_time = 0.0

        self.connect_serial()
        command_period = 1.0 / max(1.0, float(self.get_parameter("command_rate").value))
        self.create_timer(command_period, self.control_loop)
        self.create_timer(1.0, self.status_loop)

        self.get_logger().info(
            "UGV02 open-source based node ready: "
            f"mode={self.control_mode}, odom_units={self.odom_feedback_units}, "
            f"ticks_per_meter={self.ticks_per_meter}, "
            f"encoder_swap={self.encoder_swap}, use_gyro_heading={self.use_gyro_heading}"
        )

    def candidate_ports(self) -> list[str]:
        ports = [self.serial_port]
        for port in self.fallback_ports:
            if port not in ports:
                ports.append(port)
        return ports

    def connect_serial(self) -> bool:
        now = time.monotonic()
        if now - self.last_connect_attempt < 2.0:
            return False
        self.last_connect_attempt = now

        for port in self.candidate_ports():
            if port != self.serial_port and not os.path.exists(port):
                continue
            try:
                handle = serial.Serial(
                    port=port,
                    baudrate=self.baud_rate,
                    timeout=0.05,
                    write_timeout=0.2,
                    dsrdtr=False,
                    rtscts=False,
                )
                try:
                    handle.setRTS(False)
                    handle.setDTR(False)
                except Exception:
                    pass
                handle.reset_input_buffer()
                handle.reset_output_buffer()

                self.serial_handle = handle
                self.connected_port = port
                self.running = True
                self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
                self.read_thread.start()
                self.configure_rover_feedback()
                self.get_logger().info(f"Connected to UGV02 on {port} @ {self.baud_rate}")
                return True
            except serial.SerialException as exc:
                self.get_logger().warn(f"Could not open {port}: {exc}")

        self.get_logger().warn("No UGV02 serial port is available yet; will retry.")
        return False

    def configure_rover_feedback(self) -> None:
        self.send_json({"T": self.CMD_ECHO, "cmd": 0})
        time.sleep(0.1)
        if self.configure_ugv_type:
            self.send_json(
                {
                    "T": 900,
                    "main": self.ugv_main_type,
                    "module": self.ugv_module_type,
                }
            )
            time.sleep(0.1)
        self.apply_speed_rate(save=False)
        time.sleep(0.1)
        for _ in range(3):
            self.send_json({"T": self.CMD_CONTINUOUS_FEEDBACK, "cmd": 1})
            time.sleep(0.2)
        self.send_json({"T": self.CMD_ODOM_REQUEST})
        self.send_json({"T": self.CMD_IMU})

    def read_loop(self) -> None:
        buffer = bytearray()
        while self.running and self.serial_handle and self.serial_handle.is_open:
            try:
                with self.serial_lock:
                    waiting = self.serial_handle.in_waiting
                    chunk = self.serial_handle.read(max(1, min(waiting, 512)))
                if not chunk:
                    continue
                buffer.extend(chunk)
                while b"\n" in buffer:
                    raw, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    self.process_line(raw.decode("utf-8", errors="ignore").strip())
            except Exception as exc:
                self.get_logger().debug(f"Serial read error: {exc}")
                time.sleep(0.05)

    def process_line(self, line: str) -> None:
        if not line:
            return
        self.firmware_boot_error = firmware_boot_error_from_line(
            self.firmware_boot_error, line
        )
        start = line.find("{")
        end = line.rfind("}")
        if start < 0 or end <= start:
            self.last_non_json_line = line[-160:]
            return
        try:
            msg = json.loads(line[start : end + 1])
        except json.JSONDecodeError:
            return

        if msg.get("T") == self.FEEDBACK_CONTINUOUS or "odl" in msg or "odr" in msg:
            self.firmware_boot_error = ""
            self.process_continuous_feedback(msg)
        elif msg.get("T") == self.CMD_IMU or "gx" in msg or "ax" in msg:
            self.publish_imu(msg)
        else:
            self.get_logger().debug(f"UGV02 message: {msg}")

    def process_continuous_feedback(self, msg: Dict[str, Any]) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self.last_odom_time)
        self.last_odom_time = now
        self.last_feedback_time = now

        left_odom = self.safe_optional_float(msg.get("odl"))
        right_odom = self.safe_optional_float(msg.get("odr"))
        if left_odom is not None and right_odom is not None:
            if self.encoder_swap:
                left_odom, right_odom = right_odom, left_odom
            self.integrate_encoder_odom(left_odom, right_odom, dt, msg)

        if "ax" in msg or "gx" in msg or "r" in msg or "y" in msg:
            self.publish_imu(msg)
        if "v" in msg:
            self.publish_battery(msg)
        self.publish_odometry()

    def integrate_encoder_odom(
        self,
        left_odom: float,
        right_odom: float,
        dt: float,
        msg: Dict[str, Any],
    ) -> None:
        if self.last_left_odom is None or self.last_right_odom is None:
            self.last_left_odom = left_odom
            self.last_right_odom = right_odom
            return

        d_left_odom = left_odom - self.last_left_odom
        d_right_odom = right_odom - self.last_right_odom
        self.last_left_odom = left_odom
        self.last_right_odom = right_odom

        units = self.odom_feedback_units.strip().lower()
        if units in ("centimeter", "centimeters", "cm"):
            dl = d_left_odom / 100.0
            dr = d_right_odom / 100.0
            if abs(dl) > self.odom_jump_limit_m or abs(dr) > self.odom_jump_limit_m:
                self.get_logger().warn(
                    "Ignoring odom jump: "
                    f"left={dl:.3f}m, right={dr:.3f}m"
                )
                return
        elif units in ("tick", "ticks", "encoder_ticks"):
            if (
                abs(d_left_odom) > self.encoder_jump_limit_ticks
                or abs(d_right_odom) > self.encoder_jump_limit_ticks
            ):
                self.get_logger().warn(
                    "Ignoring encoder jump: "
                    f"left={d_left_odom:.3f}, right={d_right_odom:.3f}"
                )
                return
            dl = d_left_odom / self.ticks_per_meter
            dr = d_right_odom / self.ticks_per_meter
        else:
            if (
                abs(d_left_odom) > self.odom_jump_limit_m
                or abs(d_right_odom) > self.odom_jump_limit_m
            ):
                self.get_logger().warn(
                    "Ignoring odom jump: "
                    f"left={d_left_odom:.3f}m, right={d_right_odom:.3f}m"
                )
                return
            dl = d_left_odom
            dr = d_right_odom

        ds = 0.5 * (dl + dr)

        raw_gyro_z = self.raw_gyro_z(msg)
        stationary = (
            abs(self.command_linear) < self.stationary_linear_threshold
            and abs(self.command_angular) < self.stationary_angular_threshold
            and abs(ds) < self.stationary_distance_threshold
        )
        if raw_gyro_z is not None and stationary:
            alpha = clamp(self.gyro_bias_alpha_stationary, 0.0, 1.0)
            self.gyro_bias_z = (1.0 - alpha) * self.gyro_bias_z + alpha * raw_gyro_z

        gyro_z = None if raw_gyro_z is None else raw_gyro_z - self.gyro_bias_z
        if self.use_gyro_heading and gyro_z is not None:
            dtheta = gyro_z * dt
            self.wz = gyro_z
        else:
            dtheta = (dr - dl) / max(1e-6, self.wheel_separation)
            self.wz = dtheta / dt

        mid_yaw = self.yaw + 0.5 * dtheta
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw = wrap_pi(self.yaw + dtheta)
        self.vx = ds / dt

    def raw_gyro_z(self, msg: Dict[str, Any]) -> Optional[float]:
        try:
            return float(msg["gz"]) * self.gyro_scale * self.gyro_z_sign
        except (KeyError, TypeError, ValueError):
            return None

    def publish_odometry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        q = euler_to_quaternion(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05 if self.use_gyro_heading else 0.10
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.03 if self.use_gyro_heading else 0.08
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = q
            self.tf_broadcaster.sendTransform(transform)

    def publish_imu(self, msg: Dict[str, Any]) -> None:
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame

        if "ax" in msg or "gx" in msg:
            ax = self.safe_float(msg.get("ax"), 0.0) * self.accel_scale
            ay = self.safe_float(msg.get("ay"), 0.0) * self.accel_scale
            az = self.safe_float(msg.get("az"), 0.0) * self.accel_scale
            gx = self.safe_float(msg.get("gx"), 0.0) * self.gyro_scale
            gy = self.safe_float(msg.get("gy"), 0.0) * self.gyro_scale
            gz = self.safe_float(msg.get("gz"), 0.0) * self.gyro_scale

            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz * self.gyro_z_sign - self.gyro_bias_z

            roll = math.atan2(ay, az) if abs(az) > 1e-9 else 0.0
            pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
            if "mx" in msg and "my" in msg:
                yaw = math.atan2(self.safe_float(msg.get("my"), 0.0), self.safe_float(msg.get("mx"), 0.0))
            else:
                yaw = self.yaw
            imu.orientation = euler_to_quaternion(roll, pitch, yaw)
            imu.orientation_covariance[0] = 0.05
            imu.orientation_covariance[4] = 0.05
            imu.orientation_covariance[8] = 0.5
        elif "r" in msg or "p" in msg or "y" in msg:
            roll = math.radians(self.safe_float(msg.get("r"), 0.0))
            pitch = math.radians(self.safe_float(msg.get("p"), 0.0))
            yaw = math.radians(self.safe_float(msg.get("y"), 0.0))
            imu.orientation = euler_to_quaternion(roll, pitch, yaw)
        else:
            imu.orientation_covariance[0] = -1.0

        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1
        imu.angular_velocity_covariance[0] = 0.05
        imu.angular_velocity_covariance[4] = 0.05
        imu.angular_velocity_covariance[8] = 0.05
        self.imu_pub.publish(imu)

    def publish_battery(self, msg: Dict[str, Any]) -> None:
        voltage = self.safe_float(msg.get("v"), 0.0) * self.voltage_scale
        self.latest_voltage = voltage

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = voltage
        battery.present = True
        self.battery_pub.publish(battery)

        is_low = voltage < self.battery_low_threshold
        if is_low and not self.battery_low:
            self.get_logger().warn(
                f"UGV02 battery low: {voltage:.2f}V "
                f"(threshold {self.battery_low_threshold:.1f}V)"
            )
        self.battery_low = is_low

    def cmd_callback(self, msg: Twist) -> None:
        self.target_linear = clamp(float(msg.linear.x), -self.max_linear, self.max_linear)
        self.target_angular = clamp(float(msg.angular.z), -self.max_angular, self.max_angular)
        self.last_cmd_time = time.monotonic()

    def speed_rate_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            parts = [part.strip() for part in msg.data.split(",")]
            if len(parts) < 2:
                self.get_logger().warn(
                    "Invalid /ugv02/speed_rate. Use JSON like {\"L\":0.95,\"R\":1.0}."
                )
                return
            data = {"L": parts[0], "R": parts[1]}

        try:
            left = float(data.get("L", self.left_speed_rate))
            right = float(data.get("R", self.right_speed_rate))
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid speed rate values: {msg.data}")
            return

        self.left_speed_rate = clamp(abs(left), 0.0, 1.0)
        self.right_speed_rate = clamp(abs(right), 0.0, 1.0)
        self.apply_speed_rate(save=bool(data.get("save", False)))

    def apply_speed_rate(self, save: bool = False) -> None:
        self.send_json(
            {
                "T": 138,
                "L": round(self.left_speed_rate, 3),
                "R": round(self.right_speed_rate, 3),
            }
        )
        self.get_logger().info(
            f"UGV02 speed rate set: L={self.left_speed_rate:.3f}, "
            f"R={self.right_speed_rate:.3f}"
        )
        if save:
            time.sleep(0.05)
            self.send_json({"T": 140})
            self.get_logger().info("UGV02 speed rate save command sent.")

    def control_loop(self) -> None:
        if self.serial_handle is None or not self.serial_handle.is_open:
            self.connect_serial()
            return

        now = time.monotonic()
        dt = max(1e-3, min(0.5, now - self.last_control_time))
        self.last_control_time = now

        target_linear = self.target_linear
        target_angular = self.target_angular
        command_timed_out = now - self.last_cmd_time > self.cmd_timeout
        if command_timed_out:
            target_linear = 0.0
            target_angular = 0.0
            self.motion_active = False
            self.motion_block_reason = "cmd_timeout"

        requested_motion = (
            abs(target_linear) > 1e-6 or abs(target_angular) > 1e-6
        )
        if not requested_motion:
            self.motion_active = False
            self.turning_detected = False
            self.turning_direction = 0
            if not command_timed_out:
                self.motion_block_reason = "zero_command"
        else:
            safety_problem = self.motion_safety_problem(now)
            if safety_problem:
                target_linear = 0.0
                target_angular = 0.0
                self.motion_active = False
                self.turning_detected = False
                self.turning_direction = 0
                self.motion_block_reason = safety_problem
            else:
                self.motion_active = True
                self.motion_block_reason = ""

        requested_direction = (
            1 if target_angular > 1e-6 else
            (-1 if target_angular < -1e-6 else 0)
        )
        if requested_direction != self.turning_direction:
            self.turning_detected = False
            self.turning_direction = requested_direction
        if requested_direction:
            if (
                not self.turning_detected
                and abs(self.wz) >= self.turn_detect_start_wz
            ):
                self.turning_detected = True
            elif (
                self.turning_detected
                and abs(self.wz) <= self.turn_detect_stop_wz
            ):
                self.turning_detected = False
            if self.control_mode == "velocity":
                target_angular = directional_angular_floor(
                    target_angular,
                    self.turning_detected,
                    self.angular_start_floor_ccw,
                    self.angular_start_floor_cw,
                    self.angular_hold_floor_ccw,
                    self.angular_hold_floor_cw,
                )

        hard_stop = (
            (command_timed_out and self.hard_stop_on_cmd_timeout)
            or (not requested_motion and self.hard_stop_on_zero_command)
            or bool(self.motion_block_reason and requested_motion)
        )
        if hard_stop:
            self.command_linear = 0.0
            self.command_angular = 0.0
        else:
            self.command_linear = ramp_value(
                self.command_linear,
                target_linear,
                self.linear_accel_limit,
                dt,
            )
            self.command_angular = ramp_value(
                self.command_angular,
                target_angular,
                self.angular_accel_limit,
                dt,
            )

        if now - self.last_send_time >= self.heartbeat_interval:
            self.send_motion_command(self.command_linear, self.command_angular)
            self.last_send_time = now

    def motion_safety_problem(self, now: float) -> str:
        """Return an empty string when a requested motion is permitted."""
        if self.firmware_boot_error:
            return "esp32_firmware_boot_error"
        if self.require_fresh_feedback_for_motion:
            if self.last_feedback_time <= 0.0:
                return "no_chassis_feedback"
            feedback_age = now - self.last_feedback_time
            if feedback_age > self.motion_feedback_timeout:
                return f"chassis_feedback_stale:{feedback_age:.2f}s"
            if self.latest_voltage is None:
                return "no_battery_voltage"

        if self.latest_voltage is not None:
            threshold = (
                self.motion_stop_min_voltage
                if self.motion_active else
                self.motion_start_min_voltage
            )
            if self.latest_voltage < threshold:
                return (
                    f"battery_voltage_low:{self.latest_voltage:.2f}V"
                    f"<{threshold:.2f}V"
                )
        return ""

    def send_motion_command(self, linear: float, angular: float) -> None:
        if self.control_mode == "velocity":
            self.send_json(
                {
                    "T": self.CMD_VELOCITY,
                    "X": round(clamp(linear, -self.max_linear, self.max_linear), 4),
                    "Z": round(clamp(angular, -self.max_angular, self.max_angular), 4),
                }
            )
            return

        left = linear - 0.5 * angular * self.wheel_separation
        right = linear + 0.5 * angular * self.wheel_separation
        magnitude = max(abs(left), abs(right))
        if 1e-4 < magnitude < self.min_motor_power:
            scale = self.min_motor_power / magnitude
            left *= scale
            right *= scale

        self.send_json(
            {
                "T": self.CMD_WHEEL_SPEED,
                "L": round(clamp(left, -self.max_linear, self.max_linear), 4),
                "R": round(clamp(right, -self.max_linear, self.max_linear), 4),
            }
        )

    def send_json(self, data: Dict[str, Any]) -> None:
        if self.serial_handle is None or not self.serial_handle.is_open:
            return
        line = json.dumps(data, separators=(",", ":")) + "\n"
        try:
            with self.serial_lock:
                self.serial_handle.write(line.encode("utf-8"))
                self.serial_handle.flush()
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            self.close_serial()

    def status_loop(self) -> None:
        now = time.monotonic()
        feedback_age = None if self.last_feedback_time == 0.0 else now - self.last_feedback_time
        status = {
            "connected": self.serial_handle is not None and self.serial_handle.is_open,
            "port": self.connected_port,
            "mode": self.control_mode,
            "feedback_age": None if feedback_age is None else round(feedback_age, 2),
            "odom_units": self.odom_feedback_units,
            "encoder_swap": self.encoder_swap,
            "use_gyro_heading": self.use_gyro_heading,
            "gyro_bias_z": round(self.gyro_bias_z, 5),
            "battery_v": None if self.latest_voltage is None else round(self.latest_voltage, 2),
            "motion_active": self.motion_active,
            "motion_block_reason": self.motion_block_reason,
            "firmware_boot_error": self.firmware_boot_error,
            "last_non_json_line": self.last_non_json_line,
            "target_linear": round(self.target_linear, 4),
            "target_angular": round(self.target_angular, 4),
            "command_linear": round(self.command_linear, 4),
            "command_angular": round(self.command_angular, 4),
            "turning_detected": self.turning_detected,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 3),
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(message)
        if now - self.last_status_time > 5.0:
            self.last_status_time = now
            self.get_logger().info(message.data)

    def close_serial(self) -> None:
        self.running = False
        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except Exception:
                pass
        self.serial_handle = None
        self.connected_port = ""

    def stop_rover(self) -> None:
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.command_linear = 0.0
        self.command_angular = 0.0
        for _ in range(3):
            self.send_motion_command(0.0, 0.0)
            time.sleep(0.05)

    def destroy_node(self) -> None:
        self.stop_rover()
        self.send_json({"T": self.CMD_CONTINUOUS_FEEDBACK, "cmd": 0})
        self.close_serial()
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        super().destroy_node()

    @staticmethod
    def safe_int(value: Any) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def safe_optional_float(value: Any) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def safe_float(value: Any, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = UGV02OpenSourceNode()
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
