"""ROS 2 serial driver for the RoboInspect Waveshare UGV base."""

import json
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import serial
from tf2_ros import TransformBroadcaster

from ugv_base_driver.odometry import DifferentialOdometry


class UgvBaseNode(Node):
    """Bridge Waveshare JSON feedback and commands to namespaced ROS topics."""

    def __init__(self):
        super().__init__('ugv_base_node')
        self._declare_parameters()
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.max_linear = self._positive_parameter('max_linear')
        self.max_angular = self._positive_parameter('max_angular')
        self.deadman_timeout = self._positive_parameter('deadman_timeout')
        self.feedback_timeout = self._positive_parameter('feedback_timeout')
        self.command_rate = self._positive_parameter('command_rate')
        self.voltage_scale = self._positive_parameter('voltage_scale')
        self.odom_frame = self._frame_parameter('odom_frame')
        self.base_frame = self._frame_parameter('base_frame')
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.odometry = DifferentialOdometry(
            meters_per_tick=self._positive_parameter('meters_per_tick'),
            track_width=self._positive_parameter('track_width'),
            max_tick_jump=int(self.get_parameter('max_tick_jump').value),
        )

        self._running = True
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._target_linear = 0.0
        self._target_angular = 0.0
        self._last_command_time = None
        self._last_feedback_time = None
        self._last_feedback = {}
        self._serial_error = ''
        self._parse_error = ''
        self._feedback_count = 0
        self._odometry_feedback_count = 0
        self._odometry_publish_count = 0

        self._odom_publisher = self.create_publisher(Odometry, 'odom', 20)
        self._battery_publisher = self.create_publisher(
            BatteryState, 'battery_state', 10)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 20)
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None)

        self._serial = serial.Serial(
            self.port,
            self.baud,
            timeout=0.05,
            write_timeout=0.20,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._reader = threading.Thread(
            target=self._read_loop,
            name='ugv-base-serial-reader',
            daemon=True,
        )
        self._reader.start()

        self._command_timer = self.create_timer(
            1.0 / self.command_rate, self._command_loop)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)

        # The controller needs a short settling time before accepting T131.
        time.sleep(1.0)
        self._enable_feedback()
        self._send_motion(0.0, 0.0)
        self.get_logger().info(
            f'UGV base ready on {self.port} at {self.baud} baud; '
            f'topics={self.get_namespace()}/cmd_vel,{self.get_namespace()}/odom; '
            f'frames={self.odom_frame}->{self.base_frame}')

    def _declare_parameters(self):
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_linear', 0.15)
        self.declare_parameter('max_angular', 0.80)
        self.declare_parameter('deadman_timeout', 0.40)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('feedback_timeout', 2.0)
        self.declare_parameter('meters_per_tick', 0.01)
        self.declare_parameter('track_width', 0.41)
        self.declare_parameter('max_tick_jump', 100)
        self.declare_parameter('voltage_scale', 0.01)
        self.declare_parameter('odom_frame', 'ugv/odom')
        self.declare_parameter('base_frame', 'ugv/base_link')
        self.declare_parameter('publish_tf', True)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive, got {value}')
        return value

    def _frame_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip('/')
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, float(value)))

    def _send_json(self, payload: dict):
        line = json.dumps(payload, separators=(',', ':')) + '\n'
        try:
            with self._write_lock:
                self._serial.write(line.encode('utf-8'))
                self._serial.flush()
            self._serial_error = ''
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            self._serial_error = str(exc)
            raise

    def _enable_feedback(self):
        self._send_json({'T': 131, 'cmd': 1})

    def _disable_feedback(self):
        self._send_json({'T': 131, 'cmd': 0})

    def _send_motion(self, linear: float, angular: float):
        self._send_json({
            'T': 13,
            'X': round(self._clamp(linear, self.max_linear), 3),
            'Z': round(self._clamp(angular, self.max_angular), 3),
        })

    def _on_cmd_vel(self, message: Twist):
        with self._command_lock:
            self._target_linear = self._clamp(
                message.linear.x, self.max_linear)
            self._target_angular = self._clamp(
                message.angular.z, self.max_angular)
            self._last_command_time = time.monotonic()

    def _command_loop(self):
        now = time.monotonic()
        with self._command_lock:
            command_age = (
                now - self._last_command_time
                if self._last_command_time is not None else math.inf)
            if command_age <= self.deadman_timeout:
                linear = self._target_linear
                angular = self._target_angular
            else:
                linear = 0.0
                angular = 0.0
        try:
            self._send_motion(linear, angular)
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            # The diagnostics timer reports the persistent error without
            # flooding logs at the command rate.
            pass

    def _read_loop(self):
        while self._running:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                start = line.find('{')
                end = line.rfind('}')
                if start < 0 or end <= start:
                    continue
                try:
                    feedback = json.loads(line[start:end + 1])
                except json.JSONDecodeError:
                    continue
                self._last_feedback_time = time.monotonic()
                self._last_feedback = feedback
                self._feedback_count += 1
                if 'odl' in feedback and 'odr' in feedback:
                    self._odometry_feedback_count += 1
                    self._handle_odometry(feedback)
                if 'v' in feedback:
                    self._publish_battery(feedback)
            except (serial.SerialException, OSError) as exc:
                self._serial_error = str(exc)
                if self._running:
                    time.sleep(0.05)
            except Exception as exc:  # Defensive: malformed vendor telemetry.
                if self._running:
                    self._parse_error = repr(exc)

    def _handle_odometry(self, feedback: dict):
        update = self.odometry.update(
            int(feedback['odl']),
            int(feedback['odr']),
            time.monotonic(),
        )
        if update is None:
            return
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(0.5 * update.yaw)
        qw = math.cos(0.5 * update.yaw)

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = update.x
        message.pose.pose.position.y = update.y
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = update.linear_velocity
        message.twist.twist.angular.z = update.angular_velocity
        self._set_covariances(message)
        self._odom_publisher.publish(message)
        self._odometry_publish_count += 1

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = update.x
            transform.transform.translation.y = update.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _set_covariances(message: Odometry):
        # Conservative first-pass values. They deliberately acknowledge the
        # coarse (~1 cm) counters and wheel slip instead of claiming certainty.
        message.pose.covariance[0] = 0.04 ** 2
        message.pose.covariance[7] = 0.04 ** 2
        message.pose.covariance[14] = 1e6
        message.pose.covariance[21] = 1e6
        message.pose.covariance[28] = 1e6
        message.pose.covariance[35] = math.radians(8.0) ** 2
        message.twist.covariance[0] = 0.03 ** 2
        message.twist.covariance[7] = 1e6
        message.twist.covariance[14] = 1e6
        message.twist.covariance[21] = 1e6
        message.twist.covariance[28] = 1e6
        message.twist.covariance[35] = math.radians(10.0) ** 2

    def _publish_battery(self, feedback: dict):
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.voltage = float(feedback['v']) * self.voltage_scale
        message.present = True
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._battery_publisher.publish(message)

    def _publish_diagnostics(self):
        now = time.monotonic()
        feedback_age = (
            now - self._last_feedback_time
            if self._last_feedback_time is not None else math.inf)
        status = DiagnosticStatus()
        status.name = f'{self.get_namespace()}/ugv_base'
        status.hardware_id = 'waveshare-ugv01'
        if self._serial_error:
            status.level = DiagnosticStatus.ERROR
            status.message = f'serial error: {self._serial_error}'
        elif feedback_age > self.feedback_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = 'chassis feedback timeout'
            try:
                self._enable_feedback()
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                pass
        elif self._parse_error:
            status.level = DiagnosticStatus.ERROR
            status.message = f'feedback processing error: {self._parse_error}'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'base feedback healthy'
        status.values = [
            KeyValue(key='feedback_age_sec', value=f'{feedback_age:.3f}'),
            KeyValue(
                key='left_ticks',
                value=str(self._last_feedback.get('odl', 'unknown'))),
            KeyValue(
                key='right_ticks',
                value=str(self._last_feedback.get('odr', 'unknown'))),
            KeyValue(
                key='raw_voltage',
                value=str(self._last_feedback.get('v', 'unknown'))),
            KeyValue(key='feedback_count', value=str(self._feedback_count)),
            KeyValue(
                key='odometry_feedback_count',
                value=str(self._odometry_feedback_count)),
            KeyValue(
                key='odometry_publish_count',
                value=str(self._odometry_publish_count)),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def shutdown(self):
        """Stop the chassis before releasing its serial port."""
        self._running = False
        for _ in range(2):
            try:
                self._send_motion(0.0, 0.0)
            except (serial.SerialException, serial.SerialTimeoutException, OSError):
                break
            time.sleep(0.05)
        try:
            self._disable_feedback()
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            pass
        try:
            self._serial.close()
        except (serial.SerialException, OSError):
            pass
        self._reader.join(timeout=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UgvBaseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
