#!/usr/bin/env python3
"""Run one bounded UGV motion pulse and save synchronized measurements.

The program is deliberately conservative:

* zero motion is the default;
* non-zero motion needs ``--enable-motion``;
* odometry, base IMU and battery feedback must all be fresh;
* voltage and command/duration limits are checked before motion;
* loss of a required stream causes an immediate repeated zero command;
* every exit path publishes zero commands before shutting down.

Use one invocation per calibration point.  This keeps the operator in the
loop and avoids an unattended sweep through unknown motor dead zones.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu


REQUIRED_MAX_AGE = {
    'odom': 0.60,
    'base_imu': 0.60,
    'battery': 2.50,
}


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odometry(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


class MotionProbe(Node):
    """Collect the latest sensor state while publishing a bounded command."""

    FIELDNAMES = [
        'elapsed_s',
        'phase',
        'cmd_linear_mps',
        'cmd_angular_rps',
        'battery_v',
        'odom_x_m',
        'odom_y_m',
        'odom_yaw_rad',
        'odom_unwrapped_yaw_rad',
        'odom_linear_mps',
        'odom_angular_rps',
        'base_imu_wx_rps',
        'base_imu_wy_rps',
        'base_imu_wz_rps',
        'camera_gyro_wx_rps',
        'camera_gyro_wy_rps',
        'camera_gyro_wz_rps',
        'odom_age_s',
        'base_imu_age_s',
        'battery_age_s',
        'camera_gyro_age_s',
    ]

    def __init__(self, args):
        super().__init__('ugv_motion_probe')
        self.publisher = self.create_publisher(Twist, args.cmd_topic, 10)
        # BEST_EFFORT readers match both BEST_EFFORT and RELIABLE publishers.
        self.create_subscription(
            Odometry, args.odom_topic, self._on_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, args.imu_topic, self._on_base_imu,
            qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, args.battery_topic, self._on_battery,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, args.camera_gyro_topic, self._on_camera_gyro,
            qos_profile_sensor_data)

        self.started_at = time.monotonic()
        self.received_at = {}
        self.odom = None
        self.odom_unwrapped_yaw = None
        self._last_odom_yaw = None
        self.base_imu = None
        self.camera_gyro = None
        self.battery_voltage = None
        self.rows = []

    def _on_odom(self, message):
        yaw = yaw_from_odometry(message)
        if self._last_odom_yaw is None:
            self.odom_unwrapped_yaw = yaw
        else:
            self.odom_unwrapped_yaw += wrap(yaw - self._last_odom_yaw)
        self._last_odom_yaw = yaw
        self.odom = message
        self.received_at['odom'] = time.monotonic()

    def _on_base_imu(self, message):
        self.base_imu = message
        self.received_at['base_imu'] = time.monotonic()

    def _on_camera_gyro(self, message):
        self.camera_gyro = message
        self.received_at['camera_gyro'] = time.monotonic()

    def _on_battery(self, message):
        self.battery_voltage = finite_or_none(message.voltage)
        self.received_at['battery'] = time.monotonic()

    def publish(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def stop(self):
        for _ in range(12):
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.015)

    def missing_or_stale(self, now=None):
        now = time.monotonic() if now is None else now
        problems = []
        for name, max_age in REQUIRED_MAX_AGE.items():
            received_at = self.received_at.get(name)
            if received_at is None:
                problems.append(f'{name}=missing')
            elif now - received_at > max_age:
                problems.append(f'{name}=stale({now - received_at:.2f}s)')
        if self.battery_voltage is None:
            problems.append('battery_voltage=invalid')
        return problems

    def record(self, phase, linear, angular):
        now = time.monotonic()
        odom = self.odom
        base = self.base_imu
        camera = self.camera_gyro

        def age(name):
            received_at = self.received_at.get(name)
            return '' if received_at is None else now - received_at

        self.rows.append({
            'elapsed_s': now - self.started_at,
            'phase': phase,
            'cmd_linear_mps': linear,
            'cmd_angular_rps': angular,
            'battery_v': (
                '' if self.battery_voltage is None else self.battery_voltage),
            'odom_x_m': '' if odom is None else odom.pose.pose.position.x,
            'odom_y_m': '' if odom is None else odom.pose.pose.position.y,
            'odom_yaw_rad': (
                '' if odom is None else yaw_from_odometry(odom)),
            'odom_unwrapped_yaw_rad': (
                '' if self.odom_unwrapped_yaw is None
                else self.odom_unwrapped_yaw),
            'odom_linear_mps': (
                '' if odom is None else odom.twist.twist.linear.x),
            'odom_angular_rps': (
                '' if odom is None else odom.twist.twist.angular.z),
            'base_imu_wx_rps': (
                '' if base is None else base.angular_velocity.x),
            'base_imu_wy_rps': (
                '' if base is None else base.angular_velocity.y),
            'base_imu_wz_rps': (
                '' if base is None else base.angular_velocity.z),
            'camera_gyro_wx_rps': (
                '' if camera is None else camera.angular_velocity.x),
            'camera_gyro_wy_rps': (
                '' if camera is None else camera.angular_velocity.y),
            'camera_gyro_wz_rps': (
                '' if camera is None else camera.angular_velocity.z),
            'odom_age_s': age('odom'),
            'base_imu_age_s': age('base_imu'),
            'battery_age_s': age('battery'),
            'camera_gyro_age_s': age('camera_gyro'),
        })


def numeric_rows(rows, phase=None):
    if phase is None:
        return list(rows)
    return [row for row in rows if row['phase'] == phase]


def integrate(rows, field, bias=0.0):
    """Trapezoid-integrate one numeric CSV field over elapsed time."""
    samples = []
    for row in rows:
        value = row.get(field, '')
        if value != '' and math.isfinite(float(value)):
            samples.append((float(row['elapsed_s']), float(value) - bias))
    return sum(
        0.5 * (left[1] + right[1]) * (right[0] - left[0])
        for left, right in zip(samples, samples[1:]))


def mean(rows, field):
    values = [
        float(row[field]) for row in rows
        if row.get(field, '') != '' and math.isfinite(float(row[field]))
    ]
    return sum(values) / len(values) if values else None


def maximum_absolute(rows, field):
    values = [
        abs(float(row[field])) for row in rows
        if row.get(field, '') != '' and math.isfinite(float(row[field]))
    ]
    return max(values) if values else None


def summarize(rows, requested_linear, requested_angular, outcome, detail):
    preflight = numeric_rows(rows, 'preflight')
    measurement = [
        row for row in rows if row['phase'] in ('motion', 'settle')]
    base_bias = mean(preflight, 'base_imu_wz_rps')
    camera_biases = {
        axis: mean(preflight, f'camera_gyro_w{axis}_rps')
        for axis in ('x', 'y', 'z')
    }

    def endpoint_delta(field):
        values = [
            float(row[field]) for row in measurement
            if row.get(field, '') != ''
        ]
        return values[-1] - values[0] if len(values) >= 2 else None

    voltages = [
        float(row['battery_v']) for row in rows if row['battery_v'] != ''
    ]
    return {
        'outcome': outcome,
        'detail': detail,
        'requested_linear_mps': requested_linear,
        'requested_angular_rps': requested_angular,
        'sample_count': len(rows),
        'battery_start_v': voltages[0] if voltages else None,
        'battery_min_v': min(voltages) if voltages else None,
        'battery_end_v': voltages[-1] if voltages else None,
        'odom_delta_x_m': endpoint_delta('odom_x_m'),
        'odom_delta_y_m': endpoint_delta('odom_y_m'),
        'odom_delta_yaw_rad': endpoint_delta('odom_unwrapped_yaw_rad'),
        'peak_abs_odom_linear_mps': maximum_absolute(
            measurement, 'odom_linear_mps'),
        'peak_abs_odom_angular_rps': maximum_absolute(
            measurement, 'odom_angular_rps'),
        'base_imu_wz_bias_rps': base_bias,
        'base_imu_delta_yaw_rad': (
            None if base_bias is None else integrate(
                measurement, 'base_imu_wz_rps', base_bias)),
        'camera_gyro_bias_rps': camera_biases,
        'camera_gyro_integral_rad': {
            axis: (
                None if camera_biases[axis] is None else integrate(
                    measurement, f'camera_gyro_w{axis}_rps',
                    camera_biases[axis]))
            for axis in ('x', 'y', 'z')
        },
    }


def run_phase(node, phase, seconds, linear, angular, enforce_health):
    """Publish/sample at about 20 Hz for a bounded phase."""
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if enforce_health:
            problems = node.missing_or_stale()
            if problems:
                raise RuntimeError(
                    'required feedback lost: ' + ', '.join(problems))
        node.publish(linear, angular)
        node.record(phase, linear, angular)
        time.sleep(0.03)


def make_paths(output_dir, label):
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    safe_label = ''.join(
        character if character.isalnum() or character in '-_' else '_'
        for character in label)
    stem = output_dir / f'{stamp}_{safe_label}'
    return stem.with_suffix('.csv'), stem.with_suffix('.json')


def write_results(node, csv_path, json_path, summary, args):
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=MotionProbe.FIELDNAMES)
        writer.writeheader()
        writer.writerows(node.rows)
    payload = {
        'schema_version': 1,
        'created_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'topics': {
            'cmd_vel': args.cmd_topic,
            'odom': args.odom_topic,
            'base_imu': args.imu_topic,
            'battery': args.battery_topic,
            'camera_gyro': args.camera_gyro_topic,
        },
        'limits': {
            'minimum_voltage_v': args.min_voltage,
            'maximum_linear_mps': args.max_linear,
            'maximum_angular_rps': args.max_angular,
            'maximum_motion_duration_s': args.max_duration,
        },
        'summary': summary,
    }
    json_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run one safe UGV motion pulse and record sensor evidence.')
    command = parser.add_mutually_exclusive_group()
    command.add_argument(
        '--linear', type=float, default=0.0,
        help='requested linear.x in m/s')
    command.add_argument(
        '--angular', type=float, default=0.0,
        help='requested angular.z in rad/s; positive is CCW')
    parser.add_argument('--duration', type=float, default=0.50)
    parser.add_argument('--preflight-seconds', type=float, default=2.0)
    parser.add_argument('--settle-seconds', type=float, default=2.0)
    parser.add_argument('--min-voltage', type=float, default=12.0)
    parser.add_argument('--max-linear', type=float, default=0.20)
    parser.add_argument('--max-angular', type=float, default=0.80)
    parser.add_argument('--max-duration', type=float, default=2.0)
    parser.add_argument(
        '--enable-motion', action='store_true',
        help='required for a non-zero command')
    parser.add_argument('--label', default='stationary')
    parser.add_argument(
        '--output-dir', type=Path,
        default=Path.cwd() / 'ugv_motion_logs')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument(
        '--battery-topic', default='/battery_state',
        help='BatteryState feedback topic (default: /battery_state)')
    parser.add_argument(
        '--camera-gyro-topic', default='/camera/camera/gyro/sample')
    return parser.parse_args(argv)


def validate(args):
    if args.duration <= 0.0 or args.duration > args.max_duration:
        raise ValueError(
            f'duration must be in (0, {args.max_duration:.2f}] seconds')
    if args.preflight_seconds < 1.0:
        raise ValueError('preflight-seconds must be at least 1.0')
    if args.settle_seconds < 0.5:
        raise ValueError('settle-seconds must be at least 0.5')
    if abs(args.linear) > args.max_linear:
        raise ValueError(
            f'|linear| exceeds safety limit {args.max_linear:.3f} m/s')
    if abs(args.angular) > args.max_angular:
        raise ValueError(
            f'|angular| exceeds safety limit {args.max_angular:.3f} rad/s')
    moving = abs(args.linear) > 1e-9 or abs(args.angular) > 1e-9
    if moving and not args.enable_motion:
        raise ValueError('non-zero motion requires --enable-motion')
    return moving


def main():
    args = parse_args()
    try:
        moving = validate(args)
    except ValueError as exc:
        print(f'BLOCKED: {exc}', file=sys.stderr)
        return 2

    if args.label == 'stationary' and moving:
        direction = (
            'linear' if abs(args.linear) > 1e-9 else
            ('ccw' if args.angular > 0.0 else 'cw'))
        args.label = f'{direction}_{args.linear:+.3f}_{args.angular:+.3f}'

    csv_path, json_path = make_paths(args.output_dir, args.label)
    rclpy.init()
    node = MotionProbe(args)
    outcome = 'aborted'
    detail = 'unexpected shutdown'
    exit_code = 2

    def request_stop(_signum, _frame):
        raise KeyboardInterrupt

    old_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        # Preflight publishes zero, allowing the chassis deadman/heartbeat to
        # remain exercised without authorizing movement.
        run_phase(
            node, 'preflight', args.preflight_seconds,
            0.0, 0.0, enforce_health=False)
        problems = node.missing_or_stale()
        if problems:
            raise RuntimeError(
                'preflight failed: ' + ', '.join(problems))
        if node.battery_voltage < args.min_voltage:
            raise RuntimeError(
                f'battery {node.battery_voltage:.2f} V is below '
                f'{args.min_voltage:.2f} V motion threshold')

        if moving:
            print(
                f'ARMED: {args.duration:.2f}s command '
                f'linear={args.linear:+.3f} m/s, '
                f'angular={args.angular:+.3f} rad/s; starting in 3 seconds',
                flush=True)
            run_phase(node, 'countdown', 3.0, 0.0, 0.0, True)
            run_phase(
                node, 'motion', args.duration,
                args.linear, args.angular, True)
        else:
            print('STATIONARY probe: no non-zero command will be sent')
            run_phase(
                node, 'stationary', args.duration,
                0.0, 0.0, True)

        node.stop()
        run_phase(
            node, 'settle', args.settle_seconds,
            0.0, 0.0, True)
        outcome = 'completed'
        detail = 'bounded pulse and settle completed'
        exit_code = 0
    except KeyboardInterrupt:
        outcome = 'aborted'
        detail = 'operator interrupt'
        exit_code = 130
    except RuntimeError as exc:
        outcome = 'blocked'
        detail = str(exc)
        print(f'BLOCKED: {exc}', file=sys.stderr)
        exit_code = 2
    finally:
        node.stop()
        summary = summarize(
            node.rows, args.linear, args.angular, outcome, detail)
        write_results(node, csv_path, json_path, summary, args)
        print(f'CSV:  {csv_path}')
        print(f'JSON: {json_path}')
        print(json.dumps(summary, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_sigterm)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
