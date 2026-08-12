import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'ugv_motion_probe.py')
SPEC = importlib.util.spec_from_file_location('ugv_motion_probe', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_default_battery_topic_matches_turtlebot3():
    args = MODULE.parse_args([])

    assert args.battery_topic == '/battery_state'


def test_battery_topic_help_names_the_default(capsys):
    with pytest.raises(SystemExit) as exc_info:
        MODULE.parse_args(['--help'])

    assert exc_info.value.code == 0
    assert 'BatteryState feedback topic (default: /battery_state)' in (
        capsys.readouterr().out)


def row(elapsed, phase, yaw, base_wz, camera_x, battery=12.3):
    return {
        'elapsed_s': elapsed,
        'phase': phase,
        'battery_v': battery,
        'odom_x_m': elapsed,
        'odom_y_m': 0.0,
        'odom_unwrapped_yaw_rad': yaw,
        'odom_linear_mps': 0.1,
        'odom_angular_rps': base_wz,
        'base_imu_wz_rps': base_wz,
        'camera_gyro_wx_rps': camera_x,
        'camera_gyro_wy_rps': 0.0,
        'camera_gyro_wz_rps': 0.0,
    }


def test_wrap_handles_pi_crossing():
    assert MODULE.wrap(math.radians(-358.0)) == pytest.approx(
        math.radians(2.0))


def test_integrate_removes_stationary_bias():
    rows = [
        row(0.0, 'motion', 0.0, 0.1, 0.0),
        row(1.0, 'motion', 1.0, 1.1, 0.0),
        row(2.0, 'settle', 2.0, 0.1, 0.0),
    ]
    assert MODULE.integrate(
        rows, 'base_imu_wz_rps', bias=0.1) == pytest.approx(1.0)


def test_summary_keeps_odom_and_camera_axes_separate():
    rows = [
        row(0.0, 'preflight', 0.0, 0.1, 0.2),
        row(1.0, 'preflight', 0.0, 0.1, 0.2),
        row(2.0, 'motion', 0.0, 1.1, 1.2),
        row(3.0, 'motion', 1.0, 1.1, 1.2),
        row(4.0, 'settle', 1.0, 0.1, 0.2),
    ]
    summary = MODULE.summarize(
        rows, 0.0, 0.5, 'completed', 'test')
    assert summary['odom_delta_yaw_rad'] == pytest.approx(1.0)
    assert summary['base_imu_wz_bias_rps'] == pytest.approx(0.1)
    assert summary['camera_gyro_bias_rps']['x'] == pytest.approx(0.2)
    # The final one-second settle interval includes the trapezoidal decay
    # from 1 rad/s above bias to stationary, adding another 0.5 rad.
    assert summary['camera_gyro_integral_rad']['x'] == pytest.approx(1.5)
