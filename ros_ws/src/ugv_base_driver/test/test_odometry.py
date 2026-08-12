import math

import pytest

from ugv_base_driver.odometry import DifferentialOdometry


def test_straight_motion_uses_calibrated_centimetre_ticks():
    odometry = DifferentialOdometry(meters_per_tick=0.01, track_width=0.41)
    assert odometry.update(100, 200, 1.0) is None
    update = odometry.update(106, 206, 2.0)
    assert update.x == pytest.approx(0.06)
    assert update.y == pytest.approx(0.0)
    assert update.yaw == pytest.approx(0.0)
    assert update.linear_velocity == pytest.approx(0.06)


def test_rotation_uses_track_width():
    odometry = DifferentialOdometry(meters_per_tick=0.01, track_width=0.41)
    odometry.update(0, 0, 1.0)
    update = odometry.update(-5, 5, 2.0)
    assert update.x == pytest.approx(0.0)
    assert update.y == pytest.approx(0.0)
    assert update.yaw == pytest.approx(0.10 / 0.41)
    assert update.angular_velocity == pytest.approx(0.10 / 0.41)


def test_arc_integrates_at_midpoint_heading():
    odometry = DifferentialOdometry(meters_per_tick=0.01, track_width=0.40)
    odometry.update(0, 0, 1.0)
    update = odometry.update(2, 6, 2.0)
    assert update.x == pytest.approx(0.04 * math.cos(0.05))
    assert update.y == pytest.approx(0.04 * math.sin(0.05))
    assert update.yaw == pytest.approx(0.10)


def test_counter_jump_rebaselines_without_moving_pose():
    odometry = DifferentialOdometry(
        meters_per_tick=0.01,
        track_width=0.41,
        max_tick_jump=20,
    )
    odometry.update(0, 0, 1.0)
    assert odometry.update(1000, 1000, 2.0) is None
    update = odometry.update(1001, 1001, 3.0)
    assert update.x == pytest.approx(0.01)


def test_invalid_calibration_is_rejected():
    with pytest.raises(ValueError):
        DifferentialOdometry(meters_per_tick=0.0, track_width=0.41)
    with pytest.raises(ValueError):
        DifferentialOdometry(meters_per_tick=0.01, track_width=0.0)
