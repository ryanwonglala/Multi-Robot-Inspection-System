import math

import pytest

from task_layer.inspection_runner import (
    calibrated_spin_command,
    direct_heading_and_distance,
    recoverable_viewpoint_near_miss,
    shortest_angular_distance,
)


def test_shortest_angular_distance_wraps_both_directions():
    assert shortest_angular_distance(math.radians(170), math.radians(-170)) \
        == pytest.approx(math.radians(20))
    assert shortest_angular_distance(math.radians(-170), math.radians(170)) \
        == pytest.approx(math.radians(-20))


def test_calibrated_spin_command_applies_measured_scale():
    command = calibrated_spin_command(0.0, math.pi / 3.0, 1.0 / 1.0472)
    assert command == pytest.approx(1.0, abs=1e-4)


def test_calibrated_spin_command_uses_short_path():
    command = calibrated_spin_command(
        math.radians(170), math.radians(-170), 0.95494)
    assert command == pytest.approx(math.radians(20) * 0.95494)


def test_direct_heading_and_distance_matches_real_vp2_vp3_leg():
    heading, distance = direct_heading_and_distance(
        (0.163, -1.475, -1.0472), 1.225, -1.595)
    assert heading == pytest.approx(-0.1125, abs=1e-4)
    assert distance == pytest.approx(1.0688, abs=1e-4)


def test_safe_segmented_boundary_miss_can_continue_video_workflow():
    result = {
        'status': 'segmented_xy_miss',
        'safe_to_continue': True,
        'xy_error_m': 0.0573,
        'xy_tolerance_m': 0.05,
    }
    assert recoverable_viewpoint_near_miss(result, 0.065) is True


@pytest.mark.parametrize('result', [
    {'status': 'aborted', 'safe_to_continue': True,
     'xy_error_m': 0.0573, 'xy_tolerance_m': 0.05},
    {'status': 'segmented_xy_miss', 'safe_to_continue': False,
     'xy_error_m': 0.0573, 'xy_tolerance_m': 0.05},
    {'status': 'segmented_xy_miss', 'safe_to_continue': True,
     'xy_error_m': 0.066, 'xy_tolerance_m': 0.05},
])
def test_continuity_never_masks_unsafe_or_large_failures(result):
    assert recoverable_viewpoint_near_miss(result, 0.065) is False
