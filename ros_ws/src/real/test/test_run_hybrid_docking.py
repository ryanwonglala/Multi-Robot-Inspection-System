import json
import math
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_hybrid_docking as docking  # noqa: E402


def test_heading_delta_uses_shortest_rotation():
    assert docking.heading_delta_degrees(
        math.radians(-175), math.radians(89)) == pytest.approx(-96.0)
    assert docking.heading_delta_degrees(
        math.radians(175), math.radians(-175)) == pytest.approx(10.0)


def test_standalone_vp3_handoff_uses_first_scan_yaw():
    authored = docking.Pose2D(1.225, -1.595, 0.970)
    handoff = docking.vp3_scan_start_pose(authored)
    assert handoff == docking.Pose2D(1.225, -1.595, 0.0)


def test_practical_tag_gate_accepts_verified_operational_edge():
    result = {
        "final": {
            "range_error_m": 0.0119,
            "bearing_error_deg": -1.09,
            "normal_yaw_error_deg": 3.99,
        }
    }
    assert docking.practical_tag_gate(result)
    result["final"]["normal_yaw_error_deg"] = 4.01
    assert not docking.practical_tag_gate(result)


def test_terminal_rotation_compensates_tag_normal_residual():
    result = {"final": {"normal_yaw_error_deg": 3.34}}
    assert docking.compensated_terminal_rotation(result) == pytest.approx(173.36)

    result["final"]["normal_yaw_error_deg"] = -2.0
    assert docking.compensated_terminal_rotation(result) == pytest.approx(178.7)

    result["final"]["normal_yaw_error_deg"] = 4.01
    with pytest.raises(ValueError, match="outside_compensation_gate"):
        docking.compensated_terminal_rotation(result)


def test_backup_continuation_requires_odom_lidar_agreement():
    result = {
        "reason": "motion_timeout",
        "final": {
            "remaining_m": 0.017,
            "rear_median_error_m": 0.018,
            "heading_error_deg": 0.08,
        },
    }
    assert docking.safe_backup_continuation(result) == 0.017
    result["final"]["rear_median_error_m"] = 0.040
    assert docking.safe_backup_continuation(result) is None


def test_shifted_tag_reference_is_robot_left_and_operator_validated():
    path = (Path(__file__).resolve().parents[1] /
            "config/apriltag_A_left_shift_1p5cm.json")
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["derivation"]["map_direction"] == "-X"
    assert data["derivation"]["validation_status"] == (
        "operator_confirmed_stable_2026-08-07")
    assert data["reference"]["horizontal_bearing_deg"] > 0.0
