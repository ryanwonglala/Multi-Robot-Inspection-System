import importlib.util
from pathlib import Path
import sys
import types

import pytest


if 'serial' not in sys.modules:
    sys.modules['serial'] = types.SimpleNamespace(Serial=object)

SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'ugv02_opensource_node.py')
SPEC = importlib.util.spec_from_file_location(
    'ugv02_opensource_node', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def apply(requested, moving=False):
    return MODULE.directional_angular_floor(
        requested=requested,
        turning_detected=moving,
        start_ccw=0.50,
        start_cw=0.70,
        hold_ccw=0.35,
        hold_cw=0.45,
    )


def test_zero_command_is_never_modified():
    assert apply(0.0) == 0.0


def test_directional_start_floors_are_independent():
    assert apply(0.20) == pytest.approx(0.50)
    assert apply(-0.20) == pytest.approx(-0.70)


def test_hold_floor_reduces_after_turn_is_detected():
    assert apply(0.20, moving=True) == pytest.approx(0.35)
    assert apply(-0.20, moving=True) == pytest.approx(-0.45)


def test_requests_above_floor_are_not_reduced():
    assert apply(0.80) == pytest.approx(0.80)
    assert apply(-0.80) == pytest.approx(-0.80)


def test_firmware_boot_error_tracks_imu_setup_failure_and_recovery():
    error = MODULE.firmware_boot_error_from_line(
        '', 'Initialization of the sensor returned: Data Underflow')
    assert error.endswith('Data Underflow')
    assert MODULE.firmware_boot_error_from_line(
        error, 'Trying again...') == error
    assert MODULE.firmware_boot_error_from_line(
        error, 'Device connected!') == ''
