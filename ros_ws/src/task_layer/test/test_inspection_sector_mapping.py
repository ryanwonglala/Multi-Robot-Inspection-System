from task_layer.inspection_runner import (
    InspectionRunner,
    authored_scan_yaw_indices,
    classify_sector_zone,
    inspection_sector,
)
import pytest


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    def warn(self, _message):
        pass


class _RetryingSpinHarness:
    def __init__(self):
        self.results = [
            {'status': 'aborted', 'safe_to_continue': True},
            {'status': 'succeeded', 'safe_to_continue': True},
        ]
        self.settles = 0

    def get_parameter(self, name):
        values = {'scan_spin_max_attempts': 3, 'scan_spin_step_rad': 1.0}
        return _Parameter(values[name])

    def send_relative_scan_spin(self, _command, _target):
        return self.results.pop(0)

    def send_spin_to_map_yaw(self, _target):
        return self.results.pop(0)

    def wait_for_sensor_settle(self):
        self.settles += 1

    def get_logger(self):
        return _Logger()


def test_scan_spin_retries_only_after_confirmed_terminal_abort():
    harness = _RetryingSpinHarness()
    result = InspectionRunner.send_scan_spin_with_retries(
        harness, False, -2.0944)
    assert result['status'] == 'succeeded'
    assert result['attempt'] == 2
    assert harness.settles == 1


def test_scan_spin_does_not_retry_unconfirmed_goal():
    harness = _RetryingSpinHarness()
    harness.results = [
        {'status': 'spin_timeout', 'safe_to_continue': False},
        {'status': 'succeeded', 'safe_to_continue': True},
    ]
    result = InspectionRunner.send_scan_spin_with_retries(
        harness, False, -2.0944)
    assert result['status'] == 'spin_timeout'
    assert len(result['attempts']) == 1
    assert harness.settles == 0


def test_sector_lookup_uses_stop_and_yaw_index():
    area = {'inspection_sectors': {'viewpoint_1': [
        {'yaw_index': 0, 'observed_zones': ['anomaly_handling']},
        {'yaw_index': 1, 'observed_zones': ['vp1']},
    ]}}

    sector = inspection_sector(area, 'viewpoint_1', 1)

    assert sector['observed_zones'] == ['vp1']
    assert inspection_sector(area, 'viewpoint_2', 1) is None


def test_pure_sector_reports_its_authored_zone():
    sector = {'observed_zones': ['vp3']}

    assert classify_sector_zone(sector, 0.4) == 'vp3'


def test_mixed_sector_uses_coarse_near_far_split():
    sector = {
        'observed_zones': ['vp3', 'anomaly_handling'],
        'zone_rule': 'near_vp3_far_anomaly_handling',
        'zone_split_range_m': 0.70,
    }

    assert classify_sector_zone(sector, 0.69) == 'vp3'
    assert classify_sector_zone(sector, 0.71) == 'anomaly_handling'


def test_subset_scan_preserves_authored_yaw_index():
    area = {
        'scan_yaws': [2.0944],
        'scan_yaw_indices': [2],
    }

    assert authored_scan_yaw_indices(area, 1) == [2]


def test_scan_yaw_indices_must_match_scan_yaws():
    with pytest.raises(ValueError, match='exactly one index'):
        authored_scan_yaw_indices({'scan_yaw_indices': [1, 2]}, 1)
