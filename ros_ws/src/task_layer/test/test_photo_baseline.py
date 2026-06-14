from pathlib import Path

import pytest
import yaml

from task_layer.photo_baseline import (
    BaselineLibrary,
    pose_distance,
    pose_within_tolerance,
)


def test_record_and_lookup_round_trip(tmp_path: Path):
    source = tmp_path / 'capture.ppm'
    source.write_bytes(b'P6\n1 1\n255\n\x01\x02\x03')
    library = BaselineLibrary(tmp_path / 'baselines')

    recorded = library.record(
        'central_hall', 2, source,
        {'x': 1.25, 'y': -0.5, 'yaw': 1.5708})

    assert Path(recorded['image_path']).read_bytes() == source.read_bytes()
    assert recorded['pose'] == {'x': 1.25, 'y': -0.5, 'yaw': 1.5708}
    reloaded = BaselineLibrary(tmp_path / 'baselines')
    assert reloaded.lookup('central_hall', 2) == recorded


def test_pose_tolerance_uses_planar_station_error():
    baseline = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    current = {'x': 1.18, 'y': 2.24, 'yaw': 2.5}

    assert pose_distance(baseline, current) == pytest.approx(0.3)
    assert pose_within_tolerance(baseline, current, 0.30)
    assert not pose_within_tolerance(baseline, current, 0.29)


def test_rejects_unknown_index_version(tmp_path: Path):
    root = tmp_path / 'baselines'
    root.mkdir()
    (root / 'index.yaml').write_text(
        yaml.safe_dump({'version': 99, 'views': {}}), encoding='utf-8')

    with pytest.raises(ValueError, match='unsupported baseline index version'):
        BaselineLibrary(root)
