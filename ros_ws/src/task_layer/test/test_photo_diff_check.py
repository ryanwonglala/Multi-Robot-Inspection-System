from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from task_layer.photo_diff_check import CameraModel, changed_regions, detect_changes


def test_asus_c3_defaults_match_model_sdf():
    camera = CameraModel()

    assert (camera.width, camera.height) == (1280, 720)
    assert (camera.fx, camera.fy) == (640.0, 640.0)
    assert (camera.cx, camera.cy) == (639.5, 359.5)
    assert (camera.mount_x, camera.mount_z) == (0.076, 0.093)


def test_identical_view_has_no_anomaly(tmp_path: Path):
    image = np.full((720, 1280, 3), 120, dtype=np.uint8)
    baseline = tmp_path / 'baseline.png'
    current = tmp_path / 'current.png'
    assert cv2.imwrite(str(baseline), image)
    assert cv2.imwrite(str(current), image)

    result = detect_changes(
        baseline, current, (0.0, 0.0, 0.0),
        baseline_pose=(0.0, 0.0, 0.0))

    assert result['status'] == 'checked'
    assert result['anomalies'] == []


def test_physical_size_change_is_projected_to_map(tmp_path: Path):
    baseline_image = np.full((720, 1280, 3), 120, dtype=np.uint8)
    current_image = baseline_image.copy()
    current_image[100:430, 480:800] = (10, 10, 10)
    baseline = tmp_path / 'baseline.png'
    current = tmp_path / 'current.png'
    assert cv2.imwrite(str(baseline), baseline_image)
    assert cv2.imwrite(str(current), current_image)

    result = detect_changes(
        baseline, current, (1.0, 2.0, 0.0),
        baseline_pose=(1.0, 2.0, 0.0))

    assert len(result['anomalies']) == 1
    anomaly = result['anomalies'][0]
    assert anomaly['area_px'] > 1800
    assert anomaly['extent'] >= 0.25
    assert anomaly['x'] > 1.5
    assert anomaly['y'] == pytest.approx(2.0, abs=0.1)


def test_tall_thin_parallax_region_is_not_a_target():
    mask = np.zeros((720, 1280), dtype=np.uint8)
    mask[10:410, 100:125] = 1

    assert changed_regions(mask) == []
