from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from task_layer.photo_diff_check import CameraModel, changed_regions, detect_changes


def test_camera_defaults_match_model_sdf():
    camera = CameraModel()

    # Must stay in sync with turtlebot3_burger_cam_ns/model.sdf (640x480 pinhole,
    # camera mast at z=0.250 above the lidar plane).
    assert (camera.width, camera.height) == (640, 480)
    assert (camera.fx, camera.fy) == (320.0, 320.0)
    assert (camera.cx, camera.cy) == (320.5, 240.5)
    assert (camera.mount_x, camera.mount_z) == (0.076, 0.250)


def test_identical_view_has_no_anomaly(tmp_path: Path):
    image = np.full((480, 640, 3), 120, dtype=np.uint8)
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
    baseline_image = np.full((480, 640, 3), 120, dtype=np.uint8)
    current_image = baseline_image.copy()
    # A dark blob centered laterally (u ~ cx) on the ground ahead: rows below the
    # horizon (cy=240.5) so it back-projects to a forward range > 0.5 m.
    current_image[245:340, 230:410] = (10, 10, 10)
    baseline = tmp_path / 'baseline.png'
    current = tmp_path / 'current.png'
    assert cv2.imwrite(str(baseline), baseline_image)
    assert cv2.imwrite(str(current), current_image)

    result = detect_changes(
        baseline, current, (1.0, 2.0, 0.0),
        baseline_pose=(1.0, 2.0, 0.0))

    assert len(result['anomalies']) == 1
    anomaly = result['anomalies'][0]
    assert anomaly['area_px'] > 1500
    assert anomaly['x'] > 1.5
    assert anomaly['y'] == pytest.approx(2.0, abs=0.2)


def test_horizontal_sliver_is_rejected():
    mask = np.zeros((480, 640), dtype=np.uint8)
    # Wide, short band (w/h aspect well above max_aspect=6): floor/wall boundary
    # parallax residue, not a standing object -- changed_regions must reject it
    # on aspect even though its area (4000 px) clears the min-area floor.
    mask[100:120, 60:260] = 1

    assert changed_regions(mask) == []
