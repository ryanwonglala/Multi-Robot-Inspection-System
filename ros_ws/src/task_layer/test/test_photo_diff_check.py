from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from task_layer.photo_diff_check import (
    CameraModel,
    changed_regions,
    detect_changes,
    detect_red_targets,
    diff_mask,
    merge_photo_detections,
)


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


def test_roi_top_frac_suppresses_out_of_arena_change(tmp_path: Path):
    # Real-arena scenario: low walls put the top of the frame past the arena,
    # where a person walking by changes the image. With roi_top_frac covering
    # that band the diff must stay silent; without it the change registers.
    baseline_image = np.full((480, 640, 3), 120, dtype=np.uint8)
    current_image = baseline_image.copy()
    current_image[30:140, 250:390] = (10, 10, 10)   # "person" above the wall line
    b = baseline_image
    c = current_image

    full = diff_mask(b, c)
    assert int(full.sum()) > 0

    masked = diff_mask(b, c, roi_top_frac=0.35)
    assert int(masked.sum()) == 0


def test_side_and_bottom_roi_ignore_alignment_borders():
    baseline = np.zeros((100, 100, 3), dtype=np.uint8)
    current = baseline.copy()
    current[40:60, :10] = 255
    current[90:, 40:60] = 255

    masked = diff_mask(
        baseline, current, threshold=25, tolerance_px=1,
        roi_side_frac=0.1, roi_bottom_frac=0.1,
        morph_kernel_px=1)

    assert int(masked.sum()) == 0


def test_roi_top_frac_keeps_floor_detection(tmp_path: Path):
    # Same geometry as test_physical_size_change_is_projected_to_map: a box on
    # the floor below the horizon. Enabling the real-arena ROI must not cost
    # the detection — anomaly targets live below the masked band.
    baseline_image = np.full((480, 640, 3), 120, dtype=np.uint8)
    current_image = baseline_image.copy()
    current_image[245:340, 230:410] = (10, 10, 10)
    baseline = tmp_path / 'baseline.png'
    current = tmp_path / 'current.png'
    assert cv2.imwrite(str(baseline), baseline_image)
    assert cv2.imwrite(str(current), current_image)

    result = detect_changes(
        baseline, current, (1.0, 2.0, 0.0),
        baseline_pose=(1.0, 2.0, 0.0), roi_top_frac=0.35)

    assert len(result['anomalies']) == 1
    assert result['anomalies'][0]['area_px'] > 1500


def test_small_object_mode_keeps_a_14_pixel_floor_target(tmp_path: Path):
    baseline_image = np.full((480, 640, 3), 120, dtype=np.uint8)
    current_image = baseline_image.copy()
    current_image[330:344, 310:324] = (15, 15, 15)
    baseline = tmp_path / 'small_baseline.png'
    current = tmp_path / 'small_current.png'
    assert cv2.imwrite(str(baseline), baseline_image)
    assert cv2.imwrite(str(current), current_image)

    result = detect_changes(
        baseline, current, (0.0, 0.0, 0.0),
        baseline_pose=(0.0, 0.0, 0.0),
        threshold=25, tolerance_px=2, min_area_px=80,
        roi_top_frac=0.57, morph_kernel_px=3,
        min_height_px=6, min_width_px=6)

    assert len(result['anomalies']) == 1
    assert result['anomalies'][0]['area_px'] >= 150


def test_red_target_mode_keeps_3cm_class_cube_and_rejects_cardboard(tmp_path: Path):
    cardboard = np.full((480, 640, 3), (70, 120, 180), dtype=np.uint8)
    clean_path = tmp_path / 'cardboard.png'
    assert cv2.imwrite(str(clean_path), cardboard)

    clean = detect_red_targets(
        clean_path, (0.0, 0.0, 0.0), min_area_px=150,
        roi_top_frac=0.57, roi_side_frac=0.125,
        roi_bottom_frac=0.0625)
    assert clean['anomalies'] == []

    with_cube = cardboard.copy()
    with_cube[330:344, 310:324] = (0, 0, 255)
    cube_path = tmp_path / 'red_cube.png'
    assert cv2.imwrite(str(cube_path), with_cube)
    detected = detect_red_targets(
        cube_path, (0.0, 0.0, 0.0), min_area_px=150,
        roi_top_frac=0.57, roi_side_frac=0.125,
        roi_bottom_frac=0.0625)

    assert len(detected['anomalies']) == 1
    assert detected['anomalies'][0]['detection_mode'] == 'red_target'
    assert detected['anomalies'][0]['area_px'] >= 190


def test_red_target_high_hue_branch_keeps_shaded_cube_without_warm_cardboard(
        tmp_path: Path):
    # Warm cardboard sits on the low-hue side and remains subject to the
    # strict default saturation floor. A shaded magenta-red cube occupies the
    # separately calibrated high-hue branch at lower saturation.
    image = np.full((480, 640, 3), (70, 120, 180), dtype=np.uint8)
    shaded_red_hsv = np.full((18, 18, 3), (175, 100, 120), dtype=np.uint8)
    image[330:348, 310:328] = cv2.cvtColor(
        shaded_red_hsv, cv2.COLOR_HSV2BGR)
    path = tmp_path / 'shaded_red_cube.png'
    assert cv2.imwrite(str(path), image)

    detected = detect_red_targets(
        path, (0.0, 0.0, 0.0), min_area_px=180,
        roi_top_frac=0.57, roi_side_frac=0.125,
        roi_bottom_frac=0.0625, hue_high_min=173,
        high_hue_saturation_min=90)

    assert len(detected['anomalies']) == 1
    assert detected['anomalies'][0]['area_px'] >= 300


def test_dedup_never_merges_pure_different_observed_zones():
    detections = [
        {'x': 0.0, 'y': 0.0, 'range': 0.5, 'observed_zone': 'vp1'},
        {'x': 0.2, 'y': 0.1, 'range': 0.6, 'observed_zone': 'vp2'},
    ]

    merged = merge_photo_detections([], detections, link_dist=1.4)

    assert len(merged) == 2
    assert {item['observed_zone'] for item in merged} == {'vp1', 'vp2'}


def test_dedup_merges_cross_zone_duplicate_from_two_mixed_sectors():
    detections = [
        {'x': 0.0, 'y': 0.0, 'range': 0.63, 'observed_zone': 'vp2',
         'sector_mixed': True, 'zone_boundary_margin_m': 0.02},
        {'x': 0.17, 'y': 0.0, 'range': 0.83, 'observed_zone': 'vp1',
         'sector_mixed': True, 'zone_boundary_margin_m': 0.22},
    ]

    merged = merge_photo_detections([], detections)

    assert len(merged) == 1
    assert merged[0]['observed_zone'] == 'vp1'


def test_dedup_keeps_spatially_distinct_mixed_sector_targets():
    detections = [
        {'x': 0.0, 'y': 0.0, 'range': 0.6, 'observed_zone': 'vp1',
         'sector_mixed': True, 'zone_boundary_margin_m': 0.01},
        {'x': 0.5, 'y': 0.0, 'range': 0.6, 'observed_zone': 'vp2',
         'sector_mixed': True, 'zone_boundary_margin_m': 0.01},
    ]

    assert len(merge_photo_detections([], detections)) == 2
