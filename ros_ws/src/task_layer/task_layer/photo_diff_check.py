#!/usr/bin/env python3
"""Photo-diff anomaly detection (P1-5v).

Compares an inspection photo against a baseline photo taken from the same
viewpoint when the scene was known-clean. Changed regions are anomaly
candidates; each candidate's ground-contact pixel is back-projected through
the pinhole camera model onto the floor plane to get a map coordinate.

Pure functions over numpy arrays — no ROS imports, so the whole detector is
unit-testable offline against saved photos. The camera geometry (intrinsics
and mount pose) must match sim/models/turtlebot3_burger_cam_ns/model.sdf.

Why this replaces the laser map-diff detector: the camera is passive with
respect to localization (an unmapped object in view does not perturb AMCL
the way unexplained laser beams do), works on objects of any size including
ones below the scan plane, and the same baseline-diff idea transfers to the
real robot once a baseline patrol has been recorded on site.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np


@dataclass
class CameraModel:
    """Pinhole intrinsics + mount pose in the robot base frame."""
    fx: float = 320.0
    fy: float = 320.0
    cx: float = 320.5
    cy: float = 240.5
    width: int = 640
    height: int = 480
    mount_x: float = 0.076   # camera ahead of base origin (m)
    mount_z: float = 0.250   # camera height above floor (m)


def load_image(path: str | Path) -> np.ndarray:
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f'unreadable image: {path}')
    return image


def _gray(image: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def estimate_shift(baseline: np.ndarray, current: np.ndarray) -> tuple[float, float]:
    """Global (dx, dy) pixel shift of `current` relative to `baseline`.

    Repeat visits to the same viewpoint differ by the AMCL pose error
    (centimetres + a few degrees); for a mostly-static scene that reads as
    an approximately uniform image shift, which phase correlation recovers
    cheaply. The residual misalignment is absorbed by the min/max tolerance
    band in diff_mask()."""
    b = np.float32(_gray(baseline))
    c = np.float32(_gray(current))
    (dx, dy), _response = cv2.phaseCorrelate(b, c)
    return dx, dy


def diff_mask(baseline: np.ndarray, current: np.ndarray,
              threshold: int = 35, tolerance_px: int = 7) -> np.ndarray:
    """Binary mask of pixels in `current` that cannot be explained by the
    baseline even allowing a small local wobble.

    The baseline is expanded into a per-pixel [min, max] band over a
    (2*tolerance_px+1)^2 neighborhood after global shift compensation:
    edges that merely moved a few pixels (pose error) stay inside the band,
    genuinely new surfaces fall outside it."""
    dx, dy = estimate_shift(baseline, current)
    matrix = np.float32([[1, 0, dx], [0, 1, dy]])
    aligned = cv2.warpAffine(
        baseline, matrix, (baseline.shape[1], baseline.shape[0]),
        borderMode=cv2.BORDER_REPLICATE)
    kernel = np.ones((2 * tolerance_px + 1, 2 * tolerance_px + 1), np.uint8)
    low = cv2.erode(aligned, kernel)
    high = cv2.dilate(aligned, kernel)
    outside = np.maximum(low.astype(np.int16) - current.astype(np.int16),
                         current.astype(np.int16) - high.astype(np.int16))
    mask = (outside.max(axis=2) > threshold).astype(np.uint8)
    # Drop a border margin: warp padding + the shifted field of view make
    # the image edges unreliable witnesses.
    margin = max(tolerance_px + 2, int(abs(dx)) + 2, int(abs(dy)) + 2)
    mask[:margin, :] = 0
    mask[-margin:, :] = 0
    mask[:, :margin] = 0
    mask[:, -margin:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return mask


def changed_regions(mask: np.ndarray, min_area_px: int = 400,
                    min_height_px: int = 15,
                    max_aspect: float = 6.0) -> list[dict]:
    """Connected changed regions large enough to matter, as
    {'bbox': (x, y, w, h), 'area_px': int} sorted by area, biggest first.

    Shape filter: a standing object subtends tens of pixels of height even
    at max detection range (a 0.5 m object at 4 m is ~40 px), so thin
    horizontal slivers — floor/wall boundary parallax residue that survives
    the tolerance band — are rejected by min height and aspect ratio."""
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask)
    regions = []
    for index in range(1, count):
        x, y, w, h, area = stats[index]
        if area < min_area_px or h < min_height_px or w / max(h, 1) > max_aspect:
            continue
        regions.append({'bbox': (int(x), int(y), int(w), int(h)),
                        'area_px': int(area)})
    regions.sort(key=lambda r: -r['area_px'])
    return regions


def ground_point(region: dict, camera: CameraModel,
                 robot_pose: tuple[float, float, float],
                 max_range: float = 4.0) -> dict | None:
    """Map-frame floor position of a changed region.

    The region's bottom-center pixel is where the new object meets the
    floor; the ray through that pixel intersected with the floor plane
    gives range — valid for objects standing on the ground with a visible
    base (the inspection-room case). Returns None when the geometry is
    degenerate (bottom at/above the horizon ⇒ no floor intersection)."""
    x, y, w, h = region['bbox']
    u = x + w / 2.0
    v = float(y + h)  # bottom edge
    if v <= camera.cy + 2:      # at/above horizon: ray never hits the floor
        return None
    forward = camera.mount_z * camera.fy / (v - camera.cy)
    if forward > max_range or forward <= 0.05:
        return None
    lateral = -forward * (u - camera.cx) / camera.fx
    bx = forward + camera.mount_x          # robot frame, x forward y left
    by = lateral
    rx, ry, ryaw = robot_pose
    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
    map_x = rx + bx * cos_y - by * sin_y
    map_y = ry + bx * sin_y + by * cos_y
    extent = w / camera.fx * forward       # physical width estimate
    return {
        'x': round(float(map_x), 3),
        'y': round(float(map_y), 3),
        'range': round(float(forward), 3),
        'extent': round(float(extent), 3),
        'bbox': list(region['bbox']),
        'area_px': region['area_px'],
    }


def detect_changes(baseline_path: str | Path, current_path: str | Path,
                   robot_pose: tuple[float, float, float],
                   camera: CameraModel | None = None,
                   threshold: int = 35, tolerance_px: int = 7,
                   min_area_px: int = 400, max_range: float = 4.0) -> dict:
    """Full pipeline: baseline photo + current photo + robot pose at capture
    -> {'status', 'anomalies': [{'x','y','range','extent','bbox',...}]}."""
    camera = camera or CameraModel()
    baseline = load_image(baseline_path)
    current = load_image(current_path)
    if baseline.shape != current.shape:
        return {'status': 'baseline_shape_mismatch', 'anomalies': []}
    mask = diff_mask(baseline, current, threshold, tolerance_px)
    anomalies = []
    for region in changed_regions(mask, min_area_px):
        point = ground_point(region, camera, robot_pose, max_range)
        if point is not None:
            anomalies.append(point)
    return {'status': 'checked', 'anomalies': anomalies}


def merge_photo_detections(existing: list[dict], new: list[dict],
                           link_dist: float = 0.45) -> list[dict]:
    """Cross-yaw / cross-stop dedup: the same object seen from two photos
    yields two nearby estimates; keep the closer-range one (less projection
    error)."""
    merged = list(existing)
    for candidate in new:
        for index, kept in enumerate(merged):
            if math.hypot(candidate['x'] - kept['x'],
                          candidate['y'] - kept['y']) < link_dist:
                if candidate.get('range', 9e9) < kept.get('range', 9e9):
                    merged[index] = candidate
                break
        else:
            merged.append(candidate)
    return merged
