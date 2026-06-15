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


def rotation_aligned(baseline: np.ndarray, camera: CameraModel,
                     yaw_offset: float) -> tuple[np.ndarray, np.ndarray]:
    """Warp the baseline photo into the current camera orientation.

    Nav2's yaw goal tolerance lets two visits to the 'same' stop face up to
    tens of degrees apart — far beyond what translation search can absorb.
    For a pure rotation about the camera center the compensation is exact:
    H = K R K^-1. Returns (warped baseline, validity mask) — pixels warped
    in from outside the baseline's field of view are unknown, not evidence.
    """
    height, width = baseline.shape[:2]
    if abs(yaw_offset) < 1e-3:
        return baseline, np.full((height, width), 255, np.uint8)
    intrinsics = np.array([[camera.fx, 0, camera.cx],
                           [0, camera.fy, camera.cy],
                           [0, 0, 1]], dtype=np.float64)
    sin_a, cos_a = math.sin(yaw_offset), math.cos(yaw_offset)
    # Camera frame: x right, y down, z forward. A robot yaw of +a (CCW,
    # left turn) rotates the camera about its -y axis; the sign below is
    # pinned by test_rotation_alignment in the gates script.
    rotation = np.array([[cos_a, 0, sin_a],
                         [0, 1, 0],
                         [-sin_a, 0, cos_a]], dtype=np.float64)
    homography = intrinsics @ rotation @ np.linalg.inv(intrinsics)
    warped = cv2.warpPerspective(
        baseline, homography, (width, height), borderMode=cv2.BORDER_REPLICATE)
    valid = cv2.warpPerspective(
        np.full((height, width), 255, np.uint8), homography, (width, height),
        borderValue=0)
    return warped, valid


def estimate_yaw_offset(baseline: np.ndarray, current: np.ndarray,
                        camera: CameraModel, init: float = 0.0,
                        search: float = 0.25) -> float:
    """Heading difference between the two captures, measured from the
    images themselves.

    The AMCL yaw recorded at capture time cannot be trusted for this: the
    rotate-to-heading goal overshoots and the filter's yaw belief lags
    physical heading by up to ~0.5 rad right after the spin (observed:
    photos 35 deg apart whose recorded poses claimed 8 deg). A coarse-to-
    fine search over rotation-homography warps, scored by mean absolute
    difference on downsampled grayscale, recovers the true offset; `init`
    (from the recorded poses) just centers the search window."""
    base_small = cv2.resize(_gray(baseline), (160, 120))
    cur_small = cv2.resize(_gray(current), (160, 120)).astype(np.int16)
    small_cam = CameraModel(fx=camera.fx / 4, fy=camera.fy / 4,
                            cx=camera.cx / 4, cy=camera.cy / 4,
                            width=160, height=120,
                            mount_x=camera.mount_x, mount_z=camera.mount_z)

    def inliers(angle: float) -> float:
        """Alignment quality = fraction of overlap pixels that MATCH.

        Mean-error scoring lets a large new object hijack the alignment
        (its mismatch varies with angle, creating a false minimum), and
        edge-only scoring degenerates on this map's texture-poor walls.
        Inlier counting has neither failure: anomaly pixels are outliers
        at EVERY angle (a constant offset that cannot move the argmax),
        and featureless walls are inliers at every angle (a plateau the
        tie-break below resolves toward the pose-derived initial guess)."""
        warped, valid = rotation_aligned(
            cv2.cvtColor(base_small, cv2.COLOR_GRAY2BGR), small_cam, angle)
        mask = valid > 0
        overlap = int(mask.sum())
        if overlap < 1000:      # barely any overlap: useless alignment
            return -1.0
        diff = np.abs(_gray(warped).astype(np.int16) - cur_small)
        # Overlap-normalized inliers + the TIGHT search window around the
        # laser-corrected init (see corrected_capture_yaw): a wide window
        # would let a large anomaly be 'solved' by rotating it out of the
        # overlap region entirely (observed +0.4 rad hijack), and full-
        # frame normalization instead breaks texture-poor rooms. The
        # window keeps both failure modes unreachable.
        return float(((diff < 20) & mask).sum()) / overlap

    best = init
    for step, span in ((0.08, search), (0.02, 0.1), (0.005, 0.025)):
        candidates = np.arange(best - span, best + span + 1e-9, step)
        scored = [(inliers(float(a)), float(a)) for a in candidates]
        top = max(s for s, _ in scored)
        # Plateau tie-break: among near-best alignments take the one
        # closest to the initial guess, not an arbitrary plateau edge.
        plateau = [a for s, a in scored if s >= top - 0.005]
        best = min(plateau, key=lambda a: abs(a - init))
    return float(best)


def diff_mask(baseline: np.ndarray, current: np.ndarray,
              threshold: int = 35, tolerance_px: int = 7,
              yaw_offset: float = 0.0,
              camera: CameraModel | None = None) -> np.ndarray:
    """Binary mask of pixels in `current` that cannot be explained by the
    baseline even allowing a small local wobble.

    Alignment is two-stage: the known heading difference between captures
    is removed exactly via rotation homography, then phase correlation
    mops up the residual translation. The aligned baseline is expanded
    into a per-pixel [min, max] band over a (2*tolerance_px+1)^2
    neighborhood: edges that merely moved a few pixels (pose error) stay
    inside the band, genuinely new surfaces fall outside it."""
    valid = np.full(baseline.shape[:2], 255, np.uint8)
    if yaw_offset and camera is not None:
        baseline, valid = rotation_aligned(baseline, camera, yaw_offset)
    dx, dy = estimate_shift(baseline, current)
    # Reject a spurious phase-correlation peak. Rotation alignment already
    # removed the heading difference, so the residual translation is the
    # sub-degree pointing error plus a few cm of position — a handful of
    # pixels. A large reported shift means correlation locked onto the
    # rotated image's replicated invalid border instead (observed: a bogus
    # -120 px dy that dragged the wall-floor boundary into a huge false
    # band); trust the rotation and drop it.
    max_shift = 12.0
    if abs(dx) > max_shift or abs(dy) > max_shift:
        dx = dy = 0.0
    matrix = np.float32([[1, 0, dx], [0, 1, dy]])
    aligned = cv2.warpAffine(
        baseline, matrix, (baseline.shape[1], baseline.shape[0]),
        borderMode=cv2.BORDER_REPLICATE)
    valid = cv2.warpAffine(valid, matrix,
                           (baseline.shape[1], baseline.shape[0]),
                           borderValue=0)
    kernel = np.ones((2 * tolerance_px + 1, 2 * tolerance_px + 1), np.uint8)
    low = cv2.erode(aligned, kernel)
    high = cv2.dilate(aligned, kernel)
    outside = np.maximum(low.astype(np.int16) - current.astype(np.int16),
                         current.astype(np.int16) - high.astype(np.int16))
    mask = (outside.max(axis=2) > threshold).astype(np.uint8)
    # Baseline content warped in from outside the recorded field of view
    # is unknown, not evidence of change; erode the validity edge too.
    valid = cv2.erode(valid, kernel)
    mask[valid < 255] = 0
    # Drop a border margin: warp padding + the shifted field of view make
    # the image edges unreliable witnesses.
    margin = max(tolerance_px + 2, int(abs(dx)) + 2, int(abs(dy)) + 2)
    mask[:margin, :] = 0
    mask[-margin:, :] = 0
    mask[:, :margin] = 0
    mask[:, -margin:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return mask


def changed_regions(mask: np.ndarray, min_area_px: int = 1500,
                    min_height_px: int = 15,
                    max_aspect: float = 6.0) -> list[dict]:
    """Connected changed regions large enough to matter, as
    {'bbox': (x, y, w, h), 'area_px': int} sorted by area, biggest first.

    Shape filter: a standing object subtends tens of pixels of height even
    at max detection range (a 0.5 m object at 4 m is ~40 px), so thin
    horizontal slivers — floor/wall boundary parallax residue that survives
    the tolerance band — are rejected by min height and aspect ratio.

    The min area is calibrated against a survey of every detection across
    four full-map runs: real 0.45 m boxes never projected below 2700 px
    (the box face fills the frame), while the largest residual artifact
    that survived bounds-clip and merging was 667 px — a clean 4x gap, so
    1500 px drops artifacts with margin to spare. For the Final phase's
    small graspable objects this floor must come down with the vision
    route that replaces this scenario."""
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
                 max_range: float = 3.5, min_range: float = 0.3,
                 max_height_m: float = 1.8) -> dict | None:
    """Map-frame floor position of a changed region.

    The region's bottom-center pixel is where the new object meets the
    floor; the ray through that pixel intersected with the floor plane
    gives range — valid for objects standing on the ground with a visible
    base (the inspection-room case). Returns None when the geometry is
    degenerate (bottom at/above the horizon ⇒ no floor intersection) or
    when the region fails physical sanity:
    - closer than min_range: a station offset of centimetres makes near
      geometry (door frames at an entrance viewpoint) parallax by dozens
      of pixels, and a robot cannot stand 0.5 m from a real new object
      anyway — the costmap would have stopped it;
    - implied physical height above max_height_m: a doorway pillar that
      slid out of view projects as a '2 m tall object' deep in the room,
      taller than anything in this site's anomaly universe."""
    x, y, w, h = region['bbox']
    u = x + w / 2.0
    v = float(y + h)  # bottom edge
    if v <= camera.cy + 2:      # at/above horizon: ray never hits the floor
        return None
    forward = camera.mount_z * camera.fy / (v - camera.cy)
    if forward > max_range or forward < min_range:
        return None
    top_above_horizon = camera.cy - float(y)
    if top_above_horizon > 0:
        implied_height = (camera.mount_z
                          + top_above_horizon * forward / camera.fy)
        if implied_height > max_height_m:
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
                   min_area_px: int = 1500, max_range: float = 3.5,
                   baseline_pose: tuple[float, float, float] | None = None,
                   min_range: float = 0.3) -> dict:
    """Full pipeline: baseline photo + current photo + robot pose at capture
    -> {'status', 'anomalies': [{'x','y','range','extent','bbox',...}]}.

    baseline_pose is the pose the baseline photo was captured from; when
    given, the heading difference is compensated exactly (Nav2's yaw goal
    tolerance makes revisit headings differ by up to tens of degrees)."""
    camera = camera or CameraModel()
    baseline = load_image(baseline_path)
    current = load_image(current_path)
    if baseline.shape != current.shape:
        return {'status': 'baseline_shape_mismatch', 'anomalies': []}
    pose_for_projection = robot_pose
    yaw_offset = 0.0
    mask = diff_mask(baseline, current, threshold, tolerance_px)
    if baseline_pose is not None:
        init = math.atan2(
            math.sin(robot_pose[2] - baseline_pose[2]),
            math.cos(robot_pose[2] - baseline_pose[2]))
        visual = estimate_yaw_offset(baseline, current, camera, init)
        # Pick the alignment by its END PRODUCT, from a small discrete
        # candidate set: the recorded-pose offset, the visually refined
        # offset, and zero. The inlier metric cannot resolve below ~0.05
        # rad on texture-poor walls while the tolerance band only absorbs
        # ~0.02 rad, so a continuously optimized angle can itself paint a
        # large false mask. Whichever candidate explains the scene best
        # (smallest changed area) wins — 'nothing changed' beats every
        # misalignment artifact, and a real object survives all three
        # candidates because none of them can warp it away.
        best_count = int(mask.sum())
        for candidate in (init, visual):
            if abs(candidate) < 1e-4:
                continue
            trial = diff_mask(baseline, current, threshold, tolerance_px,
                              yaw_offset=candidate, camera=camera)
            count = int(trial.sum())
            if count < best_count:
                best_count, mask, yaw_offset = count, trial, candidate
        # The freshly-spun AMCL yaw lags physical heading; the baseline yaw
        # (a settled recording) plus the chosen offset is the better
        # estimate of where the camera actually pointed.
        pose_for_projection = (robot_pose[0], robot_pose[1],
                               baseline_pose[2] + yaw_offset)
    anomalies = []
    for region in changed_regions(mask, min_area_px):
        point = ground_point(region, camera, pose_for_projection, max_range,
                             min_range=min_range)
        if point is not None:
            anomalies.append(point)
    return {'status': 'checked', 'anomalies': anomalies,
            'yaw_offset': round(yaw_offset, 4)}


def merge_photo_detections(existing: list[dict], new: list[dict],
                           link_dist: float = 1.4) -> list[dict]:
    """Cross-yaw / cross-stop dedup: the same object seen from two photos
    yields two nearby estimates; keep the closer-range one (less projection
    error). The link distance covers the long-range projection scatter of
    one object seen from several yaws (bottom-edge pixel error grows with
    range; observed 1.36 m spread on a box at 1.7-1.9 m). Trade-off, by
    design: two DISTINCT objects closer than this merge into one report —
    acceptable for this site's one-object-per-room anomaly scenarios."""
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
