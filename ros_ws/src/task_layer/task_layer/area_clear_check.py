#!/usr/bin/env python3
"""area_clear check: laser-vs-map differencing (ADR-1; plan C5/P1-4).

Endpoint method: a beam endpoint only counts as anomaly evidence when it
lands in DEEP free space — a map cell that is free AND at least
`clearance_m` away from every occupied/unknown cell. Beams grazing walls
(the classic false-positive source) are rejected by construction, and the
test tolerates small localization error without comparing ranges.

Confirmation requires the same cell to be hit in `min_hit_frames` of the
scans collected while the robot stands still; clusters then need
`min_cluster_cells` cells (5: a transient 3-cell ghost appeared once in 24
empty-map stops, while a 0.45 m box yields 20+ cells nearby and 6+ at max
range), with a generous `cluster_link_m` so one object cannot split into
several. Detections within `peer_radius_m` of another robot's pose are
discarded (robots see each other on lidar).
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml


def load_pgm(path) -> np.ndarray:
    data = Path(path).read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:
        if data[i:i + 1].isspace():
            i += 1
            continue
        if data[i:i + 1] == b'#':
            i = data.find(b'\n', i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        raise ValueError('expected binary PGM (P5), got %r' % tokens[0])
    width, height = int(tokens[1]), int(tokens[2])
    i += 1  # single whitespace byte after maxval, then the raster
    img = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=i)
    return img.reshape(height, width)


def pose_uncertain(covariance, max_std_m: float = 0.12) -> bool:
    """True when AMCL xy std-dev exceeds the trust threshold (skip the check
    rather than report anomalies from a misplaced viewpoint)."""
    return math.sqrt(max(covariance[0], covariance[7], 0.0)) > max_std_m


def merge_detections(existing: list[dict], new: list[dict],
                     merge_radius_m: float = 0.5) -> list[dict]:
    """Cross-viewpoint dedup: the same object seen from two inspection
    points must stay ONE anomaly. Keeps the richer (more cells) view."""
    merged = list(existing)
    for det in new:
        twin = next((e for e in merged if math.hypot(
            e['x'] - det['x'], e['y'] - det['y']) <= merge_radius_m), None)
        if twin is None:
            merged.append(det)
        elif det['cells'] > twin['cells']:
            merged[merged.index(twin)] = det
    return merged


class AreaClearChecker:
    def __init__(self, map_yaml_path, clearance_m: float = 0.30,
                 cluster_link_m: float = 0.30, min_cluster_cells: int = 5,
                 min_hit_frames: int = 4, peer_radius_m: float = 0.35,
                 lidar_offset_x: float = -0.032, max_evidence_cells: int = 800,
                 max_detect_range_m: float = 2.5):
        self.cluster_link_m = cluster_link_m
        self.min_cluster_cells = min_cluster_cells
        self.min_hit_frames = min_hit_frames
        self.peer_radius_m = peer_radius_m
        self.lidar_offset_x = lidar_offset_x
        self.max_evidence_cells = max_evidence_cells
        # Angular pose error displaces a return by error*range: every false
        # positive observed in calibration sat at 2.75-3.2 m while the real
        # box was detected at 1.2 m. Evidence is only taken inside this
        # radius; alignment_ratio still uses the full scan.
        self.max_detect_range_m = max_detect_range_m

        with open(map_yaml_path, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        img = load_pgm(Path(map_yaml_path).parent / meta['image'])
        self.res = float(meta['resolution'])
        self.ox = float(meta['origin'][0])
        self.oy = float(meta['origin'][1])
        free_th = float(meta.get('free_thresh', 0.196))
        occ = img / 255.0 if int(meta.get('negate', 0)) else (255 - img) / 255.0
        occ = np.flipud(occ)  # PGM row 0 is max-y; flip so row index == y
        free = occ < free_th
        # Deep-free mask: erode free space by `clearance_m` of blocked
        # (occupied OR unknown) cells, chebyshev metric.
        blocked = ~free
        dilated = blocked.copy()
        for _ in range(max(1, math.ceil(clearance_m / self.res))):
            grown = dilated.copy()
            grown[1:, :] |= dilated[:-1, :]
            grown[:-1, :] |= dilated[1:, :]
            grown[:, 1:] |= dilated[:, :-1]
            grown[:, :-1] |= dilated[:, 1:]
            grown[1:, 1:] |= dilated[:-1, :-1]
            grown[1:, :-1] |= dilated[:-1, 1:]
            grown[:-1, 1:] |= dilated[1:, :-1]
            grown[:-1, :-1] |= dilated[1:, 1:]
            dilated = grown
        self.deep_free = free & ~dilated
        # "Near a mapped obstacle" mask for the alignment ratio: blocked
        # dilated by 2 cells (0.1 m) — where well-localized returns land.
        near = blocked.copy()
        for _ in range(2):
            grown = near.copy()
            grown[1:, :] |= near[:-1, :]
            grown[:-1, :] |= near[1:, :]
            grown[:, 1:] |= near[:, :-1]
            grown[:, :-1] |= near[:, 1:]
            grown[1:, 1:] |= near[:-1, :-1]
            grown[1:, :-1] |= near[:-1, 1:]
            grown[:-1, 1:] |= near[1:, :-1]
            grown[:-1, :-1] |= near[1:, 1:]
            near = grown
        self.near_blocked = near

    def alignment_ratio(self, scan, pose) -> float:
        """Fraction of in-range returns landing on/near mapped obstacles.
        A misprojected pose (AMCL yaw/position bias) rotates wall returns
        into free space and this ratio collapses — gate on it instead of
        trusting covariance alone."""
        x0, y0, yaw = pose
        lx = x0 + self.lidar_offset_x * math.cos(yaw)
        ly = y0 + self.lidar_offset_x * math.sin(yaw)
        rows, cols = self.near_blocked.shape
        matched = total = 0
        angle = float(scan.angle_min)
        for r in scan.ranges:
            beam_angle = yaw + angle
            angle += scan.angle_increment
            if not math.isfinite(r) or not (scan.range_min < r < scan.range_max):
                continue
            total += 1
            col = int((lx + r * math.cos(beam_angle) - self.ox) / self.res)
            row = int((ly + r * math.sin(beam_angle) - self.oy) / self.res)
            if 0 <= row < rows and 0 <= col < cols and self.near_blocked[row, col]:
                matched += 1
        return matched / total if total else 0.0

    def check(self, scans, pose, peers=()) -> dict:
        """scans: LaserScan-like objects taken while stationary;
        pose: (x, y, yaw) of base in map frame; peers: [(x, y), ...].
        Returns {'anomalies': [{x, y, cells, extent}], 'evidence_cells',
        'frames', 'truncated'}."""
        x0, y0, yaw = pose
        lx = x0 + self.lidar_offset_x * math.cos(yaw)
        ly = y0 + self.lidar_offset_x * math.sin(yaw)
        rows, cols = self.deep_free.shape
        hit_frames: dict[tuple, int] = {}
        for scan in scans:
            cells = set()
            angle = float(scan.angle_min)
            for r in scan.ranges:
                beam_angle = yaw + angle
                angle += scan.angle_increment
                if not math.isfinite(r) or not (scan.range_min < r < scan.range_max):
                    continue
                if r > self.max_detect_range_m:
                    continue
                wx = lx + r * math.cos(beam_angle)
                wy = ly + r * math.sin(beam_angle)
                col = int((wx - self.ox) / self.res)
                row = int((wy - self.oy) / self.res)
                if 0 <= row < rows and 0 <= col < cols and self.deep_free[row, col]:
                    cells.add((row, col))
            for cell in cells:
                hit_frames[cell] = hit_frames.get(cell, 0) + 1
        confirmed = [c for c, n in hit_frames.items() if n >= self.min_hit_frames]
        truncated = len(confirmed) > self.max_evidence_cells
        confirmed = confirmed[:self.max_evidence_cells]
        points = [((c[1] + 0.5) * self.res + self.ox,
                   (c[0] + 0.5) * self.res + self.oy) for c in confirmed]
        anomalies = []
        for cluster in self._cluster(points):
            if len(cluster) < self.min_cluster_cells:
                continue
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
            if any(math.hypot(cx - px, cy - py) <= self.peer_radius_m
                   for px, py in peers):
                continue
            anomalies.append({
                'x': round(cx, 3), 'y': round(cy, 3), 'cells': len(cluster),
                'extent': round(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 3),
            })
        return {'anomalies': anomalies, 'evidence_cells': len(confirmed),
                'frames': len(scans), 'truncated': truncated}

    def _cluster(self, points: list[tuple]) -> list[list[tuple]]:
        link_sq = self.cluster_link_m ** 2
        unvisited = set(range(len(points)))
        clusters = []
        while unvisited:
            seed = unvisited.pop()
            group, queue = [seed], [seed]
            while queue:
                i = queue.pop()
                near = [j for j in unvisited
                        if (points[i][0] - points[j][0]) ** 2
                        + (points[i][1] - points[j][1]) ** 2 <= link_sq]
                for j in near:
                    unvisited.remove(j)
                    group.append(j)
                    queue.append(j)
            clusters.append([points[i] for i in group])
        return clusters
