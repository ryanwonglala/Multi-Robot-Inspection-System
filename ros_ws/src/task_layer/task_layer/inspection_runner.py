#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from task_layer.area_clear_check import AreaClearChecker
from task_layer.photo_diff_check import (
    CameraModel,
    detect_changes,
    merge_photo_detections,
)
from task_layer.report_writer import default_report_dir, write_report
from task_layer.scan_analyzer import aggregate_scan_summaries, summarize_scan


STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


def default_world_model_path() -> str:
    share_dir = get_package_share_directory('task_layer')
    return str(Path(share_dir) / 'config' / 'world_model.yaml')


def yaw_to_quaternion(yaw: float) -> dict:
    half = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


def normalize_text(value: str) -> str:
    return value.strip().lower().replace(' ', '_').replace('-', '_')


def safe_path_name(value: str) -> str:
    return ''.join(char if char.isalnum() or char in {'_', '-'} else '_' for char in value)


class InspectionRunner(Node):
    def __init__(self):
        super().__init__('inspection_runner')
        self.declare_parameter('world_model_path', default_world_model_path())
        self.declare_parameter('target', '')
        self.declare_parameter('route', '')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('server_timeout_sec', 10.0)
        self.declare_parameter('candidate_offset', 0.5)
        self.declare_parameter('candidate_spread_ratio', 0.35)
        self.declare_parameter('bounds_margin', 0.25)
        self.declare_parameter('max_candidate_attempts_per_area', 4)
        self.declare_parameter('capture_nav_fail_evidence', True)
        # Six headings, 60 deg apart: the rotation-alignment step of photo
        # diff crops up to ~20 deg off one image edge (heading overshoot
        # between baseline and revisit), and four 90 deg-spaced photos then
        # leave coverage seams an off-axis object can hide in (observed:
        # box at the yaw0/yaw1 seam of server_room went undetected).
        self.declare_parameter('scan_yaws', [0.0, 1.0472, 2.0944, 3.1416,
                                             -2.0944, -1.0472])
        self.declare_parameter('scan_settle_sec', 1.0)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_settle_sec', 1.0)
        self.declare_parameter('report_dir', default_report_dir())
        self.declare_parameter('return_home', True)
        self.declare_parameter('home_area', 'charging_station')
        # Per-robot home override (multi-robot: each robot has its own dock;
        # the world_model robot_start is a single-robot legacy default).
        self.declare_parameter('home_x', float('nan'))
        self.declare_parameter('home_y', float('nan'))
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('return_home_standoff_distance', 0.0)
        self.declare_parameter('dry_run', False)
        # --- Navigation resilience (B-axis): never collide/hug/deadlock; a
        # blocked area is skipped gracefully so the mission always completes.
        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('static_map_topic', 'map')
        # Only TRUE lethal cells (LETHAL_OBSTACLE -> 100 in the OccupancyGrid)
        # count: inscribed (99) and the inflation gradient bleed into FREE
        # static-map cells around walls, so a lower threshold would falsely flag
        # legitimate wall-adjacent goals. 100 keeps only an obstacle's actual
        # footprint (and wall cells, which the static-map check then excludes).
        self.declare_parameter('costmap_lethal_cost', 100)
        # Static-map occupancy below this (and >= 0) is "free floor". Used to
        # tell a dynamic/unmapped obstacle (lethal in costmap, free on the map)
        # from a static wall (lethal in costmap, occupied on the map).
        self.declare_parameter('static_free_max', 50)
        self.declare_parameter('candidate_clearance_radius', 0.22)
        self.declare_parameter('nav_goal_timeout_sec', 120.0)
        # --- P1-5v photo-diff anomaly detection (OLD validated config) ---
        # Photo-diff anomaly detection: compare each inspection photo against
        # the baseline photo recorded from the same stop/yaw when the scene
        # was clean. baseline_record:=true turns a run into the baseline
        # patrol that produces that library.
        self.declare_parameter('detect_photo_diff', True)
        self.declare_parameter('baseline_record', False)
        self.declare_parameter('baseline_dir', str(
            Path.home() / 'roboinspec_ws' / 'baselines'))
        self.declare_parameter('photo_diff_threshold', 35)
        self.declare_parameter('photo_diff_tolerance_px', 7)
        # 1500 px floor: real 0.45 m boxes never projected below 2700 px
        # across the rehearsals, the largest surviving artifact was 667 px.
        self.declare_parameter('photo_diff_min_area_px', 1500)
        # Beyond ~3.5 m the ground-intersection geometry degrades (a few
        # pixels of bottom-edge error swing the estimate by metres) and the
        # only regions that big are alignment artifacts.
        self.declare_parameter('photo_diff_max_range', 3.5)
        self.declare_parameter('photo_diff_min_range', 0.3)
        # merge_distance: covers the long-range projection scatter of one
        # object seen from several yaws (observed 1.36 m spread on a box
        # at 1.7-1.9 m).
        self.declare_parameter('photo_diff_merge_distance', 1.4)
        # Camera mount in the base frame; keep in sync with the camera link
        # pose in sim/models/turtlebot3_burger_cam_ns/model.sdf (and with
        # the real robot's measured mount before field runs).
        self.declare_parameter('camera_mount_x', 0.076)
        self.declare_parameter('camera_mount_z', 0.250)
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('map_yaml', str(
            Path(get_package_share_directory('task_layer')) / 'maps' / 'tb3_map.yaml'))
        self.declare_parameter('robots_yaml', str(
            Path(get_package_share_directory('task_layer')) / 'config' / 'robots.yaml'))
        self.declare_parameter('detect_bounds_margin', 0.30)
        # photo_detect_clip_bounds: default True; gate areas that exist to
        # photograph INTO a room opt out via photo_detect_clip_bounds: false
        # in world_model.
        self.declare_parameter('photo_detect_clip_bounds', True)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._latest_scan = None
        self._latest_image = None
        self._latest_costmap = None
        self._latest_static_map = None
        self._camera_info = None
        self._own_pose = None
        self._run_dir = None
        self._anomaly_seq = 0
        self._yaw_corrector = None  # lazy: loads the static map on first use
        self.robot_name = self.get_namespace().strip('/') or 'robot'
        # Set True once a nav goal cannot be confirmed terminal (cancel not
        # confirmed, or a send timed out with a possibly-pending request). While
        # set, NO further nav goal is dispatched -- commanding a new goal while a
        # previous one may still be live server-side is unsafe (preempt race).
        self._nav_aborted = False
        scan_topic = self.get_parameter('scan_topic').value
        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self._camera_info = None
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._camera_info_callback, 10)
        amcl_qos = QoSProfile(depth=1)
        amcl_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        amcl_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self._amcl_pose_callback, amcl_qos)
        # Peer belief poses: a teammate caught in an inspection photo would
        # otherwise diff against the (empty) baseline as an anomaly.
        self._peer_poses: dict[str, tuple[float, float]] = {}
        for peer, topic in self._peer_pose_topics().items():
            self.create_subscription(
                PoseWithCovarianceStamped, topic,
                lambda msg, name=peer: self._on_peer_pose(name, msg),
                amcl_qos)
        # The global costmap is latched (transient_local); QoS must match or
        # the subscription receives nothing.
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        costmap_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, self.get_parameter('costmap_topic').value,
            self._costmap_callback, costmap_qos)
        # Static map (also latched) -- lets candidate_is_clear separate dynamic
        # obstacles from the known walls baked into the costmap.
        self.create_subscription(
            OccupancyGrid, self.get_parameter('static_map_topic').value,
            self._static_map_callback, costmap_qos)
        latched = QoSProfile(depth=10)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        # Fleet-wide buses, deliberately absolute (one shared channel for all
        # robots; latched so the GUI/allocator and a late RViz still see them).
        self._event_pub = self.create_publisher(String, '/anomaly_events', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '/anomaly_markers', latched)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    # ------------------------------------------------------------------
    # Peer tracking helpers
    # ------------------------------------------------------------------

    def _peer_pose_topics(self) -> dict[str, str]:
        try:
            path = Path(str(self.get_parameter('robots_yaml').value)).expanduser()
            with path.open(encoding='utf-8') as f:
                robots = (yaml.safe_load(f) or {}).get('robots', {})
        except Exception:  # noqa: BLE001  (no registry: single-robot run)
            return {}
        return {name: info['amcl_pose_topic']
                for name, info in robots.items()
                if name != self.robot_name and info.get('amcl_pose_topic')}

    def _on_peer_pose(self, name: str, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self._peer_poses[name] = (p.x, p.y)

    def near_peer(self, x: float, y: float, radius: float = 0.9) -> bool:
        """Is (x, y) plausibly the teammate? The radius covers the error
        budget of comparing a camera-projected sighting against the peer's
        own AMCL belief: both robots' localization error plus the ground-
        intersection projection error (a transiting robot photographed
        mid-motion landed 0.5-0.8 m from its believed pose in rehearsal).
        Real anomalies parked within 0.9 m of a robot are accepted losses —
        and transient: the next pass without the peer nearby reports them."""
        return any(math.hypot(x - px, y - py) <= radius
                   for px, py in self._peer_poses.values())

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

    def _image_callback(self, msg: Image):
        self._latest_image = msg

    def _camera_info_callback(self, msg: CameraInfo):
        self._camera_info = msg

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self._own_pose = (
            float(pose.position.x),
            float(pose.position.y),
            float(yaw),
        )

    def _costmap_callback(self, msg: OccupancyGrid):
        self._latest_costmap = msg

    def _static_map_callback(self, msg: OccupancyGrid):
        self._latest_static_map = msg

    # ------------------------------------------------------------------
    # Detection helpers (OLD P1-5v logic)
    # ------------------------------------------------------------------

    def corrected_capture_yaw(self, x: float, y: float,
                              believed_yaw: float) -> tuple[float, float]:
        """Heading at capture time, refined by matching the live laser scan
        against the static map.

        AMCL's yaw belief is transiently off by up to ~0.5 rad right after
        a rotation sequence (updates are motion-gated and convergence lags),
        which poisons both photo-diff alignment and anomaly projection. The
        laser is the right sensor for heading: 360 deg of wall structure,
        texture-independent, and alignment_ratio already excludes deep-free
        (anomaly) returns from its denominator, so a new object cannot bias
        the fit. The retired laser detector's map machinery does the work.
        Returns (corrected_yaw, ratio_at_best)."""
        scan = self._latest_scan
        if scan is None:
            return believed_yaw, 0.0
        if self._yaw_corrector is None:
            self._yaw_corrector = AreaClearChecker(
                str(self.get_parameter('map_yaml').value))

        def ratio(dyaw: float) -> float:
            return self._yaw_corrector.alignment_ratio(
                scan, (x, y, believed_yaw + dyaw))

        best = 0.0
        for step, span in ((0.04, 0.6), (0.008, 0.06)):
            candidates = [best + k * step
                          for k in range(-int(span / step), int(span / step) + 1)]
            scored = [(ratio(d), d) for d in candidates]
            top = max(s for s, _ in scored)
            # Plateau tie-break toward the believed heading.
            best = min((d for s, d in scored if s >= top - 0.01), key=abs)
        return believed_yaw + best, max(ratio(best), 0.0)

    def camera_model(self) -> CameraModel:
        """Intrinsics from the live camera_info when available (so a real
        camera swap needs no code change); mount pose always from params."""
        mount_x = float(self.get_parameter('camera_mount_x').value)
        mount_z = float(self.get_parameter('camera_mount_z').value)
        info = self._camera_info
        if info is None:
            return CameraModel(mount_x=mount_x, mount_z=mount_z)
        return CameraModel(
            fx=float(info.k[0]), fy=float(info.k[4]),
            cx=float(info.k[2]), cy=float(info.k[5]),
            width=int(info.width), height=int(info.height),
            mount_x=mount_x, mount_z=mount_z)

    def baseline_photo_path(self, area_key: str, stop_label: str,
                            yaw_index: int) -> Path:
        """Baseline library key. Shared across robots: both carry the same
        camera at the same mount (per-robot libraries become necessary only
        if the real mounts diverge)."""
        return (Path(str(self.get_parameter('baseline_dir').value)).expanduser()
                / safe_path_name(area_key) / safe_path_name(stop_label)
                / f'yaw{yaw_index:02d}.ppm')

    def detect_bounds(self, area: dict):
        """Bounds (shrunk by the detection margin) a detection must fall inside
        to count -- the doorway-leakage filter. Normally the inspected area's
        own bounds; but a doorway viewpoint that photographs INTO another area
        sets photo_detect_bounds_area to clip against THAT area's bounds. Its
        own 0.45 m strip is degenerate (margin inverts it -> everything clipped),
        while simply disabling the clip lets the near door-frame parallax project
        a phantom anomaly at the threshold (observed: clean FP at (7.564,3.222)).
        Clipping to the watched room keeps real in-room hits and drops the
        doorway phantom, which projects short of the room's near edge."""
        ref = area.get('photo_detect_bounds_area')
        if ref:
            world_model = getattr(self, '_world_model', None) or {}
            bounds = ((world_model.get('areas') or {}).get(ref) or {}).get('bounds') or {}
        else:
            bounds = area.get('bounds') or {}
        if not all(k in bounds for k in ('x_min', 'x_max', 'y_min', 'y_max')):
            return None
        margin = float(self.get_parameter('detect_bounds_margin').value)
        return (float(bounds['x_min']) + margin, float(bounds['y_min']) + margin,
                float(bounds['x_max']) - margin, float(bounds['y_max']) - margin)

    def process_photo_views(self, area_key: str,
                            scan_samples: list[dict]) -> dict:
        """Record clean views or compare captured views against their
        baselines (OLD P1-5v flat-layout baseline scheme).

        Contract (kept from revival's call site):
          Returns {'status', 'views', 'anomalies': [{x, y, ...}]}

        In baseline_record mode: archives photos under
          <baseline_dir>/<area_key>/<stop_label>/yaw{NN}.ppm
        with a .json sidecar pose.

        In detect mode: per-yaw photo diff with laser yaw correction,
        bounds clip, and peer exclusion.
        """
        # The area dict is passed in from inspect_area via _current_area so
        # photo_diff_stop can access bounds, photo_detect flag, etc.
        area = getattr(self, '_current_area', {})
        record = bool(self.get_parameter('baseline_record').value)
        outcome = {
            'status': 'pending',
            'views': [],
            'anomalies': [],
        }

        if not area.get('photo_detect', True) and not record:
            outcome['status'] = 'photo_detect_disabled'
            return outcome
        area_key_for_stop = area_key

        stop_label = 'stop'  # default single-stop label

        # Group samples by stop: each sample has a 'stop_label' key injected
        # by inspect_area, or we treat them all as one stop.
        stop_groups: dict[str, list[tuple[int, dict]]] = {}
        for sample in scan_samples:
            label = sample.get('stop_label', stop_label)
            yaw_index = sample.get('yaw_index', 0)
            stop_groups.setdefault(label, []).append((yaw_index, sample))

        all_found: list[dict] = []
        checked_total = 0
        for label, stop_samples in stop_groups.items():
            stop_dict = {'label': label,
                         'x': (stop_samples[0][1].get('pose_at_capture') or (0, 0, 0))[0],
                         'y': (stop_samples[0][1].get('pose_at_capture') or (0, 0, 0))[1]}
            stop_result = self.photo_diff_stop(
                area_key_for_stop, area, stop_dict, stop_samples)
            outcome['views'].extend(stop_result.get('views', []))
            all_found.extend(stop_result.get('anomalies', []))
            checked_total += int(stop_result.get('photos_checked', 0))

        # Cross-stop merge
        all_found = merge_photo_detections(
            [],
            all_found,
            link_dist=float(self.get_parameter('photo_diff_merge_distance').value))

        if record:
            outcome['status'] = 'baseline_recorded'
        elif checked_total > 0 or all_found:
            # A clean area still ran the diff (checked_total>0 photos compared,
            # zero anomalies) -- that IS 'checked', not 'no_baseline'. Only fall
            # to 'no_baseline' when not a single view could be compared.
            outcome['status'] = 'checked'
        else:
            outcome['status'] = 'no_baseline'

        outcome['photos_checked'] = checked_total
        outcome['anomalies'] = all_found
        return outcome

    def photo_diff_stop(self, area_key: str, area: dict, stop: dict,
                        stop_samples: list[tuple[int, dict]]) -> dict:
        """Per-stop photo handling: in baseline_record mode archive the
        photos as the clean reference; otherwise diff each photo against
        its baseline and return map-frame anomaly candidates."""
        record = bool(self.get_parameter('baseline_record').value)
        stop_label = str(stop.get('label', 'stop'))
        outcome: dict = {'stop': {'label': stop_label,
                                  'x': stop.get('x', 0.0),
                                  'y': stop.get('y', 0.0)},
                         'views': [],
                         'anomalies': []}
        if record:
            recorded = 0
            for yaw_index, sample in stop_samples:
                photo = (sample.get('image_capture') or {}).get('image_path')
                if not photo:
                    continue
                target = self.baseline_photo_path(area_key, stop_label, yaw_index)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(photo, target)
                # Capture pose sidecar: the diff stage compensates the
                # heading difference between baseline and revisit exactly,
                # so Nav2's loose yaw goal tolerance stops mattering.
                pose = sample.get('pose_at_capture') or (
                    stop.get('x', 0.0), stop.get('y', 0.0),
                    float(sample.get('yaw', 0.0)))
                target.with_suffix('.json').write_text(json.dumps(
                    {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}))
                recorded += 1
            outcome.update({'status': 'baseline_recorded', 'photos': recorded})
            self.get_logger().info(
                '%s/%s: %d baseline photo(s) recorded'
                % (area_key, stop_label, recorded))
            return outcome

        if not area.get('photo_detect', True):
            outcome.update({'status': 'photo_detect_disabled'})
            return outcome

        camera = self.camera_model()
        bounds = self.detect_bounds(area)
        clip = bool(area.get('photo_detect_clip_bounds',
                             self.get_parameter('photo_detect_clip_bounds').value))
        min_range = float(area.get(
            'photo_detect_min_range',
            self.get_parameter('photo_diff_min_range').value))
        checked = 0
        found: list[dict] = []
        for yaw_index, sample in stop_samples:
            photo = (sample.get('image_capture') or {}).get('image_path')
            base = self.baseline_photo_path(area_key, stop_label, yaw_index)
            if not photo or not base.exists():
                continue
            pose = sample.get('pose_at_capture') or (
                stop.get('x', 0.0), stop.get('y', 0.0),
                float(sample.get('yaw', 0.0)))
            base_pose = None
            base_meta = base.with_suffix('.json')
            if base_meta.exists():
                try:
                    meta = json.loads(base_meta.read_text())
                    base_pose = (meta['x'], meta['y'], meta['yaw'])
                except (ValueError, KeyError):
                    base_pose = None
            detection = detect_changes(
                base, photo, pose, camera,
                threshold=int(self.get_parameter('photo_diff_threshold').value),
                tolerance_px=int(self.get_parameter('photo_diff_tolerance_px').value),
                min_area_px=int(self.get_parameter('photo_diff_min_area_px').value),
                max_range=float(self.get_parameter('photo_diff_max_range').value),
                baseline_pose=base_pose, min_range=min_range)
            checked += 1
            for anomaly in detection['anomalies']:
                # Bounds clip keeps doorway-leaked sightings of NEIGHBOR
                # rooms out; gate areas that exist to photograph INTO a
                # room opt out via photo_detect_clip_bounds: false.
                if (clip and bounds is not None
                        and not (bounds[0] <= anomaly['x'] <= bounds[2]
                                 and bounds[1] <= anomaly['y'] <= bounds[3])):
                    continue
                if self.near_peer(anomaly['x'], anomaly['y']):
                    self.get_logger().info(
                        '%s: change at (%.2f, %.2f) matches a peer robot '
                        'pose, ignored' % (area_key, anomaly['x'], anomaly['y']))
                    continue
                anomaly['detected_from'] = {
                    'stop': stop_label, 'yaw_index': yaw_index,
                    'photo': photo}
                # Surface the evidence photo on the standard field too, so the
                # event JSON / GUI / report all carry it (not just detected_from).
                anomaly['evidence_photo'] = photo
                found.append(anomaly)
        outcome['status'] = 'checked' if checked else 'no_baseline'
        outcome['anomalies'] = merge_photo_detections([], found)
        outcome['photos_checked'] = checked
        return outcome

    def publish_anomaly(self, area_key: str, anomaly: dict, viewpoint: dict):
        self._anomaly_seq += 1
        stamp = datetime.now(timezone.utc).isoformat(timespec='seconds')
        event = {
            'robot': self.robot_name,
            'stamp': stamp,
            'area': area_key,
            'x': anomaly['x'],
            'y': anomaly['y'],
            'size': anomaly.get('extent'),
            'cells': anomaly.get('cells'),
            'evidence_photo': anomaly.get('evidence_photo'),
            'viewpoint': viewpoint,
        }
        self._event_pub.publish(String(data=json.dumps(event)))

        body = Marker()
        body.header.frame_id = 'map'
        body.ns = f'{self.robot_name}/anomalies'
        body.id = self._anomaly_seq
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = float(anomaly['x'])
        body.pose.position.y = float(anomaly['y'])
        body.pose.position.z = 0.25
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = 0.3
        body.scale.z = 0.5
        body.color.r, body.color.g, body.color.b, body.color.a = 1.0, 0.1, 0.1, 0.85
        label = Marker()
        label.header.frame_id = 'map'
        label.ns = f'{self.robot_name}/anomaly_labels'
        label.id = self._anomaly_seq
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(anomaly['x'])
        label.pose.position.y = float(anomaly['y'])
        label.pose.position.z = 0.75
        label.scale.z = 0.22
        # Red text: the map's free space renders white in RViz, so a white
        # label is invisible against it.
        label.color.r, label.color.g, label.color.b = 0.8, 0.0, 0.0
        label.color.a = 1.0
        label.text = f"{area_key} ({anomaly['x']:.2f}, {anomaly['y']:.2f})"
        self._marker_pub.publish(MarkerArray(markers=[body, label]))
        self.get_logger().warn(
            'ANOMALY %s in %s at (%.2f, %.2f) extent=%.2f'
            % (self._anomaly_seq, area_key, anomaly['x'], anomaly['y'],
               anomaly.get('extent') or 0.0))

    # ------------------------------------------------------------------
    # Navigation resilience (B-axis — PRESERVED BYTE-UNCHANGED in logic)
    # ------------------------------------------------------------------

    @staticmethod
    def _grid_value(grid: OccupancyGrid, x: float, y: float):
        """Occupancy/cost at world (x, y) in an OccupancyGrid, or None if the
        grid is missing or the point falls outside it."""
        if grid is None:
            return None
        info = grid.info
        if info.resolution <= 0.0:
            return None
        col = math.floor((x - info.origin.position.x) / info.resolution)
        row = math.floor((y - info.origin.position.y) / info.resolution)
        if 0 <= col < info.width and 0 <= row < info.height:
            return grid.data[row * info.width + col]
        return None

    def candidate_is_clear(self, x: float, y: float) -> bool:
        """False only when an UNMAPPED (dynamic) obstacle occupies the candidate
        footprint -- i.e. a costmap cell at lethal cost whose location is FREE
        on the static map.

        Static walls are lethal in the costmap too, but the static map marks
        them occupied, so they are deliberately NOT treated as blocking; their
        inflation gradient is excluded by the lethal>=100 threshold. This is
        what keeps legitimate wall-adjacent goals (the charging dock, doorway
        viewpoints) from being falsely cancelled. The False signal -- lethal
        cost over free static-map floor -- is exactly the unmapped-obstacle
        event the A-axis will publish as an anomaly; B only uses it to pick a
        reachable standoff pose.

        No costmap or no static map yet / out of map => optimistic True: let
        Nav2 try, with the per-goal timeout in send_goal_and_wait as the
        universal backstop."""
        grid = self._latest_costmap
        static = self._latest_static_map
        if grid is None or static is None:
            return True
        info = grid.info
        if info.resolution <= 0.0:
            return True
        lethal = int(self.get_parameter('costmap_lethal_cost').value)
        free_max = int(self.get_parameter('static_free_max').value)
        radius = float(self.get_parameter('candidate_clearance_radius').value)
        steps = max(0, int(radius / info.resolution))
        base_col = math.floor((x - info.origin.position.x) / info.resolution)
        base_row = math.floor((y - info.origin.position.y) / info.resolution)
        for d_row in range(-steps, steps + 1):
            for d_col in range(-steps, steps + 1):
                col = base_col + d_col
                row = base_row + d_row
                if not (0 <= col < info.width and 0 <= row < info.height):
                    continue
                if grid.data[row * info.width + col] < lethal:
                    continue
                # Lethal in the costmap. Only count it as a dynamic obstacle if
                # the static map does NOT explain it (free floor at that spot).
                wx = info.origin.position.x + (col + 0.5) * info.resolution
                wy = info.origin.position.y + (row + 0.5) * info.resolution
                static_val = self._grid_value(static, wx, wy)
                if static_val is not None and 0 <= static_val < free_max:
                    return False
        return True

    def load_world_model(self) -> dict:
        path = Path(self.get_parameter('world_model_path').value).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'world_model_path does not exist: {path}')
        with path.open('r', encoding='utf-8') as file:
            return yaml.safe_load(file) or {}

    def requested_targets(self) -> list[str]:
        route = str(self.get_parameter('route').value or '').strip()
        target = str(self.get_parameter('target').value or '').strip()
        if route:
            separators_normalized = route.replace(';', ',')
            return [item.strip() for item in separators_normalized.split(',') if item.strip()]
        if target:
            return [target]
        raise ValueError("Parameter 'target' or 'route' is required")

    def resolve_area(self, world_model: dict, target: str) -> tuple[str, dict]:
        query = target.strip()
        if not query:
            raise ValueError('Empty area target in route')

        areas = world_model.get('areas', {})
        if query in areas:
            return query, areas[query]

        normalized = normalize_text(query)
        for key, area in areas.items():
            names = {
                normalize_text(key),
                normalize_text(area.get('display_name', key)),
                normalize_text(area.get('marker_model', '')),
            }
            if normalized in names:
                return key, area

        known = ', '.join(sorted(areas))
        raise ValueError(f"Unknown target area '{target}'. Known areas: {known}")

    def generate_candidate_poses(self, area: dict) -> list[dict]:
        # Explicit viewpoints (world_model) override generation entirely:
        # doorway-type areas (restricted_gate) are too narrow for the ring
        # generator — bounds minus margin invert and every candidate dies —
        # and their semantics is "stand HERE, face THERE", which a generated
        # ring cannot express. The author's poses are trusted as-is.
        viewpoints = area.get('viewpoints')
        if viewpoints:
            return [{
                'label': f'viewpoint_{i}',
                'x': round(float(vp['x']), 3),
                'y': round(float(vp['y']), 3),
                'yaw': round(float(vp.get('yaw', 0.0)), 4),
            } for i, vp in enumerate(viewpoints, start=1)]
        center = area.get('center')
        if not center or len(center) < 2:
            raise ValueError('Selected area is missing center: [x, y]')
        cx = float(center[0])
        cy = float(center[1])
        min_offset = float(self.get_parameter('candidate_offset').value)
        spread_ratio = float(self.get_parameter('candidate_spread_ratio').value)
        bounds = area.get('bounds') or {}
        margin = float(self.get_parameter('bounds_margin').value)

        if all(key in bounds for key in ['x_min', 'x_max', 'y_min', 'y_max']):
            x_min = float(bounds['x_min']) + margin
            x_max = float(bounds['x_max']) - margin
            y_min = float(bounds['y_min']) + margin
            y_max = float(bounds['y_max']) - margin
            cx = min(max(cx, x_min), x_max)
            cy = min(max(cy, y_min), y_max)
            width = max(0.0, x_max - x_min)
            height = max(0.0, y_max - y_min)
            offset_x = min(max(min_offset, width * spread_ratio), width * 0.5)
            offset_y = min(max(min_offset, height * spread_ratio), height * 0.5)
        else:
            offset_x = min_offset
            offset_y = min_offset

        raw = [
            ('center', cx, cy),
            ('east_wide', cx + offset_x, cy),
            ('west_wide', cx - offset_x, cy),
            ('north_wide', cx, cy + offset_y),
            ('south_wide', cx, cy - offset_y),
            ('north_east_wide', cx + offset_x, cy + offset_y),
            ('north_west_wide', cx - offset_x, cy + offset_y),
            ('south_east_wide', cx + offset_x, cy - offset_y),
            ('south_west_wide', cx - offset_x, cy - offset_y),
        ]

        candidates = []
        seen = set()
        for label, x, y in raw:
            if not self._inside_bounds(area, x, y):
                continue
            key = (round(x, 3), round(y, 3))
            if key in seen:
                continue
            seen.add(key)
            candidates.append({'label': label, 'x': round(x, 3), 'y': round(y, 3)})
        return candidates

    def _inside_bounds(self, area: dict, x: float, y: float) -> bool:
        bounds = area.get('bounds') or {}
        required = ['x_min', 'x_max', 'y_min', 'y_max']
        if not all(key in bounds for key in required):
            return True
        margin = float(self.get_parameter('bounds_margin').value)
        return (
            float(bounds['x_min']) + margin <= x <= float(bounds['x_max']) - margin
            and float(bounds['y_min']) + margin <= y <= float(bounds['y_max']) - margin
        )

    def build_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        quat = yaw_to_quaternion(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('goal_frame').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat['x']
        pose.pose.orientation.y = quat['y']
        pose.pose.orientation.z = quat['z']
        pose.pose.orientation.w = quat['w']
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _await_terminal(self, result_future, deadline_sec: float) -> bool:
        """Spin until the goal's result_future reaches a terminal state or the
        deadline elapses. Returns True iff terminal was confirmed."""
        deadline = time.time() + deadline_sec
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return result_future.done()

    def _finish(self, result: dict) -> dict:
        """Tag a nav result and, if it could NOT be confirmed terminal, latch
        the mission-wide abort so no further goal is dispatched."""
        if not result.get('safe_to_continue', True):
            self._nav_aborted = True
            self.get_logger().error(
                'Nav goal left in an unconfirmed state (%s) -- aborting further '
                'navigation to avoid a preempt race.' % result.get('status'))
        return result

    def send_goal_and_wait(self, goal: NavigateToPose.Goal) -> dict:
        timeout = float(self.get_parameter('server_timeout_sec').value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            # No goal was sent -> the server is executing nothing -> safe.
            return self._finish({'status': 'server_unavailable', 'safe_to_continue': True})

        goal_x = float(goal.pose.pose.position.x)
        goal_y = float(goal.pose.pose.position.y)
        started = time.time()
        send_future = self._client.send_goal_async(goal)
        # Bound the goal-request phase too: a server that accepts connections
        # but never answers send_goal would otherwise hang the mission forever.
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        if not send_future.done():
            return self._finish(self._handle_send_timeout(send_future, goal_x, goal_y, started))
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            # Not accepted -> server is executing nothing -> safe to continue.
            return self._finish({'status': 'rejected', 'safe_to_continue': True,
                                 'duration_sec': round(time.time() - started, 3)})

        result_future = goal_handle.get_result_async()
        nav_timeout = float(self.get_parameter('nav_goal_timeout_sec').value)

        def _cancel(reason: str) -> dict:
            self.get_logger().warn(
                'Cancelling nav goal (%.3f, %.3f): %s' % (goal_x, goal_y, reason))
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            # Did the server actually accept the cancel (vs the goal finishing
            # on its own)? goals_canceling is non-empty only when accepted.
            cancel_accepted = bool(
                cancel_future.done() and cancel_future.result() is not None
                and len(cancel_future.result().goals_canceling) > 0)
            # Wait (bounded) for the goal to actually reach a terminal state.
            # Only once it is terminal is it safe to dispatch the next goal.
            terminal = self._await_terminal(result_future, 5.0)
            out = {
                'status': reason,
                'cancel_accepted': cancel_accepted,
                'cancel_terminal': 'confirmed' if terminal else 'unconfirmed',
                'safe_to_continue': terminal,
                'duration_sec': round(time.time() - started, 3),
            }
            if terminal:
                # If the goal actually completed on its own, surface the true
                # outcome instead of pretending the cancel reason was the cause.
                res = result_future.result()
                if res is not None:
                    out['final_nav_status'] = STATUS_TEXT.get(res.status, str(res.status))
            return out

        last_blocked_check = 0.0
        while rclpy.ok() and not result_future.done():
            elapsed = time.time() - started
            # Hard backstop: a goal Nav2 can never reach (recovery-cycling on
            # an obstacle) would hang forever -- bound it. Sized well above the
            # worst legitimate cross-map navigation so clear goals are never
            # falsely cancelled; a real deadlock never completes either way.
            if nav_timeout > 0.0 and elapsed > nav_timeout:
                return self._finish(_cancel('timeout'))
            # Mid-nav re-check (every ~2 s): once the robot is close enough to
            # mark an obstacle sitting ON the target, the goal cell turns
            # lethal -> cancel immediately instead of cycling recovery for the
            # whole backstop. A clear long-distance goal stays clear and keeps
            # navigating, so this never penalises normal (obstacle-free) runs.
            if elapsed - last_blocked_check >= 2.0:
                last_blocked_check = elapsed
                if not self.candidate_is_clear(goal_x, goal_y):
                    return self._finish(_cancel('goal_blocked'))
            rclpy.spin_once(self, timeout_sec=0.1)
        result = result_future.result()
        status_text = STATUS_TEXT.get(result.status, str(result.status))
        return self._finish({
            'status': status_text,
            'safe_to_continue': True,
            'duration_sec': round(time.time() - started, 3),
        })

    def _handle_send_timeout(self, send_future, goal_x: float, goal_y: float,
                             started: float) -> dict:
        """A send_goal request did not get a response in time. The request may
        still be pending inside rclpy; if the server later accepts it the goal
        runs UNMANAGED. Give the response a bounded second chance: if a handle
        arrives accepted, cancel it and confirm terminal; only if we can prove
        nothing is (or will be) executing is it safe to continue."""
        extra = float(self.get_parameter('server_timeout_sec').value)
        deadline = time.time() + extra
        while rclpy.ok() and not send_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send_future.done():
            # Still no response -- the request may yet be accepted server-side.
            # Stop the client from acting on a late response and fail safe.
            try:
                send_future.cancel()
            except Exception:
                pass
            return {'status': 'send_timeout', 'safe_to_continue': False,
                    'duration_sec': round(time.time() - started, 3)}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'status': 'send_timeout_rejected', 'safe_to_continue': True,
                    'duration_sec': round(time.time() - started, 3)}
        self.get_logger().warn(
            'Late goal accept after send_timeout (%.3f, %.3f) -- cancelling'
            % (goal_x, goal_y))
        result_future = goal_handle.get_result_async()
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        terminal = self._await_terminal(result_future, 5.0)
        return {
            'status': 'send_timeout_cancelled',
            'cancel_terminal': 'confirmed' if terminal else 'unconfirmed',
            'safe_to_continue': terminal,
            'duration_sec': round(time.time() - started, 3),
        }

    def wait_for_sensor_settle(self):
        settle = float(self.get_parameter('scan_settle_sec').value)
        end_time = time.time() + settle
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def collect_scan_sample(
        self,
        x: float,
        y: float,
        yaw: float,
        area_key: str,
        area_dir: Path,
        index: int,
    ) -> dict:
        goal = self.build_goal(x, y, yaw)
        nav_result = self.send_goal_and_wait(goal)
        self.wait_for_sensor_settle()
        summary = summarize_scan(self._latest_scan)
        image_capture = self.capture_image(area_key, area_dir, index, yaw)
        observed_pose = self._own_pose
        if observed_pose is None:
            observed_pose = (float(x), float(y), float(yaw))
            pose_source = 'commanded_fallback'
        else:
            pose_source = 'amcl_at_capture'
        # Apply laser yaw correction (HIGHEST-IMPACT: AMCL yaw lags after
        # a spin; laser alignment gives the true heading for photo-diff).
        corrected_yaw = observed_pose[2]
        yaw_fit = 0.0
        if observed_pose is not None:
            corrected_yaw, yaw_fit = self.corrected_capture_yaw(
                observed_pose[0], observed_pose[1], observed_pose[2])
        summary.update({
            'index': index,
            'yaw': round(float(yaw), 4),
            'turn_result': nav_result,
            'pose_at_capture': (observed_pose[0], observed_pose[1], corrected_yaw),
            'capture_pose_source': pose_source,
            'yaw_correction': round(corrected_yaw - observed_pose[2], 4),
            'yaw_fit_ratio': round(yaw_fit, 3),
            'image_capture': image_capture,
        })
        return summary

    def inspect_area(
        self,
        area_key: str,
        area: dict,
        sequence_index: int,
        dry_run: bool,
    ) -> dict:
        candidates = self.generate_candidate_poses(area)
        # Per-area override: a doorway viewpoint photographs INTO the room
        # (a few inward yaws) instead of the default 360-degree sweep.
        scan_yaws = [float(value) for value in
                     (area.get('scan_yaws') or self.get_parameter('scan_yaws').value)]
        area_dir = self.area_evidence_dir(sequence_index, area_key)
        result = {
            'sequence_index': sequence_index,
            'target_area': area_key,
            'display_name': area.get('display_name', area_key),
            'status': 'pending',
            'evidence_dir': str(area_dir),
            'candidate_poses': candidates,
            'selected_pose': None,
            'nav_attempts': [],
            'nav_fail_evidence': [],
            'candidate_attempt_limit': int(self.get_parameter('max_candidate_attempts_per_area').value),
            'candidate_spread_ratio': float(self.get_parameter('candidate_spread_ratio').value),
            'scan_sequence': [round(yaw, 4) for yaw in scan_yaws],
            'scan_samples': [],
            'photo_diff': {
                'status': 'not_run',
                'views': [],
                'anomalies': [],
            },
        }

        area_dir.mkdir(parents=True, exist_ok=True)
        if dry_run:
            result['status'] = 'dry_run'
            result['scan_summary'] = aggregate_scan_summaries([])
            return result

        if not candidates:
            result['status'] = 'unchecked'
            result['reason'] = 'no_candidate_pose_inside_bounds'
            result['scan_summary'] = aggregate_scan_summaries([])
            return result

        if self._nav_aborted:
            result['status'] = 'nav_aborted'
            result['reason'] = 'prior_nav_goal_unconfirmed'
            result['scan_summary'] = aggregate_scan_summaries([])
            return result

        selected = None
        stops: list[dict] = []
        attempt_limit = int(self.get_parameter('max_candidate_attempts_per_area').value)
        attempts_made = 0
        # A-axis hook: candidates an unmapped obstacle occupies in the costmap.
        blocked_candidates = []
        # Explicit viewpoints are each REQUIRED coverage (e.g. a large hall needs
        # a west AND an east stop), not interchangeable backups: costmap-prescreen
        # them (same obstacle guard as the ring path) and sweep EVERY clear one.
        # Generated ring candidates ARE backups -- take the first that navigates.
        # The sweep loop below drives the robot to each stop, so the viewpoint
        # path needs no pre-nav and never doubles back.
        viewpoint_mode = bool(area.get('viewpoints'))
        if viewpoint_mode:
            for candidate in candidates:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.candidate_is_clear(candidate['x'], candidate['y']):
                    stops.append(candidate)
                else:
                    self.get_logger().warn(
                        '%s viewpoint %s (%.3f, %.3f) blocked in costmap '
                        '(unmapped obstacle) -- skipping'
                        % (area_key, candidate['label'],
                           candidate['x'], candidate['y']))
                    blocked_candidates.append(dict(candidate))
            selected = stops[0] if stops else None
        ring_candidates = [] if viewpoint_mode else candidates
        for candidate in ring_candidates:
            if attempt_limit > 0 and attempts_made >= attempt_limit:
                break
            # Refresh the costmap, then skip any candidate an obstacle occupies
            # so we never aim the robot INTO an obstacle (no collision/hugging
            # /center-on-obstacle deadlock). An obstacle discovered en route is
            # caught here on the next iteration, after the failed attempt below
            # has updated the costmap.
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.candidate_is_clear(candidate['x'], candidate['y']):
                self.get_logger().warn(
                    '%s candidate %s (%.3f, %.3f) blocked in costmap '
                    '(unmapped obstacle) -- skipping'
                    % (area_key, candidate['label'], candidate['x'], candidate['y'])
                )
                blocked_candidates.append(dict(candidate))
                continue
            attempts_made += 1
            self.get_logger().info(
                'Trying %s candidate %s x=%.3f y=%.3f'
                % (area_key, candidate['label'], candidate['x'], candidate['y'])
            )
            goal = self.build_goal(candidate['x'], candidate['y'],
                                   float(candidate.get('yaw', 0.0)))
            nav_result = self.send_goal_and_wait(goal)
            attempt = dict(candidate)
            attempt['result'] = nav_result
            result['nav_attempts'].append(attempt)
            if nav_result.get('status') == 'succeeded':
                selected = candidate
                break

            # An unconfirmed-terminal goal makes any further dispatch unsafe:
            # stop trying candidates rather than racing a possibly-live goal.
            if self._nav_aborted:
                break

            # Let the costmap absorb whatever the robot sensed en route before
            # the next candidate is re-validated against it.
            rclpy.spin_once(self, timeout_sec=0.2)
            evidence = self.capture_nav_fail_evidence(area_key, area_dir, len(result['nav_attempts']))
            if evidence:
                attempt['nav_fail_evidence'] = evidence
                result['nav_fail_evidence'].append(evidence)

        if not viewpoint_mode and selected is not None:
            stops = [selected]
        result['blocked_candidates'] = blocked_candidates
        if not stops:
            if self._nav_aborted:
                result['status'] = 'nav_aborted'
                result['reason'] = 'nav_goal_unconfirmed_terminal'
                result['scan_summary'] = aggregate_scan_summaries([])
                return result
            result['status'] = 'nav_failed'
            if blocked_candidates and not result['nav_attempts']:
                result['reason'] = 'all_candidates_blocked_by_unmapped_obstacle'
            elif attempt_limit > 0 and attempts_made >= attempt_limit:
                result['reason'] = 'candidate_attempt_limit_reached'
            else:
                result['reason'] = 'all_attempted_candidate_poses_failed'
            result['scan_summary'] = aggregate_scan_summaries([])
            return result

        result['selected_pose'] = selected
        result['selected_stops'] = [dict(s) for s in stops]
        photo_diff_on = (bool(self.get_parameter('detect_photo_diff').value)
                         and not dry_run)
        sample_seq = 0
        for stop in stops:
            stop_label = stop.get('label', 'stop')
            for yaw_index, yaw in enumerate(scan_yaws):
                sample_seq += 1
                self.get_logger().info(
                    'Inspecting %s/%s yaw=%.4f' % (area_key, stop_label, yaw))
                sample = self.collect_scan_sample(
                    stop['x'],
                    stop['y'],
                    yaw,
                    area_key,
                    area_dir,
                    sample_seq,
                )
                # Annotate sample with stop/yaw metadata for photo_diff_stop lookup
                sample['stop_label'] = stop_label
                sample['yaw_index'] = yaw_index
                result['scan_samples'].append(sample)
                # A turn-in-place goal we could not confirm terminal aborts the
                # rest of the sweep (and the mission) for the same safety reason.
                if self._nav_aborted:
                    result['scan_summary'] = aggregate_scan_summaries(result['scan_samples'])
                    result['status'] = 'nav_aborted'
                    result['reason'] = 'nav_goal_unconfirmed_terminal_during_scan'
                    return result

        # Photo diff detection: store the area context so process_photo_views
        # can pass it to photo_diff_stop (detect_bounds, photo_detect flag, etc.)
        if photo_diff_on:
            self._current_area = area
            result['photo_diff'] = self.process_photo_views(
                area_key, result['scan_samples'])
            del self._current_area
            # Publish each anomaly at the pose of the stop it was detected from
            # (multi-viewpoint areas have more than one).
            if result['photo_diff'].get('anomalies'):
                stop_by_label = {s.get('label', 'stop'): s for s in stops}
                for anomaly in result['photo_diff']['anomalies']:
                    src = anomaly.get('detected_from', {})
                    stop = stop_by_label.get(src.get('stop'), selected)
                    self.publish_anomaly(area_key, anomaly,
                                         {'x': stop['x'], 'y': stop['y']})

        result['scan_summary'] = aggregate_scan_summaries(result['scan_samples'])
        # Observability: a stop we navigated to and photographed but for which NO
        # baseline could be compared (photo_diff -> 'no_baseline') is NOT a clean
        # inspection. Surface it as a distinct status (+ warn) so a baseline gap
        # cannot masquerade as 'checked' in the mission summary -- e.g. a ring
        # fall-through to an un-recorded backup stop (observed: main_corridor box
        # at center blocked center+east_wide, robot scanned the un-recorded
        # north_wide -> silent no_baseline), or a new viewpoint not yet recorded.
        # Only triggers in detect mode (photo_diff actually ran).
        if photo_diff_on and result['photo_diff'].get('status') == 'no_baseline':
            result['status'] = 'checked_no_baseline'
            self.get_logger().warn(
                '%s: scanned %d photo(s) but found no baseline to diff against '
                '(photo_diff=no_baseline) -- area NOT inspected for anomalies; '
                'record its baseline at the visited stop(s)'
                % (area_key, len(result['scan_samples'])))
        else:
            result['status'] = 'checked'
        return result

    def create_run_dir(self, route: list[str]) -> Path:
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        route_hint = '_'.join(safe_path_name(key) for key in route[:3])
        if len(route) > 3:
            route_hint += '_etc'
        name = f'inspection_{timestamp}_{route_hint or "route"}'
        run_dir = Path(self.get_parameter('report_dir').value).expanduser() / name
        run_dir.mkdir(parents=True, exist_ok=False)
        self._run_dir = run_dir
        return run_dir

    def area_evidence_dir(self, sequence_index: int, area_key: str) -> Path:
        if self._run_dir is None:
            raise RuntimeError('inspection run directory has not been created')
        return self._run_dir / f'{sequence_index:02d}_{safe_path_name(area_key)}'

    def home_pose(self, world_model: dict) -> dict:
        home_x = float(self.get_parameter('home_x').value)
        home_y = float(self.get_parameter('home_y').value)
        if math.isfinite(home_x) and math.isfinite(home_y):
            return {
                'source': 'param_override',
                'area': None,
                'x': home_x,
                'y': home_y,
                'yaw': float(self.get_parameter('home_yaw').value),
                'standoff_distance': 0.0,
            }

        robot_start = world_model.get('robot_start') or {}
        start_pose = robot_start.get('pose') or {}
        if 'x' in start_pose and 'y' in start_pose:
            yaw = float(start_pose.get('yaw', 0.0))
            standoff = float(self.get_parameter('return_home_standoff_distance').value)
            return {
                'source': 'robot_start_standoff' if standoff else 'robot_start',
                'area': robot_start.get('area'),
                'x': float(start_pose['x']) - math.cos(yaw) * standoff,
                'y': float(start_pose['y']) - math.sin(yaw) * standoff,
                'yaw': yaw,
                'standoff_distance': round(standoff, 3),
                'dock_pose': {
                    'x': round(float(start_pose['x']), 3),
                    'y': round(float(start_pose['y']), 3),
                    'yaw': round(yaw, 4),
                },
            }

        home_area = str(self.get_parameter('home_area').value or '').strip()
        if home_area:
            area_key, area = self.resolve_area(world_model, home_area)
            center = area.get('center') or []
            if len(center) >= 2:
                return {
                    'source': 'home_area',
                    'area': area_key,
                    'x': float(center[0]),
                    'y': float(center[1]),
                    'yaw': 0.0,
                    'standoff_distance': 0.0,
                }

        raise ValueError('No return-home pose available from robot_start or home_area')

    def return_home_result(self, world_model: dict, dry_run: bool) -> dict:
        attempted = bool(self.get_parameter('return_home').value)
        result = {
            'attempted': attempted,
            'target': None,
            'pose': None,
            'result': None,
        }
        if not attempted:
            result['result'] = {'status': 'disabled'}
            return result

        pose = self.home_pose(world_model)
        result['target'] = pose.get('area') or pose.get('source')
        result['pose'] = {
            'x': round(float(pose['x']), 3),
            'y': round(float(pose['y']), 3),
            'yaw': round(float(pose['yaw']), 4),
        }
        result['source'] = pose.get('source')
        result['standoff_distance'] = pose.get('standoff_distance')
        if pose.get('dock_pose'):
            result['dock_pose'] = pose['dock_pose']
        if dry_run:
            result['result'] = {'status': 'dry_run'}
            return result

        if self._nav_aborted:
            # A prior goal could not be confirmed terminal -- do not command the
            # robot home on top of a possibly-live goal.
            result['result'] = {'status': 'skipped_nav_aborted', 'safe_to_continue': False}
            return result

        self.get_logger().info(
            'Returning home: x=%.3f y=%.3f yaw=%.4f'
            % (pose['x'], pose['y'], pose['yaw'])
        )
        nav_result = self.send_goal_and_wait(self.build_goal(pose['x'], pose['y'], pose['yaw']))
        result['result'] = nav_result
        return result

    def build_summary_report(self, detail_report: dict, details_path: Path) -> dict:
        areas = []
        for area in detail_report.get('areas', []):
            image_paths = []
            image_statuses = []
            nav_fail_image_paths = []
            for evidence in area.get('nav_fail_evidence', []):
                image_statuses.append(evidence.get('status', 'unknown'))
                if evidence.get('image_path'):
                    nav_fail_image_paths.append(evidence['image_path'])

            for sample in area.get('scan_samples', []):
                capture = sample.get('image_capture') or {}
                image_statuses.append(capture.get('status', 'unknown'))
                if capture.get('image_path'):
                    image_paths.append(capture['image_path'])

            all_image_paths = nav_fail_image_paths + image_paths
            photo_diff = area.get('photo_diff') or {}
            area_summary = {
                'sequence_index': area.get('sequence_index'),
                'area': area.get('target_area'),
                'display_name': area.get('display_name'),
                'status': area.get('status'),
                'evidence_dir': area.get('evidence_dir'),
                'captured_image_count': len(all_image_paths),
                'image_paths': all_image_paths,
                'photo_diff_status': photo_diff.get('status'),
                'anomalies': photo_diff.get('anomalies', []),
            }
            if nav_fail_image_paths:
                area_summary['nav_fail_image_paths'] = nav_fail_image_paths
            if area.get('reason'):
                area_summary['reason'] = area.get('reason')
            if image_statuses and len(image_paths) != len(image_statuses):
                area_summary['image_capture_statuses'] = image_statuses
            areas.append(area_summary)

        return_home = detail_report.get('return_home') or {}
        return_result = return_home.get('result') or {}
        summary = dict(detail_report.get('summary') or {})
        summary['return_home_status'] = return_result.get('status')

        return {
            'task': detail_report.get('task'),
            'status': detail_report.get('status'),
            'run_dir': detail_report.get('run_dir'),
            'route': detail_report.get('route'),
            'summary': summary,
            'anomalies': detail_report.get('anomalies', []),
            'areas': areas,
            'return_home': {
                'attempted': return_home.get('attempted'),
                'target': return_home.get('target'),
                'status': return_result.get('status'),
            },
            'details_file': str(details_path),
            'notes': [
                'v0.4: P1-5v photo-baseline anomaly detection with OLD laser '
                'yaw correction and flat baseline layout.',
                'anomalies[].type=photo_diff marks visual anomalies.',
            ],
        }

    def run_once(self) -> int:
        world_model = self.load_world_model()
        # Kept on the node so detect_bounds() can resolve a viewpoint's
        # photo_detect_bounds_area (e.g. restricted_gate -> restricted_zone).
        self._world_model = world_model
        targets = self.requested_targets()
        resolved = [self.resolve_area(world_model, target) for target in targets]
        dry_run = bool(self.get_parameter('dry_run').value)
        route = [area_key for area_key, _area in resolved]
        run_dir = self.create_run_dir(route)

        report = {
            'task': 'inspect_route' if len(resolved) > 1 else 'inspect_area',
            'status': 'pending',
            'run_dir': str(run_dir),
            'route': route,
            'execution_policy': {
                'continue_on_area_nav_fail': True,
                'return_home_after_route': bool(self.get_parameter('return_home').value),
                'return_home_standoff_distance': float(self.get_parameter('return_home_standoff_distance').value),
                'max_candidate_attempts_per_area': int(self.get_parameter('max_candidate_attempts_per_area').value),
                'candidate_spread_ratio': float(self.get_parameter('candidate_spread_ratio').value),
                'capture_nav_fail_evidence': bool(self.get_parameter('capture_nav_fail_evidence').value),
            },
            'summary': {
                'requested_count': len(resolved),
                'checked_count': 0,
                'failed_count': 0,
                'unchecked_count': 0,
                'no_baseline_count': 0,
            },
            'areas': [],
            'return_home': None,
        }

        self.get_logger().info('Inspection route: %s' % ', '.join(route))
        for index, (area_key, area) in enumerate(resolved, start=1):
            area_result = self.inspect_area(area_key, area, index, dry_run)
            report['areas'].append(area_result)

        checked = [area for area in report['areas'] if area.get('status') == 'checked']
        failed = [area for area in report['areas'] if area.get('status') == 'nav_failed']
        unchecked = [area for area in report['areas'] if area.get('status') == 'unchecked']
        aborted = [area for area in report['areas'] if area.get('status') == 'nav_aborted']
        no_baseline = [area for area in report['areas']
                       if area.get('status') == 'checked_no_baseline']
        anomalies = []
        for area in report['areas']:
            for anomaly in (area.get('photo_diff') or {}).get('anomalies', []):
                anomalies.append({'area': area.get('target_area'), **anomaly})
        report['anomalies'] = anomalies
        report['summary'].update({
            'checked_count': len(checked),
            'failed_count': len(failed),
            'unchecked_count': len(unchecked),
            'aborted_count': len(aborted),
            'no_baseline_count': len(no_baseline),
            'anomaly_count': len(anomalies),
        })

        report['return_home'] = self.return_home_result(world_model, dry_run)
        return_status = (report['return_home'].get('result') or {}).get('status')

        if dry_run:
            report['status'] = 'dry_run'
        elif self._nav_aborted:
            # A nav goal could not be confirmed terminal; the mission was stopped
            # rather than risk commanding the robot on top of a live goal.
            report['status'] = 'aborted_unsafe_nav_state'
        elif return_status not in {'succeeded', 'disabled'}:
            report['status'] = 'completed_return_failed'
        elif failed or unchecked or no_baseline:
            # A scanned-but-undiffable area (no baseline) is a partial inspection
            # failure, same class as unchecked -- it must not read as 'completed'.
            report['status'] = 'completed_with_failures'
        else:
            report['status'] = 'completed'

        details_path = write_report(report, run_dir, filename='details.yaml')
        summary_report = self.build_summary_report(report, details_path)
        report_path = write_report(summary_report, run_dir, filename='report.yaml')
        self.get_logger().info('Inspection report written: %s' % report_path)
        self.get_logger().info('Inspection details written: %s' % details_path)
        return 0 if report['status'] in {'completed', 'dry_run'} else 5

    def capture_nav_fail_evidence(self, area_key: str, area_dir: Path, attempt_index: int) -> dict | None:
        if not bool(self.get_parameter('capture_nav_fail_evidence').value):
            return None
        capture = self.capture_named_image(area_dir, f'nav_fail_attempt_{attempt_index:02d}')
        capture['attempt_index'] = attempt_index
        capture['description'] = 'Camera evidence captured immediately after a failed Nav2 attempt.'
        return capture

    def capture_named_image(self, area_dir: Path, stem: str) -> dict:
        self._latest_image = None
        settle = float(self.get_parameter('camera_settle_sec').value)
        end_time = time.time() + settle
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_image is not None:
                break

        if self._latest_image is None:
            return {'status': 'no_image_received', 'image_path': None}

        area_dir.mkdir(parents=True, exist_ok=True)
        image_path = save_ros_image_as_portable_image(self._latest_image, area_dir / stem)
        if image_path is None:
            return {
                'status': 'unsupported_encoding',
                'image_path': None,
                'encoding': self._latest_image.encoding,
            }
        return {
            'status': 'captured',
            'image_path': str(image_path),
            'encoding': self._latest_image.encoding,
        }

    def capture_image(self, area_key: str, area_dir: Path, sample_index: int, yaw: float) -> dict:
        return self.capture_named_image(area_dir, f'scan_{sample_index:02d}_yaw_{yaw:.4f}')


def save_ros_image_as_portable_image(image: Image, base_path: Path) -> Path | None:
    encoding = image.encoding.lower()
    width = int(image.width)
    height = int(image.height)
    step = int(image.step)
    data = bytes(image.data)

    if encoding == 'mono8':
        path = base_path.with_suffix('.pgm')
        with path.open('wb') as file:
            file.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            for row in range(height):
                start = row * step
                file.write(data[start:start + width])
        return path

    if encoding not in {'rgb8', 'bgr8', 'rgba8', 'bgra8'}:
        return None

    channels = 4 if encoding in {'rgba8', 'bgra8'} else 3
    path = base_path.with_suffix('.ppm')
    with path.open('wb') as file:
        file.write(f'P6\n{width} {height}\n255\n'.encode('ascii'))
        for row in range(height):
            start = row * step
            row_data = data[start:start + width * channels]
            if encoding == 'rgb8':
                file.write(row_data)
            elif encoding == 'rgba8':
                file.write(b''.join(row_data[i:i + 3] for i in range(0, len(row_data), 4)))
            elif encoding == 'bgr8':
                file.write(b''.join(
                    bytes((row_data[i + 2], row_data[i + 1], row_data[i]))
                    for i in range(0, len(row_data), 3)
                ))
            elif encoding == 'bgra8':
                file.write(b''.join(
                    bytes((row_data[i + 2], row_data[i + 1], row_data[i]))
                    for i in range(0, len(row_data), 4)
                ))
    return path


def main(args=None):
    rclpy.init(args=args)
    node = InspectionRunner()
    try:
        code = node.run_once()
    except Exception as exc:  # noqa: BLE001 - keep prototype errors visible in ROS logs.
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
