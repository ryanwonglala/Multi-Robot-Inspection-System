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
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from task_layer.anomaly_scanner import AnomalyScanner
from task_layer.area_clear_check import AreaClearChecker, merge_detections
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
        # 2 -> 4 (plan C5): with costmap prechecking, trying more ring
        # candidates is cheap — blocked ones are skipped without driving.
        self.declare_parameter('max_candidate_attempts_per_area', 4)
        # --- P1-5: anomaly detection + candidate costmap prechecking ---
        # Laser map-diff detection is retired (its test regime — large
        # unmapped obstacles — corrupts AMCL, see MASTER_PLAN C5 pivot).
        # Kept behind this flag as an experiment; photo diff is the default.
        self.declare_parameter('detect_anomalies', False)
        # Photo-diff anomaly detection (P1-5v): compare each inspection
        # photo against the baseline photo recorded from the same stop/yaw
        # when the scene was clean. baseline_record:=true turns a run into
        # the baseline patrol that produces that library.
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
        self.declare_parameter('max_pose_std', 0.35)
        self.declare_parameter('min_alignment', 0.80)
        self.declare_parameter('detect_frames', 5)
        self.declare_parameter('detect_bounds_margin', 0.30)
        self.declare_parameter('costmap_precheck', True)
        # Occupancy values are 0-100; 50 is past the inscribed band the
        # planner will not enter. -1 (unknown) also counts as blocked.
        # 120 clears the inflation overlap of a 1 m corridor (midline cost
        # ~94 with 0.7 m inflation: narrow_passage would otherwise be
        # condemned forever) while still catching real occupancy: an object
        # on the goal puts lethal/inscribed (253/254) cells in the check
        # neighborhood, and unknown (-1) always blocks.
        self.declare_parameter('costmap_cost_threshold', 120)
        self.declare_parameter('precheck_radius', 0.18)
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

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._latest_scan = None
        self._latest_image = None
        self._run_dir = None
        scan_topic = self.get_parameter('scan_topic').value
        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(LaserScan, scan_topic, self._scan_callback, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self._camera_info = None
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._camera_info_callback, 10)

        self.robot_name = self.get_namespace().strip('/') or 'robot'
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._anomaly_seq = 0
        self._yaw_corrector = None  # lazy: loads the static map on first use
        # Own belief pose, independent of any detector: photo localization
        # and the post-failure displacement check both need it.
        self._own_pose = None  # (x, y, yaw)
        amcl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._on_amcl_pose, amcl_qos)
        # Peer belief poses: a teammate caught in an inspection photo would
        # otherwise diff against the (empty) baseline as an anomaly.
        self._peer_poses: dict[str, tuple[float, float]] = {}
        for peer, topic in self._peer_pose_topics().items():
            self.create_subscription(
                PoseWithCovarianceStamped, topic,
                lambda msg, name=peer: self._on_peer_pose(name, msg),
                amcl_qos)
        self.scanner = None
        if bool(self.get_parameter('detect_anomalies').value):
            self.scanner = AnomalyScanner(
                self,
                map_yaml=str(self.get_parameter('map_yaml').value),
                robots_yaml=str(self.get_parameter('robots_yaml').value),
                max_pose_std=float(self.get_parameter('max_pose_std').value),
                min_alignment=float(self.get_parameter('min_alignment').value),
                frames=int(self.get_parameter('detect_frames').value),
            )
        latched = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # Fleet-wide buses, deliberately absolute (one shared channel for all
        # robots; latched so the GUI/allocator and a late RViz still see them).
        self._event_pub = self.create_publisher(String, '/anomaly_events', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '/anomaly_markers', latched)
        self._costmap = None
        if bool(self.get_parameter('costmap_precheck').value):
            costmap_qos = QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, 'global_costmap/costmap',
                                     self._costmap_callback, costmap_qos)

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

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._own_pose = (p.position.x, p.position.y, yaw)

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

    def _image_callback(self, msg: Image):
        self._latest_image = msg

    def _camera_info_callback(self, msg: CameraInfo):
        self._camera_info = msg

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

    def photo_diff_stop(self, area_key: str, area: dict, stop: dict,
                        stop_samples: list[tuple[int, dict]]) -> dict:
        """Per-stop photo handling: in baseline_record mode archive the
        photos as the clean reference; otherwise diff each photo against
        its baseline and return map-frame anomaly candidates."""
        record = bool(self.get_parameter('baseline_record').value)
        stop_label = str(stop.get('label', 'stop'))
        outcome = {'stop': {'label': stop_label, 'x': stop['x'], 'y': stop['y']},
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
                    stop['x'], stop['y'], float(sample.get('yaw', 0.0)))
                target.with_suffix('.json').write_text(json.dumps(
                    {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}))
                recorded += 1
            outcome.update({'status': 'baseline_recorded', 'photos': recorded})
            self.get_logger().info(
                '%s/%s: %d baseline photo(s) recorded'
                % (area_key, stop_label, recorded))
            return outcome

        if not area.get('photo_detect', True):
            # Areas where photo diff is structurally unsound (a 1 m corridor:
            # every wall is near-field, station parallax dwarfs any object
            # signal — and any real obstacle there blocks the lane, which
            # the costmap precheck/en-route cancel already reports).
            outcome.update({'status': 'photo_detect_disabled'})
            return outcome
        camera = self.camera_model()
        bounds = self.detect_bounds(area)
        clip = bool(area.get('photo_detect_clip_bounds', True))
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
                stop['x'], stop['y'], float(sample.get('yaw', 0.0)))
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
                found.append(anomaly)
        outcome['status'] = 'checked' if checked else 'no_baseline'
        outcome['anomalies'] = merge_photo_detections([], found)
        outcome['photos_checked'] = checked
        return outcome

    def _costmap_callback(self, msg: OccupancyGrid):
        self._costmap = msg

    def candidate_blocked(self, x: float, y: float) -> bool:
        """True when the latest global costmap shows lethal/unknown cost in
        the robot-radius neighborhood of (x, y). TB4-era 'circling' was Nav2
        recovery-looping on goals that sat on an obstacle; checking the live
        costmap before sending (and again after any abort, by which time the
        drive-by has painted the obstacle in) replaces blind retries."""
        grid = self._costmap
        if grid is None:
            return False  # no data: do not block inspection on a missing topic
        threshold = int(self.get_parameter('costmap_cost_threshold').value)
        radius = float(self.get_parameter('precheck_radius').value)
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        cells = max(1, int(radius / res))
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                r, c = row + dr, col + dc
                if not (0 <= r < grid.info.height and 0 <= c < grid.info.width):
                    return True  # outside the map
                value = grid.data[r * grid.info.width + c]
                if value < 0 or value >= threshold:
                    return True
        return False

    def clear_global_costmap(self) -> bool:
        """One-shot global costmap reset. Obstacle marks persist after the
        object is gone until a live scan ray crosses that cell again; cells
        the robot never re-observes (inside no-entry rooms, behind narrow
        doorways) keep their ghosts forever and eventually condemn every
        candidate pose of an area. Only call this when localization is
        trusted — clearing with a bad pose bakes the error in (see the
        rescue-order rule in DEVLOG)."""
        client = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('global costmap clear service unavailable')
            return False
        future = client.call_async(ClearEntireCostmap.Request())
        end = time.time() + 5.0
        while time.time() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        # Give the costmap a moment to repaint from live sensor data.
        settle = time.time() + 2.0
        while time.time() < settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def unstick_reverse(self, distance_m: float = 0.45, speed: float = 0.08):
        """Deterministic back-off after a failed approach. Driving at an
        occupied goal wedges the nose into the obstacle's inflation zone;
        with the start pose near-lethal every later plan fails instantly
        (observed: center blocked -> all ring candidates 'aborted' in 25 s).
        Reversing the way we came is the documented unstick recipe."""
        msg = Twist()
        msg.linear.x = -abs(speed)
        end = time.time() + distance_m / abs(speed)
        while time.time() < end:
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._cmd_pub.publish(Twist())

    def detect_bounds(self, area: dict):
        """Inspected area's bounds shrunk by the detection margin (doorway
        leakage filter, same rule the P1-4 gates passed with)."""
        bounds = area.get('bounds') or {}
        if not all(k in bounds for k in ('x_min', 'x_max', 'y_min', 'y_max')):
            return None
        margin = float(self.get_parameter('detect_bounds_margin').value)
        return (float(bounds['x_min']) + margin, float(bounds['y_min']) + margin,
                float(bounds['x_max']) - margin, float(bounds['y_max']) - margin)

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
            'ANOMALY %s in %s at (%.2f, %.2f) extent=%.2f cells=%d'
            % (self._anomaly_seq, area_key, anomaly['x'], anomaly['y'],
               anomaly.get('extent') or 0.0, anomaly.get('cells') or 0))

    def capture_anomaly_evidence(self, stop: dict, anomaly: dict,
                                 area_dir: Path, index: int) -> dict:
        """Turn in place to face the anomaly and take one close-look photo."""
        yaw = math.atan2(anomaly['y'] - stop['y'], anomaly['x'] - stop['x'])
        self.send_goal_and_wait(self.build_goal(stop['x'], stop['y'], yaw))
        capture = self.capture_named_image(area_dir, f'anomaly_{index:02d}')
        capture['description'] = 'Camera evidence facing a detected anomaly.'
        return capture

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

    def send_goal_and_wait(self, goal: NavigateToPose.Goal,
                           blocked_probe: tuple | None = None) -> dict:
        timeout = float(self.get_parameter('server_timeout_sec').value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            return {'status': 'server_unavailable'}

        started = time.time()
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'status': 'rejected', 'duration_sec': round(time.time() - started, 3)}

        result_future = goal_handle.get_result_async()
        last_probe = time.time()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            # En-route goal precheck: an obstacle on the goal only enters the
            # costmap once the lidar sees it (~3.5 m out). Cancelling RIGHT
            # THEN stops the robot at sensor range; letting Nav2 push on
            # wedges the nose into the inflation zone where MPPI has no
            # collision-free samples left ('Optimizer fail to compute path'
            # loop, observed) and even spin recovery reports
            # 'Collision Ahead'.
            if blocked_probe is not None and time.time() - last_probe > 1.0:
                last_probe = time.time()
                if self.candidate_blocked(*blocked_probe):
                    goal_handle.cancel_goal_async()
                    cancel_deadline = time.time() + 5.0
                    while time.time() < cancel_deadline and not result_future.done():
                        rclpy.spin_once(self, timeout_sec=0.1)
                    return {'status': 'goal_blocked_en_route',
                            'duration_sec': round(time.time() - started, 3)}
        result = result_future.result()
        status_text = STATUS_TEXT.get(result.status, str(result.status))
        return {
            'status': status_text,
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
        summary.update({
            'index': index,
            'yaw': round(float(yaw), 4),
            'turn_result': nav_result,
            'image_capture': self.capture_image(area_key, area_dir, index, yaw),
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

        attempt_limit = int(self.get_parameter('max_candidate_attempts_per_area').value)
        precheck_on = (bool(self.get_parameter('costmap_precheck').value)
                       and not dry_run)
        nav_tries = 0

        def try_reach(candidate) -> bool:
            """Costmap precheck + navigation, both recorded as attempts.
            Prechecked-blocked candidates cost no drive and no attempt
            quota; each later precheck naturally re-reads the freshest
            costmap (the failed drive painted the obstacle in), which
            replaces blind fixed-order retries."""
            nonlocal nav_tries
            attempt = dict(candidate)
            if precheck_on and self.candidate_blocked(candidate['x'], candidate['y']):
                attempt['result'] = {'status': 'precheck_blocked'}
                result['nav_attempts'].append(attempt)
                self.get_logger().warn(
                    '%s candidate %s (%.2f, %.2f) blocked on the live '
                    'costmap, skipping without driving'
                    % (area_key, candidate['label'], candidate['x'], candidate['y']))
                return False
            self.get_logger().info(
                'Trying %s candidate %s x=%.3f y=%.3f'
                % (area_key, candidate['label'], candidate['x'], candidate['y']))
            nav_tries += 1
            pose_before = self._own_pose
            nav_result = self.send_goal_and_wait(
                self.build_goal(candidate['x'], candidate['y'],
                                float(candidate.get('yaw', 0.0))),
                blocked_probe=((candidate['x'], candidate['y'])
                               if precheck_on else None))
            attempt['result'] = nav_result
            result['nav_attempts'].append(attempt)
            if nav_result.get('status') == 'succeeded':
                return True
            if nav_result.get('status') == 'goal_blocked_en_route':
                # Robot stopped cleanly at sensor range — nothing to recover
                # from, and the stop itself is evidence the goal is occupied.
                self.get_logger().warn(
                    '%s candidate %s became blocked en route, goal cancelled'
                    % (area_key, candidate['label']))
                return False
            evidence = self.capture_nav_fail_evidence(
                area_key, area_dir, len(result['nav_attempts']))
            if evidence:
                attempt['nav_fail_evidence'] = evidence
                result['nav_fail_evidence'].append(evidence)
            # Back off only when the robot actually drove somewhere and got
            # wedged near the blocked goal. An instant planning failure
            # moves nothing — blind reversing then just walks the robot
            # backwards (observed: four reverses marched it into a doorway).
            pose_after = self._own_pose
            if (pose_before is not None and pose_after is not None
                    and math.hypot(pose_after[0] - pose_before[0],
                                   pose_after[1] - pose_before[1]) > 0.3):
                self.unstick_reverse()
            return False

        detections = []
        photo_checks = []
        photo_diff_on = (bool(self.get_parameter('detect_photo_diff').value)
                         and not dry_run)
        sample_index = 0

        def inspect_stop(stop):
            nonlocal sample_index
            stop_samples = []
            for yaw_index, yaw in enumerate(scan_yaws):
                sample_index += 1
                self.get_logger().info('Inspecting %s yaw=%.4f' % (area_key, yaw))
                sample = self.collect_scan_sample(
                    stop['x'], stop['y'], yaw, area_key, area_dir, sample_index)
                pose = self._own_pose
                if pose is not None:
                    corrected, fit = self.corrected_capture_yaw(*pose)
                    sample['pose_at_capture'] = (pose[0], pose[1], corrected)
                    sample['yaw_correction'] = round(corrected - pose[2], 4)
                    sample['yaw_fit_ratio'] = round(fit, 3)
                result['scan_samples'].append(sample)
                stop_samples.append((yaw_index, sample))
            if self.scanner is not None and not dry_run:
                detection = self.scanner.detect_here(bounds=self.detect_bounds(area))
                detection['stop'] = {'x': stop['x'], 'y': stop['y']}
                detections.append(detection)
            if photo_diff_on:
                photo_checks.append(
                    self.photo_diff_stop(area_key, area, stop, stop_samples))

        stops_visited = []
        # Areas longer than the lidar diameter (east_hall) declare
        # viewpoints_visit_all: every viewpoint is a mandatory stop and the
        # per-stop detections are merged. Otherwise candidates are
        # alternatives and the first reachable one wins — except during a
        # baseline patrol, which records the first TWO reachable stops so
        # the common "center occupied -> relocate to first ring candidate"
        # inspection still finds a baseline at the relocated stop.
        baseline_mode = bool(self.get_parameter('baseline_record').value)

        def visit_candidates():
            if area.get('viewpoints') and area.get('viewpoints_visit_all'):
                for candidate in candidates:
                    if try_reach(candidate):
                        stops_visited.append(candidate)
                        inspect_stop(candidate)
                return
            wanted_stops = 2 if baseline_mode else 1
            for candidate in candidates:
                if len(stops_visited) >= wanted_stops:
                    break
                if attempt_limit > 0 and nav_tries >= attempt_limit:
                    break
                if try_reach(candidate):
                    stops_visited.append(candidate)
                    inspect_stop(candidate)

        visit_candidates()
        # Distrust-the-map retry: ONE candidate physically cannot block a
        # whole area, so when every attempt read blocked the costmap itself
        # is suspect (stale ghosts of removed objects, or marks painted
        # during a past pose error). Localization is fine here — the robot
        # just navigated between areas — so clearing is safe; the truth
        # repaints from live scans within a couple of frames.
        blocked_states = {'precheck_blocked', 'goal_blocked_en_route'}
        if (not stops_visited and result['nav_attempts']
                and all((a.get('result') or {}).get('status') in blocked_states
                        for a in result['nav_attempts'])):
            self.get_logger().warn(
                '%s: every candidate read blocked — clearing the global '
                'costmap once and retrying (suspected stale ghosts)' % area_key)
            self.clear_global_costmap()
            result['costmap_cleared_retry'] = True
            visit_candidates()

        if not stops_visited:
            result['status'] = 'nav_failed'
            result['reason'] = (
                'candidate_attempt_limit_reached'
                if (attempt_limit > 0 and nav_tries >= attempt_limit
                    and len(result['nav_attempts']) < len(candidates))
                else 'all_attempted_candidate_poses_failed'
            )
            result['scan_summary'] = aggregate_scan_summaries([])
            result['area_clear'] = {'status': 'no_stop', 'anomalies': []}
            return result

        result['selected_pose'] = stops_visited[0]
        if len(stops_visited) > 1:
            result['stops_visited'] = stops_visited
        # "Center forced to relocate" is an anomaly PRIOR (plan C5): if the
        # primary stop was blocked/unreachable but a fallback worked,
        # something is occupying the primary spot — record it so it can
        # corroborate (or trigger re-checking of) area_clear findings.
        first = result['nav_attempts'][0] if result['nav_attempts'] else None
        if (first is not None and stops_visited
                and first['label'] != stops_visited[0]['label']
                and first['label'] in ('center', 'viewpoint_1')):
            result['center_relocated'] = {
                'from': {'label': first['label'], 'x': first['x'], 'y': first['y']},
                'to': stops_visited[0]['label'],
                'reason': (first.get('result') or {}).get('status'),
            }
            self.get_logger().warn(
                '%s: primary stop %s was unusable (%s) — relocated to %s; '
                'recorded as anomaly prior'
                % (area_key, first['label'],
                   result['center_relocated']['reason'], stops_visited[0]['label']))

        area_clear = {'status': 'disabled', 'anomalies': []}
        if detections:  # laser route, experimental (detect_anomalies:=true)
            merged = []
            for detection in detections:
                merged = merge_detections(merged, detection['anomalies'])
            checked = [d for d in detections if d['status'] == 'checked']
            area_clear = {
                'status': 'checked' if checked else detections[0]['status'],
                'anomalies': merged,
                'stops': detections,
            }
        if photo_checks:  # photo-diff route (P1-5v default)
            merged_photo = []
            for check in photo_checks:
                merged_photo = merge_photo_detections(
                    merged_photo, check['anomalies'])
            photo_checked = [c for c in photo_checks
                             if c['status'] == 'checked']
            photo_status = ('checked' if photo_checked
                            else photo_checks[0]['status'])
            if area_clear['status'] == 'disabled':
                area_clear = {'status': photo_status,
                              'anomalies': merged_photo,
                              'stops': photo_checks}
            else:  # both detectors on: photo findings join the list as-is
                area_clear['anomalies'] = (
                    area_clear['anomalies'] + merged_photo)
                area_clear['photo_status'] = photo_status
        if area_clear['anomalies']:
            final_stop = stops_visited[-1]
            for index, anomaly in enumerate(area_clear['anomalies'], start=1):
                capture = self.capture_anomaly_evidence(
                    final_stop, anomaly, area_dir, index)
                anomaly['evidence_photo'] = capture.get('image_path')
                self.publish_anomaly(area_key, anomaly,
                                     {'x': final_stop['x'], 'y': final_stop['y']})
        result['area_clear'] = area_clear

        result['scan_summary'] = aggregate_scan_summaries(result['scan_samples'])
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
            area_clear = area.get('area_clear') or {}
            area_summary = {
                'sequence_index': area.get('sequence_index'),
                'area': area.get('target_area'),
                'display_name': area.get('display_name'),
                'status': area.get('status'),
                'evidence_dir': area.get('evidence_dir'),
                'captured_image_count': len(all_image_paths),
                'image_paths': all_image_paths,
                'area_clear_status': area_clear.get('status'),
                'anomalies': area_clear.get('anomalies', []),
            }
            if area.get('center_relocated'):
                area_summary['center_relocated'] = area['center_relocated']
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
                'v0.3: includes laser-vs-map (area_clear) anomaly detection.',
                'anomalies[].type=center_relocated_prior marks a primary stop',
                'that was occupied (recorded as a prior, not a confirmed object).',
                'Visual anomaly judgment lands with the Final-phase vision work.',
            ],
        }

    def run_once(self) -> int:
        world_model = self.load_world_model()
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
        anomalies = []
        for area in report['areas']:
            for anomaly in (area.get('area_clear') or {}).get('anomalies', []):
                anomalies.append({'area': area.get('target_area'), **anomaly})
            if area.get('center_relocated'):
                relocated = area['center_relocated']
                anomalies.append({
                    'area': area.get('target_area'),
                    'type': 'center_relocated_prior',
                    # copy: sharing the nested dict makes yaml emit &id
                    # anchors in the human-facing report
                    'from': dict(relocated['from']),
                    'to': relocated['to'],
                    'reason': relocated['reason'],
                })
        report['anomalies'] = anomalies
        report['summary'].update({
            'checked_count': len(checked),
            'failed_count': len(failed),
            'unchecked_count': len(unchecked),
            'anomaly_count': len([a for a in anomalies
                                  if a.get('type') != 'center_relocated_prior']),
        })

        report['return_home'] = self.return_home_result(world_model, dry_run)
        return_status = (report['return_home'].get('result') or {}).get('status')

        if dry_run:
            report['status'] = 'dry_run'
        elif return_status not in {'succeeded', 'disabled'}:
            report['status'] = 'completed_return_failed'
        elif failed or unchecked:
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
