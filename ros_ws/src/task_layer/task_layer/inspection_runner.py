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
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from task_layer.area_clear_check import AreaClearChecker
from task_layer.photo_diff_check import (
    CameraModel,
    detect_changes,
    detect_red_targets,
    merge_photo_detections,
)
from task_layer.report_writer import (
    default_report_dir,
    prune_report_dirs,
    write_markdown_report,
    write_report,
)
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


def shortest_angular_distance(current: float, target: float) -> float:
    """Signed shortest rotation from current yaw to target yaw."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def calibrated_spin_command(
    current: float,
    target: float,
    command_scale: float,
) -> float:
    """Relative Nav2 Spin command for an absolute map-frame target yaw."""
    return shortest_angular_distance(current, target) * command_scale


def direct_heading_and_distance(current_pose: tuple[float, float, float],
                                goal_x: float,
                                goal_y: float) -> tuple[float, float]:
    """Map heading and Euclidean distance for a collision-checked straight leg."""
    dx = float(goal_x) - float(current_pose[0])
    dy = float(goal_y) - float(current_pose[1])
    return math.atan2(dy, dx), math.hypot(dx, dy)


def recoverable_viewpoint_near_miss(
    result: dict,
    continuity_tolerance_m: float,
) -> bool:
    """Whether a safely stopped segmented arrival may continue its scan.

    This deliberately excludes generic Nav2 aborts/timeouts and any action
    whose terminal state was not confirmed.  It only absorbs centimetre-scale
    AMCL boundary jitter after the bounded segmented correction sequence.
    """
    if (result.get('status') != 'segmented_xy_miss'
            or not result.get('safe_to_continue', False)):
        return False
    try:
        error = float(result['xy_error_m'])
        strict_tolerance = float(result['xy_tolerance_m'])
        continuity_tolerance = float(continuity_tolerance_m)
    except (KeyError, TypeError, ValueError):
        return False
    return (math.isfinite(error) and math.isfinite(strict_tolerance)
            and strict_tolerance < error <= continuity_tolerance)


def normalize_text(value: str) -> str:
    return value.strip().lower().replace(' ', '_').replace('-', '_')


def safe_path_name(value: str) -> str:
    return ''.join(char if char.isalnum() or char in {'_', '-'} else '_' for char in value)


def inside_clearance_radius(
    goal_x: float,
    goal_y: float,
    cell_x: float,
    cell_y: float,
    radius: float,
) -> bool:
    """Whether a costmap cell center lies in the configured circular guard."""
    return math.hypot(cell_x - goal_x, cell_y - goal_y) <= radius


def inspection_sector(area: dict, stop_label: str,
                      yaw_index: int) -> dict | None:
    """Return the site-verified semantic sector for one captured view."""
    sectors = area.get('inspection_sectors') or {}
    for sector in sectors.get(stop_label, []):
        if int(sector.get('yaw_index', -1)) == int(yaw_index):
            return sector
    return None


def authored_scan_yaw_indices(area: dict, yaw_count: int) -> list[int]:
    """Return stable authored indices for a full or subset scan.

    A one-heading field check may set ``scan_yaws: [2.0944]`` while that
    heading is still the site's canonical yaw02.  Without the parallel
    ``scan_yaw_indices: [2]`` metadata, baseline files and sector lookup would
    silently call it yaw00 merely because it is first in the temporary list.
    """
    configured = area.get('scan_yaw_indices')
    if configured is None:
        return list(range(yaw_count))
    indices = [int(value) for value in configured]
    if len(indices) != yaw_count:
        raise ValueError(
            'scan_yaw_indices must contain exactly one index per scan_yaw')
    if len(set(indices)) != len(indices) or any(index < 0 for index in indices):
        raise ValueError('scan_yaw_indices must be unique non-negative integers')
    return indices


def classify_sector_zone(sector: dict | None,
                         detection_range_m: float | None) -> str | None:
    """Classify a detection into the observed VP/handling zone.

    Pure sectors have one zone. Mixed sectors use the authored near/far rule
    and its coarse range split; exact map localization is deliberately not a
    requirement for the small-object patrol report.
    """
    if not sector:
        return None
    zones = [str(zone) for zone in sector.get('observed_zones', [])]
    if len(zones) == 1:
        return zones[0]
    rule = str(sector.get('zone_rule', ''))
    split = sector.get('zone_split_range_m')
    if rule.startswith('near_') and '_far_' in rule and split is not None:
        near_zone, far_zone = rule[len('near_'):].split('_far_', 1)
        if detection_range_m is not None:
            return (near_zone if float(detection_range_m) <= float(split)
                    else far_zone)
    return '_or_'.join(zones) if zones else None


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
        # A PositionGoalChecker deliberately ignores viewpoint orientation so
        # patrol arrival is judged by XY only. On the real robot, therefore,
        # same-XY NavigateToPose goals cannot perform the camera sweep. Use the
        # behavior_server Spin action for each absolute map yaw instead. The
        # scale compensates the measured Burger response: a 1.000 rad command
        # produces about 1.047 rad of physical rotation.
        self.declare_parameter('scan_use_spin_action', False)
        self.declare_parameter('scan_spin_action_name', 'spin')
        self.declare_parameter('scan_spin_timeout_sec', 20.0)
        self.declare_parameter('scan_spin_command_scale', 1.0 / 1.0472)
        self.declare_parameter('scan_spin_step_rad', 1.0)
        self.declare_parameter('scan_spin_skip_tolerance_rad', 0.03)
        # A transient TF/footprint miss can make Nav2 terminate a Spin in less
        # than a second even though the action is safely stopped. Retry only a
        # confirmed-terminal result; an unconfirmed goal still latches the
        # mission-wide safety abort and is never followed by another command.
        self.declare_parameter('scan_spin_max_attempts', 1)
        # A workflow may hand the final viewpoint directly to another motion
        # stage.  Restore the first scan heading after the final viewpoint so
        # the handoff starts from a repeatable orientation instead of yaw05.
        self.declare_parameter('restore_final_viewpoint_scan_yaw', False)
        self.declare_parameter('restore_final_viewpoint_scan_yaw_attempts', 1)
        # Real-arena Home -> VP1 is a 0.38 m diagonal that MPPI repeatedly
        # over-corrects. Use Nav2's collision-checked straight behavior for an
        # orthogonal line-turn-line entry, then verify final AMCL XY.
        self.declare_parameter('home_to_vp1_segmented', False)
        self.declare_parameter('segmented_entry_drive_action', '/drive_on_heading')
        self.declare_parameter('segmented_entry_speed_mps', 0.05)
        self.declare_parameter('segmented_entry_timeout_sec', 15.0)
        self.declare_parameter('segmented_entry_home_radius_m', 0.15)
        self.declare_parameter('segmented_entry_xy_tolerance_m', 0.05)
        self.declare_parameter('segmented_entry_pose_wait_sec', 5.0)
        self.declare_parameter('segmented_entry_correction_limit_m', 0.15)
        self.declare_parameter('segmented_entry_max_corrections', 2)
        self.declare_parameter(
            'segmented_entry_realign_threshold_rad', 0.0873)
        # VP2 -> VP3 is an unobstructed ~1.07 m straight leg. After VP2's
        # final scan heading MPPI intermittently recovery-cycles for 120 s
        # instead of acquiring that line. Bypass only this known leg through
        # Nav2's collision-checked Spin + DriveOnHeading behaviors.
        self.declare_parameter('vp2_to_vp3_segmented', False)
        self.declare_parameter('segmented_transit_speed_mps', 0.05)
        self.declare_parameter('segmented_transit_timeout_sec', 35.0)
        self.declare_parameter('segmented_transit_start_radius_m', 0.15)
        self.declare_parameter('segmented_transit_xy_tolerance_m', 0.05)
        self.declare_parameter('segmented_transit_correction_limit_m', 0.25)
        self.declare_parameter('segmented_transit_max_corrections', 2)
        self.declare_parameter(
            'segmented_transit_realign_threshold_rad', 0.0873)
        self.declare_parameter(
            'viewpoint_continuity_tolerance_m', 0.065)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_settle_sec', 1.0)
        # Real robot over WiFi only: a persistent raw-image subscription
        # saturates the robot's uplink and starves /scan below what AMCL
        # needs. True = no standing subscription; each capture subscribes,
        # takes the 3rd frame (first ones can be stale/mid-exposure) and
        # unsubscribes, so navigation runs with zero image traffic.
        self.declare_parameter('image_on_demand', False)
        # A temporal median keeps a stationary 3 cm-class target while
        # rejecting transient compression noise and people crossing a view.
        # The legacy single-frame behavior remains the default for sim.
        self.declare_parameter('image_burst_warmup_frames', 2)
        self.declare_parameter('image_burst_count', 1)
        self.declare_parameter('report_dir', default_report_dir())
        self.declare_parameter('report_keep_runs', 10)
        self.declare_parameter('return_home', True)
        self.declare_parameter('home_area', 'charging_station')
        # Per-robot home override (multi-robot: each robot has its own dock;
        # the world_model robot_start is a single-robot legacy default).
        self.declare_parameter('home_x', float('nan'))
        self.declare_parameter('home_y', float('nan'))
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('return_home_standoff_distance', 0.0)
        # HOME is a validated wall-adjacent dock. Keep a dedicated dynamic-
        # obstacle guard smaller than the generic viewpoint guard; Nav2's
        # footprint/inflation/collision checks remain active throughout.
        self.declare_parameter('return_home_clearance_radius', 0.15)
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
        self.declare_parameter('photo_diff_min_height_px', 15)
        self.declare_parameter('photo_diff_min_width_px', 1)
        self.declare_parameter('photo_diff_morph_kernel_px', 5)
        # Site-verified stop/yaw -> observed-zone reporting. When enabled,
        # sectors disabled by the world model are skipped and detections in
        # ignored zones (notably anomaly_handling) are discarded.
        self.declare_parameter('photo_diff_report_by_sector', False)
        # Phase-specific small-object gate. Empty = generic baseline diff;
        # "red" = the site's graspable red cube, robust to near-wall parallax.
        self.declare_parameter('photo_target_color', '')
        self.declare_parameter('photo_target_min_area_px', 150)
        self.declare_parameter('photo_target_high_hue_min', 175)
        self.declare_parameter('photo_target_high_hue_saturation_min', 130)
        # Beyond ~3.5 m the ground-intersection geometry degrades (a few
        # pixels of bottom-edge error swing the estimate by metres) and the
        # only regions that big are alignment artifacts.
        self.declare_parameter('photo_diff_max_range', 3.5)
        self.declare_parameter('photo_diff_min_range', 0.3)
        # Real arena only: fraction of the frame TOP to ignore in the diff.
        # The physical site's walls are low (~0.35 m vs camera at ~0.25 m),
        # so the upper image rows see past the arena into an uncontrolled
        # room. 0.0 (sim default) = full frame; tune on-site from real
        # captures (start near 0.35) before recording baselines.
        self.declare_parameter('photo_diff_roi_top_frac', 0.0)
        self.declare_parameter('photo_diff_roi_side_frac', 0.0)
        self.declare_parameter('photo_diff_roi_bottom_frac', 0.0)
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
        self._spin_client = ActionClient(
            self, Spin, self.get_parameter('scan_spin_action_name').value)
        self._drive_client = ActionClient(
            self, DriveOnHeading,
            self.get_parameter('segmented_entry_drive_action').value)
        self._latest_scan = None
        self._latest_image = None
        self._last_capture_meta: dict = {}
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
        self._image_topic = self.get_parameter('image_topic').value
        # BEST_EFFORT to match sensor drivers (LDS-02 publishes sensor-data
        # QoS); a best-effort subscription still matches RELIABLE sim
        # publishers, so this is safe in both sim and on the real robot.
        self.create_subscription(LaserScan, scan_topic, self._scan_callback,
                                 qos_profile_sensor_data)
        if not bool(self.get_parameter('image_on_demand').value):
            self.create_subscription(
                Image, self._image_topic, self._image_callback, 10)
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
        # Uncalibrated cameras (v4l2_camera with no calibration file)
        # publish an all-zero K; using it makes the homography/projection
        # math singular. Treat it like "no camera_info" and fall back to
        # the default intrinsics until the real calibration lands.
        if info is None or float(info.k[0]) <= 0.0 or float(info.k[4]) <= 0.0:
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
        sector_mode = bool(self.get_parameter(
            'photo_diff_report_by_sector').value)
        if record:
            recorded = 0
            for yaw_index, sample in stop_samples:
                sector = inspection_sector(area, stop_label, yaw_index)
                if sector_mode and (sector is None or not bool(
                        sector.get('routine_detection_enabled', True))):
                    outcome['views'].append({
                        'yaw_index': yaw_index,
                        'status': 'ignored_sector',
                    })
                    continue
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
                roi_top_frac = float(
                    self.get_parameter('photo_diff_roi_top_frac').value)
                roi_side_frac = float(
                    self.get_parameter('photo_diff_roi_side_frac').value)
                roi_bottom_frac = float(
                    self.get_parameter('photo_diff_roi_bottom_frac').value)
                target.with_suffix('.json').write_text(json.dumps(
                    {
                        'x': pose[0],
                        'y': pose[1],
                        'yaw': pose[2],
                        'image_roi': {
                            'type': 'ignore_top_fraction',
                            'top_fraction': roi_top_frac,
                            'side_fraction': roi_side_frac,
                            'bottom_fraction': roi_bottom_frac,
                        },
                    }))
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
        target_color = normalize_text(str(
            self.get_parameter('photo_target_color').value or ''))
        bounds = self.detect_bounds(area)
        # Site-verified sector mode deliberately reports only the VP region,
        # not a fragile projected map coordinate. The legacy shrunken-bounds
        # clip can discard a real object near an arena edge (v5 VP1/yaw02),
        # so it applies only to the old coordinate-reporting mode.
        clip = (not sector_mode and bool(area.get(
            'photo_detect_clip_bounds',
            self.get_parameter('photo_detect_clip_bounds').value)))
        min_range = float(area.get(
            'photo_detect_min_range',
            self.get_parameter('photo_diff_min_range').value))
        checked = 0
        found: list[dict] = []
        for yaw_index, sample in stop_samples:
            sector = inspection_sector(area, stop_label, yaw_index)
            if sector_mode and (sector is None or not bool(
                    sector.get('routine_detection_enabled', True))):
                outcome['views'].append({
                    'yaw_index': yaw_index,
                    'status': 'ignored_sector',
                })
                continue
            photo = (sample.get('image_capture') or {}).get('image_path')
            base = self.baseline_photo_path(area_key, stop_label, yaw_index)
            if not photo or (target_color != 'red' and not base.exists()):
                continue
            pose = sample.get('pose_at_capture') or (
                stop.get('x', 0.0), stop.get('y', 0.0),
                float(sample.get('yaw', 0.0)))
            base_pose = None
            roi_top_frac = float(
                self.get_parameter('photo_diff_roi_top_frac').value)
            roi_side_frac = float(
                self.get_parameter('photo_diff_roi_side_frac').value)
            roi_bottom_frac = float(
                self.get_parameter('photo_diff_roi_bottom_frac').value)
            base_meta = base.with_suffix('.json')
            if base_meta.exists():
                try:
                    meta = json.loads(base_meta.read_text())
                    base_pose = (meta['x'], meta['y'], meta['yaw'])
                    roi = meta.get('image_roi') or {}
                    if roi.get('type') == 'ignore_top_fraction':
                        # The baseline defines its own evidence region. This
                        # prevents a later parameter change from comparing two
                        # images under different ROI rules.
                        roi_top_frac = float(roi['top_fraction'])
                        roi_side_frac = float(roi.get(
                            'side_fraction', roi_side_frac))
                        roi_bottom_frac = float(roi.get(
                            'bottom_fraction', roi_bottom_frac))
                except (ValueError, KeyError):
                    base_pose = None
            if target_color == 'red':
                detection = detect_red_targets(
                    photo, pose, camera,
                    min_area_px=int(self.get_parameter(
                        'photo_target_min_area_px').value),
                    max_range=float(self.get_parameter(
                        'photo_diff_max_range').value),
                    min_range=min_range,
                    roi_top_frac=roi_top_frac,
                    roi_side_frac=roi_side_frac,
                    roi_bottom_frac=roi_bottom_frac,
                    min_height_px=int(self.get_parameter(
                        'photo_diff_min_height_px').value),
                    min_width_px=int(self.get_parameter(
                        'photo_diff_min_width_px').value),
                    hue_high_min=int(self.get_parameter(
                        'photo_target_high_hue_min').value),
                    high_hue_saturation_min=int(self.get_parameter(
                        'photo_target_high_hue_saturation_min').value))
            else:
                detection = detect_changes(
                    base, photo, pose, camera,
                    threshold=int(self.get_parameter('photo_diff_threshold').value),
                    tolerance_px=int(self.get_parameter('photo_diff_tolerance_px').value),
                    min_area_px=int(self.get_parameter('photo_diff_min_area_px').value),
                    max_range=float(self.get_parameter('photo_diff_max_range').value),
                    baseline_pose=base_pose, min_range=min_range,
                    roi_top_frac=roi_top_frac,
                    roi_side_frac=roi_side_frac,
                    roi_bottom_frac=roi_bottom_frac,
                    morph_kernel_px=int(self.get_parameter(
                        'photo_diff_morph_kernel_px').value),
                    min_height_px=int(self.get_parameter(
                        'photo_diff_min_height_px').value),
                    min_width_px=int(self.get_parameter(
                        'photo_diff_min_width_px').value))
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
                observed_zone = classify_sector_zone(
                    sector, anomaly.get('range')) if sector_mode else None
                if sector_mode:
                    sector_zones = list((sector or {}).get(
                        'observed_zones', []))
                    anomaly['sector_mixed'] = len(sector_zones) > 1
                    split = (sector or {}).get('zone_split_range_m')
                    if split is not None and anomaly.get('range') is not None:
                        anomaly['zone_boundary_margin_m'] = round(abs(
                            float(anomaly['range']) - float(split)), 3)
                    ignored_zones = set(
                        (area.get('inspection_detection_policy') or {}).get(
                            'ignored_zones', []))
                    ignored_zones.update((sector or {}).get(
                        'ignored_zones', []))
                    if observed_zone in ignored_zones:
                        outcome['views'].append({
                            'yaw_index': yaw_index,
                            'status': 'change_ignored_by_zone',
                            'observed_zone': observed_zone,
                            'area_px': anomaly.get('area_px'),
                        })
                        continue
                    if observed_zone:
                        anomaly['observed_zone'] = observed_zone
                        anomaly['area'] = observed_zone
                        anomaly['type'] = ('red_target' if target_color == 'red'
                                           else 'viewpoint_change')
                        anomaly['description'] = (
                            f'{observed_zone}区域发现红色小目标'
                            if target_color == 'red' else
                            f'{observed_zone}区域存在持续视觉变化')
                anomaly['detected_from'] = {
                    'stop': stop_label, 'yaw_index': yaw_index,
                    'photo': photo,
                    'observed_zone': observed_zone}
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
            'area': anomaly.get('observed_zone', area_key),
            'capture_area': area_key,
            'capture_stop': (anomaly.get('detected_from') or {}).get('stop'),
            'capture_yaw_index': (anomaly.get('detected_from') or {}).get(
                'yaw_index'),
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
        zone = str(anomaly.get('observed_zone') or area_key)
        label.text = f"{zone.upper()} ANOMALY"
        self._marker_pub.publish(MarkerArray(markers=[body, label]))
        self.get_logger().warn(
            'ANOMALY %s in %s at (%.2f, %.2f) extent=%.2f'
            % (self._anomaly_seq, area_key, anomaly['x'], anomaly['y'],
               anomaly.get('extent') or 0.0))

    def clear_anomaly_markers(self) -> None:
        """Clear stale RViz points before a new stress-test round.

        Marker IDs restart at one for every short-lived runner process. A
        DELETEALL message prevents a round with fewer anomalies from leaving
        higher-ID points from the previous round on screen.
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = Marker.DELETEALL
        self._marker_pub.publish(MarkerArray(markers=[marker]))

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

    def candidate_is_clear(
        self,
        x: float,
        y: float,
        clearance_radius: float | None = None,
    ) -> bool:
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
        radius = (float(self.get_parameter('candidate_clearance_radius').value)
                  if clearance_radius is None else float(clearance_radius))
        steps = max(0, int(radius / info.resolution))
        base_col = math.floor((x - info.origin.position.x) / info.resolution)
        base_row = math.floor((y - info.origin.position.y) / info.resolution)
        for d_row in range(-steps, steps + 1):
            for d_col in range(-steps, steps + 1):
                col = base_col + d_col
                row = base_row + d_row
                if not (0 <= col < info.width and 0 <= row < info.height):
                    continue
                wx = info.origin.position.x + (col + 0.5) * info.resolution
                wy = info.origin.position.y + (row + 0.5) * info.resolution
                # The index bounds above form a square.  Apply the documented
                # Euclidean radius as well; otherwise diagonal cells as far as
                # radius*sqrt(2) away can falsely cancel a safe goal.
                if not inside_clearance_radius(x, y, wx, wy, radius):
                    continue
                if grid.data[row * info.width + col] < lethal:
                    continue
                # Lethal in the costmap. Only count it as a dynamic obstacle if
                # the static map does NOT explain it (free floor at that spot).
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

    def send_goal_and_wait(
        self,
        goal: NavigateToPose.Goal,
        clearance_radius: float | None = None,
    ) -> dict:
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
                if not self.candidate_is_clear(
                        goal_x, goal_y, clearance_radius=clearance_radius):
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

    def send_spin_to_map_yaw(self, target_yaw: float) -> dict:
        """Rotate in place to an absolute map-frame yaw using Nav2 Spin.

        AMCL yaw can lag immediately after rotation, so the current yaw is
        refined by matching the live laser scan against the static map before
        calculating the relative Spin command.
        """
        wait_deadline = time.time() + 5.0
        while rclpy.ok() and (
            self._own_pose is None or self._latest_scan is None
        ) and time.time() < wait_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._own_pose is None or self._latest_scan is None:
            return {'status': 'sensor_unavailable', 'safe_to_continue': True}

        current_raw = float(self._own_pose[2])
        current_yaw, fit_ratio = self.corrected_capture_yaw(
            float(self._own_pose[0]), float(self._own_pose[1]), current_raw)
        delta = shortest_angular_distance(current_yaw, target_yaw)
        skip_tolerance = float(
            self.get_parameter('scan_spin_skip_tolerance_rad').value)
        if abs(delta) <= skip_tolerance:
            return {
                'status': 'succeeded',
                'mode': 'spin_already_aligned',
                'target_map_yaw': round(float(target_yaw), 4),
                'current_map_yaw': round(float(current_yaw), 4),
                'map_yaw_error': round(float(delta), 4),
                'yaw_fit_ratio': round(float(fit_ratio), 3),
                'safe_to_continue': True,
                'duration_sec': 0.0,
            }

        timeout = float(self.get_parameter('scan_spin_timeout_sec').value)
        if not self._spin_client.wait_for_server(timeout_sec=10.0):
            return {'status': 'spin_server_unavailable',
                    'safe_to_continue': True}
        scale = float(self.get_parameter('scan_spin_command_scale').value)
        command = calibrated_spin_command(current_yaw, target_yaw, scale)
        goal = Spin.Goal()
        goal.target_yaw = float(command)
        goal.time_allowance = Duration(sec=max(1, int(math.ceil(timeout))))
        started = time.time()
        send_future = self._spin_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if (not send_future.done() or send_future.result() is None
                or not send_future.result().accepted):
            return {
                'status': 'spin_rejected',
                'safe_to_continue': True,
                'duration_sec': round(time.time() - started, 3),
            }

        handle = send_future.result()
        result_future = handle.get_result_async()
        terminal = self._await_terminal(result_future, timeout + 5.0)
        if not terminal:
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            terminal = self._await_terminal(result_future, 5.0)
            result = {
                'status': 'spin_timeout',
                'cancel_terminal': 'confirmed' if terminal else 'unconfirmed',
                'safe_to_continue': terminal,
                'duration_sec': round(time.time() - started, 3),
            }
            return self._finish(result)

        action_result = result_future.result()
        status = STATUS_TEXT.get(action_result.status, str(action_result.status))
        return {
            'status': status,
            'mode': 'spin_action',
            'target_map_yaw': round(float(target_yaw), 4),
            'current_map_yaw': round(float(current_yaw), 4),
            'map_yaw_error': round(float(delta), 4),
            'spin_command_rad': round(float(command), 4),
            'yaw_fit_ratio': round(float(fit_ratio), 3),
            'safe_to_continue': True,
            'duration_sec': round(time.time() - started, 3),
        }

    def send_relative_scan_spin(
        self,
        command_rad: float,
        target_yaw: float,
    ) -> dict:
        """Execute one calibrated relative scan step.

        After the first absolute heading is acquired, all remaining scan
        headings use this fixed odometry-relative command. This prevents AMCL
        convergence jumps during rotation from changing the physical spacing
        of subsequent camera views.
        """
        timeout = float(self.get_parameter('scan_spin_timeout_sec').value)
        if not self._spin_client.wait_for_server(timeout_sec=10.0):
            return {'status': 'spin_server_unavailable',
                    'safe_to_continue': True}
        goal = Spin.Goal()
        goal.target_yaw = float(command_rad)
        goal.time_allowance = Duration(sec=max(1, int(math.ceil(timeout))))
        started = time.time()
        send_future = self._spin_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if (not send_future.done() or send_future.result() is None
                or not send_future.result().accepted):
            return {
                'status': 'spin_rejected',
                'safe_to_continue': True,
                'duration_sec': round(time.time() - started, 3),
            }

        handle = send_future.result()
        result_future = handle.get_result_async()
        terminal = self._await_terminal(result_future, timeout + 5.0)
        if not terminal:
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            terminal = self._await_terminal(result_future, 5.0)
            return self._finish({
                'status': 'spin_timeout',
                'cancel_terminal': 'confirmed' if terminal else 'unconfirmed',
                'safe_to_continue': terminal,
                'duration_sec': round(time.time() - started, 3),
            })

        action_result = result_future.result()
        status = STATUS_TEXT.get(action_result.status, str(action_result.status))
        return {
            'status': status,
            'mode': 'spin_relative_calibrated',
            'target_map_yaw': round(float(target_yaw), 4),
            'spin_command_rad': round(float(command_rad), 4),
            'safe_to_continue': True,
            'duration_sec': round(time.time() - started, 3),
        }

    def send_scan_spin_with_retries(
        self,
        first_heading: bool,
        target_yaw: float,
    ) -> dict:
        """Retry a safely terminated transient Spin without skipping safety."""
        max_attempts = max(1, int(
            self.get_parameter('scan_spin_max_attempts').value))
        history = []
        for attempt in range(1, max_attempts + 1):
            if first_heading:
                result = self.send_spin_to_map_yaw(target_yaw)
            else:
                result = self.send_relative_scan_spin(
                    float(self.get_parameter('scan_spin_step_rad').value),
                    target_yaw)
            history.append({'attempt': attempt, **result})
            if result.get('status') == 'succeeded':
                final = dict(result)
                final['attempt'] = attempt
                if attempt > 1:
                    final['retry_history'] = history
                return final
            if not result.get('safe_to_continue', True):
                break
            if attempt < max_attempts:
                self.get_logger().warn(
                    'Scan Spin target yaw %.4f returned terminal status %s; '
                    'waiting for fresh TF/footprint and retrying (%d/%d).'
                    % (target_yaw, result.get('status'),
                       attempt + 1, max_attempts))
                self.wait_for_sensor_settle()
        final = dict(history[-1])
        final['attempts'] = history
        return final

    def send_drive_on_heading(self, distance_m: float,
                              speed_mps: float | None = None,
                              timeout_sec: float | None = None) -> dict:
        """Collision-checked straight motion through Nav2 behavior_server."""
        timeout = (float(timeout_sec) if timeout_sec is not None else float(
            self.get_parameter('segmented_entry_timeout_sec').value))
        if distance_m <= 0.005:
            return {'status': 'succeeded', 'mode': 'drive_already_reached',
                    'distance_m': round(float(distance_m), 4),
                    'safe_to_continue': True, 'duration_sec': 0.0}
        if not self._drive_client.wait_for_server(timeout_sec=10.0):
            return {'status': 'drive_server_unavailable',
                    'safe_to_continue': True}
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance_m)
        goal.speed = (float(speed_mps) if speed_mps is not None else float(
            self.get_parameter('segmented_entry_speed_mps').value))
        goal.time_allowance = Duration(sec=max(1, int(math.ceil(timeout))))
        started = time.time()
        send_future = self._drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if (not send_future.done() or send_future.result() is None
                or not send_future.result().accepted):
            return {'status': 'drive_rejected', 'safe_to_continue': True,
                    'duration_sec': round(time.time() - started, 3)}
        handle = send_future.result()
        result_future = handle.get_result_async()
        terminal = self._await_terminal(result_future, timeout + 5.0)
        if not terminal:
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future,
                                             timeout_sec=5.0)
            terminal = self._await_terminal(result_future, 5.0)
            return self._finish({
                'status': 'drive_timeout',
                'cancel_terminal': 'confirmed' if terminal else 'unconfirmed',
                'safe_to_continue': terminal,
                'distance_m': round(float(distance_m), 4),
                'duration_sec': round(time.time() - started, 3),
            })
        action_result = result_future.result()
        return {
            'status': STATUS_TEXT.get(
                action_result.status, str(action_result.status)),
            'mode': 'drive_on_heading',
            'distance_m': round(float(distance_m), 4),
            'speed_mps': round(float(goal.speed), 3),
            'safe_to_continue': True,
            'duration_sec': round(time.time() - started, 3),
        }

    def home_to_vp1_segmented_result(self, stop: dict) -> dict | None:
        """Home -> VP1 as X-line, in-place turn, then Y-line.

        Returns None when the special entry is disabled/not applicable so the
        caller can fall back to NavigateToPose. Distances use live AMCL after
        every segment, absorbing centimetres of manual Home placement error.
        """
        if not bool(self.get_parameter('home_to_vp1_segmented').value):
            return None
        if str(stop.get('label')) != 'viewpoint_1':
            return None
        # AMCL is a live topic, unlike the latched map/costmap inputs.  A newly
        # started one-shot runner can reach the first stop before its first
        # AMCL callback; immediately returning None here silently selects the
        # slow NavigateToPose fallback even though the robot is at Home.  Give
        # the initial pose a short bounded window to arrive before deciding
        # whether the segmented entry is applicable.
        pose_deadline = time.time() + max(0.0, float(self.get_parameter(
            'segmented_entry_pose_wait_sec').value))
        while self._own_pose is None and time.time() < pose_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._own_pose is None:
            self.get_logger().warn(
                'Home -> VP1 segmented entry unavailable: no AMCL pose')
            return None
        home = (self._world_model.get('robot_start') or {}).get('pose') or {}
        if not all(key in home for key in ('x', 'y', 'yaw')):
            return None
        start = tuple(float(value) for value in self._own_pose)
        if math.hypot(start[0] - float(home['x']),
                      start[1] - float(home['y'])) > float(self.get_parameter(
                          'segmented_entry_home_radius_m').value):
            return None

        started = time.time()
        segments = []
        first_yaw = float(home['yaw'])
        align = self.send_spin_to_map_yaw(first_yaw)
        segments.append({'stage': 'align_home_heading', **align})
        if align.get('status') != 'succeeded' or self._own_pose is None:
            return {'status': 'segmented_align_failed',
                    'safe_to_continue': align.get('safe_to_continue', True),
                    'mode': 'home_to_vp1_segmented', 'segments': segments,
                    'duration_sec': round(time.time() - started, 3)}

        pose = tuple(float(value) for value in self._own_pose)
        cos_first = math.cos(first_yaw)
        if abs(cos_first) < 0.5:
            return {'status': 'segmented_invalid_home_heading',
                    'safe_to_continue': True,
                    'mode': 'home_to_vp1_segmented', 'segments': segments,
                    'duration_sec': round(time.time() - started, 3)}
        first_distance = (float(stop['x']) - pose[0]) / cos_first
        if first_distance < -0.005:
            return None
        first = self.send_drive_on_heading(max(0.0, first_distance))
        segments.append({'stage': 'x_straight', **first})
        if first.get('status') != 'succeeded' or self._own_pose is None:
            return {'status': 'segmented_first_drive_failed',
                    'safe_to_continue': first.get('safe_to_continue', True),
                    'mode': 'home_to_vp1_segmented', 'segments': segments,
                    'duration_sec': round(time.time() - started, 3)}

        pose = tuple(float(value) for value in self._own_pose)
        turn_yaw = (-math.pi / 2.0 if float(stop['y']) < pose[1]
                    else math.pi / 2.0)
        turn = self.send_spin_to_map_yaw(turn_yaw)
        segments.append({'stage': 'corner_turn', **turn})
        if turn.get('status') != 'succeeded' or self._own_pose is None:
            return {'status': 'segmented_turn_failed',
                    'safe_to_continue': turn.get('safe_to_continue', True),
                    'mode': 'home_to_vp1_segmented', 'segments': segments,
                    'duration_sec': round(time.time() - started, 3)}

        pose = tuple(float(value) for value in self._own_pose)
        second_distance = ((float(stop['y']) - pose[1])
                           / math.sin(turn_yaw))
        if second_distance < -0.005:
            return {'status': 'segmented_overshot_corner',
                    'safe_to_continue': True,
                    'mode': 'home_to_vp1_segmented', 'segments': segments,
                    'duration_sec': round(time.time() - started, 3)}
        second = self.send_drive_on_heading(max(0.0, second_distance))
        segments.append({'stage': 'y_straight', **second})
        if second.get('status') != 'succeeded' or self._own_pose is None:
            return {'status': 'segmented_second_drive_failed',
                    'safe_to_continue': second.get('safe_to_continue', True),
                    'mode': 'home_to_vp1_segmented', 'segments': segments,
                    'duration_sec': round(time.time() - started, 3)}

        pose = tuple(float(value) for value in self._own_pose)
        error = math.hypot(pose[0] - float(stop['x']),
                           pose[1] - float(stop['y']))
        tolerance = float(self.get_parameter(
            'segmented_entry_xy_tolerance_m').value)
        correction_limit = float(self.get_parameter(
            'segmented_entry_correction_limit_m').value)
        max_corrections = max(0, int(self.get_parameter(
            'segmented_entry_max_corrections').value))
        # The orthogonal entry deliberately avoids MPPI's long correction
        # cycle, but DriveOnHeading terminates from odometry and can leave a
        # repeatable 6--8 cm AMCL miss.  Correct only a bounded near-goal miss
        # with the same collision-checked Spin + DriveOnHeading primitives.
        # The strict final tolerance is unchanged; a larger miss remains a
        # hard failure instead of being hidden by a relaxed acceptance band.
        for correction_index in range(1, max_corrections + 1):
            if error <= tolerance or error > correction_limit:
                break
            heading, _distance = direct_heading_and_distance(
                pose, float(stop['x']), float(stop['y']))
            align = self.send_spin_to_map_yaw(heading)
            segments.append({
                'stage': f'vp1_correction_align_{correction_index}', **align})
            if align.get('status') != 'succeeded' or self._own_pose is None:
                return {
                    'status': 'segmented_correction_align_failed',
                    'safe_to_continue': align.get('safe_to_continue', True),
                    'mode': 'home_to_vp1_segmented',
                    'segments': segments,
                    'duration_sec': round(time.time() - started, 3),
                }
            pose = tuple(float(value) for value in self._own_pose)
            refined_heading, distance = direct_heading_and_distance(
                pose, float(stop['x']), float(stop['y']))
            # A large in-place turn can make AMCL update XY.  Re-aim once
            # when that update materially changes the live VP1 goal vector;
            # otherwise DriveOnHeading would use the stale pre-spin heading.
            realign_threshold = float(self.get_parameter(
                'segmented_entry_realign_threshold_rad').value)
            heading_shift = abs(shortest_angular_distance(
                pose[2], refined_heading))
            if heading_shift > realign_threshold:
                realign = self.send_spin_to_map_yaw(refined_heading)
                segments.append({
                    'stage': f'vp1_correction_realign_{correction_index}',
                    'post_spin_heading_shift_rad': round(heading_shift, 4),
                    **realign,
                })
                if (realign.get('status') != 'succeeded'
                        or self._own_pose is None):
                    return {
                        'status': 'segmented_correction_realign_failed',
                        'safe_to_continue': realign.get(
                            'safe_to_continue', True),
                        'mode': 'home_to_vp1_segmented',
                        'segments': segments,
                        'duration_sec': round(time.time() - started, 3),
                    }
                pose = tuple(float(value) for value in self._own_pose)
                refined_heading, distance = direct_heading_and_distance(
                    pose, float(stop['x']), float(stop['y']))
            correction = self.send_drive_on_heading(
                distance,
                speed_mps=float(self.get_parameter(
                    'segmented_entry_speed_mps').value),
                timeout_sec=min(15.0, float(self.get_parameter(
                    'segmented_entry_timeout_sec').value)),
            )
            segments.append({
                'stage': f'vp1_correction_drive_{correction_index}',
                'target_heading': round(refined_heading, 4), **correction})
            if (correction.get('status') != 'succeeded'
                    or self._own_pose is None):
                return {
                    'status': 'segmented_correction_drive_failed',
                    'safe_to_continue': correction.get(
                        'safe_to_continue', True),
                    'mode': 'home_to_vp1_segmented',
                    'segments': segments,
                    'duration_sec': round(time.time() - started, 3),
                }
            pose = tuple(float(value) for value in self._own_pose)
            error = math.hypot(pose[0] - float(stop['x']),
                               pose[1] - float(stop['y']))
        return {
            'status': 'succeeded' if error <= tolerance
                      else 'segmented_xy_miss',
            'safe_to_continue': True,
            'mode': 'home_to_vp1_segmented',
            'segments': segments,
            'final_pose': [round(value, 4) for value in pose],
            'xy_error_m': round(error, 4),
            'xy_tolerance_m': tolerance,
            'duration_sec': round(time.time() - started, 3),
        }

    def vp2_to_vp3_segmented_result(self, stop: dict) -> dict | None:
        """Enter VP3 from VP2 by aligning once and driving the clear line.

        This is selected only when AMCL confirms that the robot is still at
        the authored VP2. DriveOnHeading continuously checks the local
        costmap, so a newly introduced obstacle aborts instead of turning this
        into open-loop motion.
        """
        if not bool(self.get_parameter('vp2_to_vp3_segmented').value):
            return None
        if str(stop.get('label')) != 'viewpoint_3' or self._own_pose is None:
            return None
        arena = ((self._world_model.get('areas') or {}).get('arena') or {})
        viewpoints = list(arena.get('viewpoints') or [])
        if len(viewpoints) < 2:
            return None
        vp2 = viewpoints[1]
        start = tuple(float(value) for value in self._own_pose)
        start_error = math.hypot(start[0] - float(vp2['x']),
                                 start[1] - float(vp2['y']))
        start_radius = float(self.get_parameter(
            'segmented_transit_start_radius_m').value)
        if start_error > start_radius:
            return None

        started = time.time()
        segments = []
        heading, _distance = direct_heading_and_distance(
            start, float(stop['x']), float(stop['y']))
        align = self.send_spin_to_map_yaw(heading)
        segments.append({'stage': 'align_vp3_line', **align})
        if align.get('status') != 'succeeded' or self._own_pose is None:
            return {
                'status': 'segmented_align_failed',
                'safe_to_continue': align.get('safe_to_continue', True),
                'mode': 'vp2_to_vp3_segmented',
                'segments': segments,
                'duration_sec': round(time.time() - started, 3),
            }

        # Recompute after the spin so centimetres of localization drift do not
        # turn into a systematic endpoint error over the one-metre leg.
        pose = tuple(float(value) for value in self._own_pose)
        heading, distance = direct_heading_and_distance(
            pose, float(stop['x']), float(stop['y']))
        drive = self.send_drive_on_heading(
            distance,
            speed_mps=float(self.get_parameter(
                'segmented_transit_speed_mps').value),
            timeout_sec=float(self.get_parameter(
                'segmented_transit_timeout_sec').value),
        )
        segments.append({'stage': 'vp2_vp3_straight',
                         'target_heading': round(heading, 4), **drive})
        if drive.get('status') != 'succeeded' or self._own_pose is None:
            return {
                'status': 'segmented_drive_failed',
                'safe_to_continue': drive.get('safe_to_continue', True),
                'mode': 'vp2_to_vp3_segmented',
                'segments': segments,
                'duration_sec': round(time.time() - started, 3),
            }

        tolerance = float(self.get_parameter(
            'segmented_transit_xy_tolerance_m').value)
        correction_limit = float(self.get_parameter(
            'segmented_transit_correction_limit_m').value)
        max_corrections = max(0, int(self.get_parameter(
            'segmented_transit_max_corrections').value))
        pose = tuple(float(value) for value in self._own_pose)
        error = math.hypot(pose[0] - float(stop['x']),
                           pose[1] - float(stop['y']))
        # DriveOnHeading holds an odometric heading, but the one-metre real
        # leg can accumulate ~0.13 m of lateral AMCL error.  Correct only a
        # bounded near-goal miss: re-aim from the live pose and make a short,
        # collision-checked leg.  A larger miss remains a hard failure.
        for correction_index in range(1, max_corrections + 1):
            if error <= tolerance or error > correction_limit:
                break
            heading, distance = direct_heading_and_distance(
                pose, float(stop['x']), float(stop['y']))
            align = self.send_spin_to_map_yaw(heading)
            segments.append({
                'stage': f'vp3_correction_align_{correction_index}', **align})
            if align.get('status') != 'succeeded' or self._own_pose is None:
                return {
                    'status': 'segmented_correction_align_failed',
                    'safe_to_continue': align.get('safe_to_continue', True),
                    'mode': 'vp2_to_vp3_segmented',
                    'segments': segments,
                    'duration_sec': round(time.time() - started, 3),
                }
            pose = tuple(float(value) for value in self._own_pose)
            refined_heading, distance = direct_heading_and_distance(
                pose, float(stop['x']), float(stop['y']))
            # AMCL can shift XY by several centimetres while the robot spins,
            # especially beside the VP3 wall.  The old code recomputed only
            # the distance after that shift and then drove along the stale
            # pre-spin heading.  In the 2026-08-11 failure this turned a 6 cm
            # correction into another 5.73 cm miss.  Re-aim once when the
            # live post-spin vector differs materially; collision checking is
            # preserved because this remains a Nav2 Spin action.
            realign_threshold = float(self.get_parameter(
                'segmented_transit_realign_threshold_rad').value)
            heading_shift = abs(shortest_angular_distance(
                pose[2], refined_heading))
            if heading_shift > realign_threshold:
                realign = self.send_spin_to_map_yaw(refined_heading)
                segments.append({
                    'stage': f'vp3_correction_realign_{correction_index}',
                    'post_spin_heading_shift_rad': round(heading_shift, 4),
                    **realign,
                })
                if (realign.get('status') != 'succeeded'
                        or self._own_pose is None):
                    return {
                        'status': 'segmented_correction_realign_failed',
                        'safe_to_continue': realign.get(
                            'safe_to_continue', True),
                        'mode': 'vp2_to_vp3_segmented',
                        'segments': segments,
                        'duration_sec': round(time.time() - started, 3),
                    }
                pose = tuple(float(value) for value in self._own_pose)
                refined_heading, distance = direct_heading_and_distance(
                    pose, float(stop['x']), float(stop['y']))
            correction = self.send_drive_on_heading(
                distance,
                speed_mps=float(self.get_parameter(
                    'segmented_transit_speed_mps').value),
                timeout_sec=min(15.0, float(self.get_parameter(
                    'segmented_transit_timeout_sec').value)),
            )
            segments.append({
                'stage': f'vp3_correction_drive_{correction_index}',
                'target_heading': round(refined_heading, 4), **correction})
            if (correction.get('status') != 'succeeded'
                    or self._own_pose is None):
                return {
                    'status': 'segmented_correction_drive_failed',
                    'safe_to_continue': correction.get(
                        'safe_to_continue', True),
                    'mode': 'vp2_to_vp3_segmented',
                    'segments': segments,
                    'duration_sec': round(time.time() - started, 3),
                }
            pose = tuple(float(value) for value in self._own_pose)
            error = math.hypot(pose[0] - float(stop['x']),
                               pose[1] - float(stop['y']))
        return {
            'status': 'succeeded' if error <= tolerance
                      else 'segmented_xy_miss',
            'safe_to_continue': True,
            'mode': 'vp2_to_vp3_segmented',
            'segments': segments,
            'start_error_m': round(start_error, 4),
            'final_pose': [round(value, 4) for value in pose],
            'xy_error_m': round(error, 4),
            'xy_tolerance_m': tolerance,
            'duration_sec': round(time.time() - started, 3),
        }

    def collect_scan_sample(
        self,
        x: float,
        y: float,
        yaw: float,
        area_key: str,
        area_dir: Path,
        index: int,
        motion_result: dict | None = None,
    ) -> dict:
        if motion_result is None:
            goal = self.build_goal(x, y, yaw)
            motion_result = self.send_goal_and_wait(goal)
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
            'turn_result': motion_result,
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
        scan_yaw_indices = authored_scan_yaw_indices(area, len(scan_yaws))
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
            'scan_yaw_indices': scan_yaw_indices,
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
            if viewpoint_mode:
                # Enter the stop through its AUTHORED yaw before sweeping:
                # the travel goal then matches the pose the viewpoint was
                # validated with. Travelling straight to (x, y, scan_yaws[0])
                # changes MPPI's final-approach geometry enough to stall on
                # wall-adjacent stops (real arena: face-center entry passed
                # 5/5 while the yaw=0 entry recovery-cycled to timeout).
                entry_result = self.home_to_vp1_segmented_result(stop)
                if entry_result is None:
                    entry_result = self.vp2_to_vp3_segmented_result(stop)
                if entry_result is None:
                    entry_result = self.send_goal_and_wait(self.build_goal(
                        stop['x'], stop['y'], float(stop.get('yaw', 0.0))))
                result['nav_attempts'].append(
                    {'label': stop_label, 'x': stop['x'], 'y': stop['y'],
                     'entry': True, 'result': entry_result})
                if self._nav_aborted:
                    result['scan_summary'] = aggregate_scan_summaries(
                        result['scan_samples'])
                    result['status'] = 'nav_aborted'
                    result['reason'] = 'nav_goal_unconfirmed_terminal'
                    return result
                continuity_tolerance = float(self.get_parameter(
                    'viewpoint_continuity_tolerance_m').value)
                if recoverable_viewpoint_near_miss(
                        entry_result, continuity_tolerance):
                    strict_status = entry_result['status']
                    entry_result['strict_arrival_status'] = strict_status
                    entry_result['status'] = 'succeeded_near_tolerance'
                    entry_result['continuity_override'] = True
                    warning = {
                        'stop': stop_label,
                        'strict_status': strict_status,
                        'xy_error_m': entry_result.get('xy_error_m'),
                        'strict_xy_tolerance_m': entry_result.get(
                            'xy_tolerance_m'),
                        'continuity_tolerance_m': continuity_tolerance,
                    }
                    result.setdefault('continuity_warnings', []).append(warning)
                    self.get_logger().warn(
                        '%s %s arrival missed strict XY by %.3f m but is '
                        'safely stopped within %.3f m; continuing scan for '
                        'workflow continuity'
                        % (area_key, stop_label,
                           float(entry_result['xy_error_m']) -
                           float(entry_result['xy_tolerance_m']),
                           continuity_tolerance))
                # A required viewpoint is evidence geometry, not merely a
                # convenient navigation waypoint. Never rotate/capture after a
                # timeout, cancellation or abort: photos taken from an unknown
                # XY silently poison both sector labels and projected marker
                # positions. The mission-level policy will mark this round as
                # failed and attempt the normal return-home path.
                if entry_result.get('status') not in {
                        'succeeded', 'succeeded_near_tolerance'}:
                    result['scan_summary'] = aggregate_scan_summaries(
                        result['scan_samples'])
                    result['status'] = 'nav_failed'
                    result['reason'] = 'required_viewpoint_entry_failed'
                    result['failed_stop'] = {
                        'label': stop_label,
                        'x': stop['x'],
                        'y': stop['y'],
                        'nav_status': entry_result.get('status'),
                    }
                    return result
            for sequence_yaw_index, (yaw_index, yaw) in enumerate(zip(
                    scan_yaw_indices, scan_yaws)):
                sample_seq += 1
                self.get_logger().info(
                    'Inspecting %s/%s yaw=%.4f' % (area_key, stop_label, yaw))
                motion_result = None
                if (viewpoint_mode and bool(
                        self.get_parameter('scan_use_spin_action').value)):
                    motion_result = self.send_scan_spin_with_retries(
                        sequence_yaw_index == 0, yaw)
                    if motion_result.get('status') != 'succeeded':
                        result['scan_summary'] = aggregate_scan_summaries(
                            result['scan_samples'])
                        result['status'] = 'scan_failed'
                        result['reason'] = motion_result.get('status')
                        result['failed_scan'] = {
                            'stop': stop_label,
                            'yaw_index': yaw_index,
                            'yaw': round(yaw, 4),
                            'result': motion_result,
                        }
                        return result
                sample = self.collect_scan_sample(
                    stop['x'],
                    stop['y'],
                    yaw,
                    area_key,
                    area_dir,
                    sample_seq,
                    motion_result=motion_result,
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

        if (viewpoint_mode and stops and scan_yaws and bool(
                self.get_parameter(
                    'restore_final_viewpoint_scan_yaw').value)):
            attempts = max(1, int(self.get_parameter(
                'restore_final_viewpoint_scan_yaw_attempts').value))
            restore_history = []
            for attempt in range(1, attempts + 1):
                restore = self.send_spin_to_map_yaw(scan_yaws[0])
                restore_history.append({'attempt': attempt, **restore})
                if restore.get('status') == 'succeeded':
                    break
                if not restore.get('safe_to_continue', True):
                    break
                self.get_logger().warn(
                    'Final viewpoint heading restore attempt %d/%d returned '
                    '%s; goal is terminal, retrying without aborting the '
                    'completed inspection.' % (
                        attempt, attempts, restore.get('status')))
            result['final_scan_heading_restore'] = {
                'target_yaw': round(float(scan_yaws[0]), 4),
                'status': restore_history[-1].get('status'),
                'attempts': restore_history,
            }

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
                'arrival_orientation_required': True,
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
                'arrival_orientation_required': bool(
                    robot_start.get('arrival_orientation_required', False)),
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
                    'arrival_orientation_required': False,
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
        result['arrival_orientation_required'] = bool(
            pose.get('arrival_orientation_required', False))
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
        nav_result = self.send_goal_and_wait(
            self.build_goal(pose['x'], pose['y'], pose['yaw']),
            clearance_radius=float(self.get_parameter(
                'return_home_clearance_radius').value),
        )
        if (nav_result.get('status') == 'succeeded'
                and pose.get('arrival_orientation_required')):
            self.wait_for_sensor_settle()
            orientation_result = self.send_spin_to_map_yaw(float(pose['yaw']))
            result['orientation_result'] = orientation_result
            nav_result['orientation_result'] = orientation_result
            if orientation_result.get('status') != 'succeeded':
                nav_result['xy_status'] = 'succeeded'
                nav_result['status'] = 'home_orientation_failed'
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
        self.clear_anomaly_markers()

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
        failed = [area for area in report['areas']
                  if area.get('status') in {'nav_failed', 'scan_failed'}]
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

        # Task 5.2 — two-file report layout: details.yaml (full machine) + report.md (bilingual).
        # 1. Build the summary from the report dict (details_path is set to details.yaml).
        summary_report = self.build_summary_report(report, run_dir / 'details.yaml')
        # 2. Embed the summary so the allocator can read it from details.yaml without report.yaml.
        report['summary_report'] = summary_report
        # 3. Write the full report (now includes summary_report) as details.yaml.
        write_report(report, run_dir, filename='details.yaml')
        # 4. Write the bilingual Markdown summary.
        md_path = write_markdown_report(summary_report, run_dir, 'report.md')
        # 5. Log exactly the required prefix so the GUI can grep it; point to report.md.
        self.get_logger().info('Inspection report written: %s' % md_path)
        # 6. report.yaml is no longer written. Retention: keep 10 newest runs.
        if not dry_run:
            prune_report_dirs(
                run_dir.parent, 'inspection_*',
                keep=max(1, int(self.get_parameter('report_keep_runs').value)))
        return 0 if report['status'] in {'completed', 'dry_run'} else 5

    def capture_nav_fail_evidence(self, area_key: str, area_dir: Path, attempt_index: int) -> dict | None:
        if not bool(self.get_parameter('capture_nav_fail_evidence').value):
            return None
        capture = self.capture_named_image(area_dir, f'nav_fail_attempt_{attempt_index:02d}')
        capture['attempt_index'] = attempt_index
        capture['description'] = 'Camera evidence captured immediately after a failed Nav2 attempt.'
        return capture

    def _grab_image_on_demand(self) -> None:
        """Momentary subscription (WiFi-constrained real robot): pull a short
        burst at the stop, take its temporal median, then unsubscribe. Uses the
        COMPRESSED stream and decodes locally: a raw 640x480 frame
        (~900 KB) frequently cannot cross the campus WiFi uplink at all,
        while the ~50 KB JPEG arrives in well under a second."""
        import cv2
        import numpy as np
        self._latest_image = None
        self._last_capture_meta = {}
        frames: list[CompressedImage] = []
        warmup_count = max(0, int(self.get_parameter(
            'image_burst_warmup_frames').value))
        burst_count = max(1, int(self.get_parameter(
            'image_burst_count').value))
        target_count = warmup_count + burst_count
        sub = self.create_subscription(
            CompressedImage, self._image_topic + '/compressed',
            lambda m: frames.append(m), qos_profile_sensor_data)
        deadline = time.time() + 15.0
        while len(frames) < target_count and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if not frames:
            return
        selected = frames[-min(burst_count, len(frames)):]
        arrays = [cv2.imdecode(
            np.frombuffer(bytes(frame.data), np.uint8), cv2.IMREAD_COLOR)
            for frame in selected]
        arrays = [array for array in arrays if array is not None]
        if not arrays or any(array.shape != arrays[0].shape for array in arrays):
            return
        array = np.median(np.stack(arrays), axis=0).astype(np.uint8)
        image = Image()
        image.header = selected[-1].header
        image.height, image.width = array.shape[:2]
        image.encoding = 'bgr8'
        image.step = image.width * 3
        image.data = array.tobytes()
        self._latest_image = image
        self._last_capture_meta = {
            'capture_mode': ('temporal_median' if len(arrays) > 1
                             else 'single_frame'),
            'burst_frames_requested': burst_count,
            'burst_frames_used': len(arrays),
            'warmup_frames_requested': warmup_count,
        }

    def capture_named_image(self, area_dir: Path, stem: str) -> dict:
        self._last_capture_meta = {}
        if bool(self.get_parameter('image_on_demand').value):
            self._grab_image_on_demand()
        else:
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
        result = {
            'status': 'captured',
            'image_path': str(image_path),
            'encoding': self._latest_image.encoding,
        }
        result.update(self._last_capture_meta)
        return result

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
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
