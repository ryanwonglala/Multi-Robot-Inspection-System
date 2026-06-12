#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime, timezone
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import yaml

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
        self.declare_parameter('max_candidate_attempts_per_area', 2)
        self.declare_parameter('capture_nav_fail_evidence', True)
        self.declare_parameter('scan_yaws', [0.0, 1.5708, 3.1416, -1.5708])
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

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

    def _image_callback(self, msg: Image):
        self._latest_image = msg

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

    def send_goal_and_wait(self, goal: NavigateToPose.Goal) -> dict:
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
        rclpy.spin_until_future_complete(self, result_future)
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

        selected = None
        attempt_limit = int(self.get_parameter('max_candidate_attempts_per_area').value)
        candidates_to_try = candidates if attempt_limit <= 0 else candidates[:attempt_limit]
        for candidate in candidates_to_try:
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

            evidence = self.capture_nav_fail_evidence(area_key, area_dir, len(result['nav_attempts']))
            if evidence:
                attempt['nav_fail_evidence'] = evidence
                result['nav_fail_evidence'].append(evidence)

        if selected is None:
            result['status'] = 'nav_failed'
            result['reason'] = (
                'candidate_attempt_limit_reached'
                if len(candidates_to_try) < len(candidates) else
                'all_attempted_candidate_poses_failed'
            )
            result['scan_summary'] = aggregate_scan_summaries([])
            return result

        result['selected_pose'] = selected
        for index, yaw in enumerate(scan_yaws, start=1):
            self.get_logger().info('Inspecting %s yaw=%.4f' % (area_key, yaw))
            sample = self.collect_scan_sample(
                selected['x'],
                selected['y'],
                yaw,
                area_key,
                area_dir,
                index,
            )
            result['scan_samples'].append(sample)

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
            area_summary = {
                'sequence_index': area.get('sequence_index'),
                'area': area.get('target_area'),
                'display_name': area.get('display_name'),
                'status': area.get('status'),
                'evidence_dir': area.get('evidence_dir'),
                'captured_image_count': len(all_image_paths),
                'image_paths': all_image_paths,
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
            'areas': areas,
            'return_home': {
                'attempted': return_home.get('attempted'),
                'target': return_home.get('target'),
                'status': return_result.get('status'),
            },
            'details_file': str(details_path),
            'notes': [
                'This v0.2 report records inspection execution and photo evidence only.',
                'No LiDAR or visual anomaly judgment is included in this report.',
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
        report['summary'].update({
            'checked_count': len(checked),
            'failed_count': len(failed),
            'unchecked_count': len(unchecked),
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
