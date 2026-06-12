#!/usr/bin/env python3
"""Split an inspection route across robots and run one inspection_runner per
robot (subprocess + namespace; becomes an action-client fan-out in v0.4).

Usage:
  ros2 run task_layer task_allocator.py --ros-args \
      -p route:='storage_area,utility_area,server_room,central_hall'

Exit codes: 0 = all robots finished OK, 2 = bad input, 5 = some robot failed.
"""
from __future__ import annotations

from datetime import datetime, timezone
import math
from pathlib import Path
import shutil
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import yaml

from task_layer.report_writer import default_report_dir


def default_share(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('task_layer')) / 'config' / name)


def export_anomaly_evidence(mission_dir: Path, anomalies: list[dict]) -> None:
    """Gather every anomaly's evidence photo into mission_dir/anomaly_evidence/
    (converted to PNG when PIL is available, raw copy otherwise) and point the
    anomaly entry at the copy. The original stays in the per-robot run dir."""
    with_photo = [a for a in anomalies if a.get('evidence_photo')]
    if not with_photo:
        return
    out_dir = mission_dir / 'anomaly_evidence'
    out_dir.mkdir(parents=True, exist_ok=True)
    for index, anomaly in enumerate(with_photo, start=1):
        src = Path(anomaly['evidence_photo'])
        if not src.exists():
            continue
        stem = f"{index:02d}_{anomaly.get('robot', 'robot')}_{anomaly.get('area', 'area')}"
        try:
            from PIL import Image as PILImage
            dst = out_dir / f'{stem}.png'
            PILImage.open(src).save(dst)
        except Exception:  # noqa: BLE001  (no PIL / odd encoding: keep raw)
            dst = out_dir / (stem + src.suffix)
            try:
                shutil.copy2(src, dst)
            except OSError:
                continue
        anomaly['evidence_photo'] = str(dst)


def build_summary_text(mission: dict) -> str:
    """One-screen plain-language digest written next to report.yaml: the
    operator reads this first and only opens the yaml for forensics."""
    info = mission.get('mission', {})
    robots = mission.get('robots', {})
    anomalies = mission.get('anomalies', [])
    confirmed = [a for a in anomalies if a.get('type') != 'center_relocated_prior']
    priors = [a for a in anomalies if a.get('type') == 'center_relocated_prior']
    status_cn = {
        'completed': '全部完成',
        'completed_with_failures': '有失败项（细节见 report.yaml）',
    }
    return_cn = {'succeeded': '已回桩', 'failed': '回桩失败',
                 'skipped': '未回桩', 'disabled': '未启用回桩'}

    lines = ['RoboInspect 巡检速览', '=' * 46]
    lines.append('时间: %s    用时: %ss' % (info.get('generated_at', '?'),
                                          info.get('duration_sec', '?')))
    lines.append('结论: %s    异常: %d 处' % (
        status_cn.get(info.get('status'), str(info.get('status'))),
        len(confirmed)))
    lines.append('')
    lines.append('—— 路线分配与完成情况 ——')
    for ns, entry in robots.items():
        areas = entry.get('allocated_areas') or []
        if not areas:
            lines.append(f'{ns}: （本次未分配任务）')
            continue
        lines.append('%s: %s ｜ 完成 %s ｜ %s' % (
            ns, ', '.join(areas), entry.get('checked', '?'),
            return_cn.get(entry.get('return_home'),
                          str(entry.get('return_home')))))

    lines.append('')
    lines.append('—— 异常清单 ——')
    if not confirmed:
        lines.append('未发现异常。')
    used_priors: set[int] = set()
    for index, anomaly in enumerate(confirmed, start=1):
        lines.append('%d. %s (%.2f, %.2f)  尺寸≈%.2fm    发现者: %s' % (
            index, anomaly.get('area', '?'),
            float(anomaly.get('x', 0.0)), float(anomaly.get('y', 0.0)),
            float(anomaly.get('extent') or 0.0), anomaly.get('robot', '?')))
        if anomaly.get('evidence_photo'):
            lines.append('   照片: %s' % anomaly['evidence_photo'])
        for j, prior in enumerate(priors):
            if (j in used_priors or prior.get('robot') != anomaly.get('robot')
                    or prior.get('area') != anomaly.get('area')):
                continue
            src = prior.get('from') or {}
            lines.append('   线索: 原定巡检点 %s(%.2f, %.2f) 被占，改在 %s 完成检测'
                         % (src.get('label', '?'), float(src.get('x', 0.0)),
                            float(src.get('y', 0.0)), prior.get('to', '?')))
            used_priors.add(j)
    leftover = [p for j, p in enumerate(priors) if j not in used_priors]
    if leftover:
        lines.append('')
        lines.append('—— 待核实线索（巡检点被占但未确认异常物体，建议人工查看）——')
        for prior in leftover:
            src = prior.get('from') or {}
            lines.append('%s/%s: 点位 %s(%.2f, %.2f) 被占（%s），改址 %s' % (
                prior.get('robot', '?'), prior.get('area', '?'),
                src.get('label', '?'), float(src.get('x', 0.0)),
                float(src.get('y', 0.0)), prior.get('reason', '?'),
                prior.get('to', '?')))

    issues = []
    skip_cn = {'skipped_uncertain_pose': '定位不稳',
               'skipped_misaligned': '扫描与地图对不齐'}
    for ns, entry in robots.items():
        if entry.get('status') == 'no_report':
            issues.append(f'{ns}: 未产出报告（进程异常，看 allocator_run.log）')
        for area in entry.get('areas') or []:
            name = area.get('area', '?')
            if area.get('status') != 'checked':
                issues.append('%s/%s: 区域未完成（%s）'
                              % (ns, name, area.get('status')))
            clear_status = area.get('area_clear_status') or ''
            if clear_status.startswith('skipped'):
                issues.append('%s/%s: 到点异常检测跳过（%s）——只有环拍照片，'
                              '该区异常可能漏检' % (ns, name,
                              skip_cn.get(clear_status, clear_status)))
        if entry.get('return_home') == 'failed':
            issues.append(f'{ns}: 回桩失败')
    lines.append('')
    lines.append('—— 需要注意 ——')
    lines.extend(issues or ['无'])
    lines.append('')
    lines.append('完整细节: report.yaml    异常照片: anomaly_evidence/')
    return '\n'.join(lines) + '\n'


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.declare_parameter('route', '')
        self.declare_parameter('robots_yaml', default_share('robots.yaml'))
        self.declare_parameter('world_model_path', default_share('world_model.yaml'))
        self.declare_parameter('report_dir', default_report_dir())
        self.declare_parameter('pose_wait_sec', 5.0)
        self.declare_parameter('return_home', True)
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        with open(self.get_parameter('robots_yaml').value, encoding='utf-8') as f:
            registry = yaml.safe_load(f)
        self.robots = registry['robots']
        self.home_gate = registry.get('home_gate') or {}
        with open(self.get_parameter('world_model_path').value, encoding='utf-8') as f:
            self.world_model = yaml.safe_load(f)

        self._plan_clients: dict = {}   # ns -> ActionClient | False (unavailable)
        self._cost_cache: dict = {}     # (ns, start_xy, area) -> meters

        # AMCL latches its last pose (transient_local); a default volatile
        # subscription would never see it for a robot that is standing still.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_poses = {}
        for ns, info in self.robots.items():
            self.create_subscription(
                PoseWithCovarianceStamped, info['amcl_pose_topic'],
                lambda msg, ns=ns: self.robot_poses.__setitem__(
                    ns, (msg.pose.pose.position.x, msg.pose.pose.position.y)),
                latched)

    def area_center(self, area_key: str) -> tuple[float, float]:
        center = self.world_model['areas'][area_key]['center']
        return float(center[0]), float(center[1])

    def wait_for_poses(self):
        deadline = time.time() + float(self.get_parameter('pose_wait_sec').value)
        while time.time() < deadline and len(self.robot_poses) < len(self.robots):
            rclpy.spin_once(self, timeout_sec=0.1)
        # Robots without a live amcl_pose fall back to their home area center.
        for ns, info in self.robots.items():
            if ns not in self.robot_poses:
                home = info.get('home_pose') or {}
                if {'x', 'y'} <= home.keys():
                    self.robot_poses[ns] = (float(home['x']), float(home['y']))
                else:
                    self.robot_poses[ns] = self.area_center(info['home_area'])
                self.get_logger().warn(
                    f'{ns}: no amcl_pose received, assuming home position')

    def path_length(self, ns: str, start: tuple, goal: tuple) -> float | None:
        """Planner-reported path length in meters, or None when the robot's
        planner is unavailable or finds no path."""
        client = self._plan_clients.get(ns)
        if client is False:
            return None
        if client is None:
            action_name = (self.robots[ns]['nav_action'].rsplit('/', 1)[0]
                           + '/compute_path_to_pose')
            client = ActionClient(self, ComputePathToPose, action_name)
            if not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    f'{ns}: planner unavailable, using straight-line distances')
                self._plan_clients[ns] = False
                return None
            self._plan_clients[ns] = client
        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = float(start[0])
        goal_msg.start.pose.position.y = float(start[1])
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = float(goal[0])
        goal_msg.goal.pose.position.y = float(goal[1])
        goal_msg.goal.pose.orientation.w = 1.0
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        result = result_future.result()
        if result is None:
            return None
        poses = result.result.path.poses
        if len(poses) < 2:
            return None
        return sum(
            math.hypot(b.pose.position.x - a.pose.position.x,
                       b.pose.position.y - a.pose.position.y)
            for a, b in zip(poses, poses[1:]))

    def travel_cost(self, ns: str, start: tuple, area_key: str) -> float:
        """Real path length from start to the area center (walls count);
        straight-line fallback keeps allocation alive without a planner."""
        key = (ns, start, area_key)
        if key not in self._cost_cache:
            goal = self.area_center(area_key)
            length = self.path_length(ns, start, goal)
            if length is None:
                length = math.hypot(start[0] - goal[0], start[1] - goal[1])
            self._cost_cache[key] = length
        return self._cost_cache[key]

    def allocate(self, route: list[str]) -> dict[str, list[str]]:
        """Cheapest (robot, area) pair first, repeated until the route is
        exhausted. Costs are planner path lengths from each robot's *virtual*
        position (it moves onto an area once assigned), so the split ignores
        the order the operator picked the rooms in and walls between a robot
        and a room count at their detour cost. The quota (ceil(N/robots))
        stops the greedy cascade where one robot that is 'on the way'
        swallows the whole route while the others idle."""
        quota = math.ceil(len(route) / max(len(self.robots), 1))
        cursor = dict(self.robot_poses)
        plan: dict[str, list[str]] = {ns: [] for ns in self.robots}
        remaining = list(route)
        while remaining:
            candidates = [ns for ns in cursor if len(plan[ns]) < quota]
            _, best, area_key = min(
                (self.travel_cost(ns, cursor[ns], area), ns, area)
                for ns in candidates for area in remaining)
            plan[best].append(area_key)
            cursor[best] = self.area_center(area_key)
            remaining.remove(area_key)
        return plan

    def run_once(self) -> int:
        route = [item.strip() for item in
                 str(self.get_parameter('route').value).replace(';', ',').split(',')
                 if item.strip()]
        if not route:
            self.get_logger().error("Parameter 'route' is required")
            return 2
        unknown = [a for a in route if a not in self.world_model['areas']]
        if unknown:
            self.get_logger().error(f'Unknown areas: {unknown}')
            return 2
        walled = [a for a in route
                  if not self.world_model['areas'][a].get('accessible', True)]
        if walled:
            self.get_logger().error(f'Walled-off areas in route: {walled}')
            return 2

        self.wait_for_poses()
        plan = self.allocate(route)
        for ns, areas in plan.items():
            self.get_logger().info(f'Allocation: {ns} -> {areas or "(idle)"}')

        use_sim_time = str(bool(self.get_parameter('use_sim_time').value)).lower()
        started_at = time.time()
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        # One directory per dispatch: every robot's run lands under it and
        # the merged human-readable report sits on top.
        mission_dir = Path(self.get_parameter('report_dir').value) / f'mission_{timestamp}'
        procs = {}
        launch_order = [ns for ns, areas in plan.items() if areas]
        for index, ns in enumerate(launch_order):
            areas = plan[ns]
            report_dir = mission_dir / ns
            report_dir.mkdir(parents=True, exist_ok=True)
            # Runners never return home on their own: the allocator owns the
            # end-of-mission return so it can arbitrate the shared doorway
            # into the mother_base bay (unmanaged simultaneous returns wedged
            # each other in that opening). See return_all_home().
            command = [
                'ros2', 'run', 'task_layer', 'inspection_runner.py', '--ros-args',
                '-r', f'__ns:=/{ns}',
                '-p', f'use_sim_time:={use_sim_time}',
                '-p', f"route:={','.join(areas)}",
                '-p', 'return_home:=false',
                '-p', f'report_dir:={report_dir}',
            ]
            log_file = open(report_dir / 'allocator_run.log', 'w', encoding='utf-8')
            procs[ns] = (subprocess.Popen(
                command, stdout=log_file, stderr=subprocess.STDOUT, text=True), log_file)
            self.get_logger().info(f'{ns}: inspecting {areas}')
            # Staggered departure: outbound robots share the single
            # mother_base doorway just like returning ones, and simultaneous
            # launches brushed/collided around the docks. Hold the next
            # launch until this robot has threaded the gate outbound.
            if index < len(launch_order) - 1:
                self.wait_departed(ns)

        codes = {}
        for ns, (process, log_file) in procs.items():
            codes[ns] = process.wait()
            log_file.close()
            self.get_logger().info(f'{ns}: finished with code {codes[ns]}')

        return_results: dict[str, bool] = {}
        if bool(self.get_parameter('return_home').value):
            return_results = self.return_all_home(list(procs))
            for ns, ok in return_results.items():
                if not ok:
                    codes[ns] = codes.get(ns) or 6

        report_path = self.write_mission_report(
            mission_dir, route, plan, codes, return_results, started_at)
        self.get_logger().info(f'Mission report written: {report_path}')
        return 0 if all(code == 0 for code in codes.values()) else 5

    def wait_departed(self, ns: str, timeout_sec: float = 90.0):
        """Block until `ns` has passed the mother_base gate outbound (farther
        from its dock than the gate is, and outside the gate zone), or until
        timeout. Robots already away from their dock pass instantly."""
        gate_x = float(self.home_gate.get('x', -1.65))
        gate_y = float(self.home_gate.get('y', -3.3))
        gate_radius = float(self.home_gate.get('radius', 1.0))
        home = self.robots[ns].get('home_pose') or {}
        if not {'x', 'y'} <= home.keys():
            time.sleep(10.0)  # no dock to measure from: fixed stagger
            return
        hx, hy = float(home['x']), float(home['y'])
        gate_to_home = math.hypot(gate_x - hx, gate_y - hy)
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self.robot_poses.get(ns)
            if pose is None:
                continue
            gate_dist = math.hypot(pose[0] - gate_x, pose[1] - gate_y)
            home_dist = math.hypot(pose[0] - hx, pose[1] - hy)
            if gate_dist > gate_radius and home_dist > gate_to_home:
                self.get_logger().info(f'{ns}: cleared the dock gate')
                return
        self.get_logger().warn(
            f'{ns}: departure not confirmed after {timeout_sec:.0f}s, '
            'launching the next robot anyway')

    MISSION_REPORT_GUIDE = """\
# ==========================================================================
# RoboInspect 联合巡检报告（task_allocator 在全部机器人结束后自动汇总）
#
# 想快速看结论：读同目录 SUMMARY.txt（一屏速览）；本文件是完整记录。
# 异常取证照已集中拷贝到同目录 anomaly_evidence/，按"序号_机器人_区域"命名。
#
# 怎么读：
#   mission.status   整体结论：completed = 全部区域完成且全部回桩；
#                    completed_with_failures = 有失败项，到 robots 段找原因
#   allocation       本次路线如何拆给各机器人（按真实路径代价就近分配）
#   robots.<机器人>.areas   每个被巡检区域一条：
#       status: checked    = 已到点完成 360° 环拍取证
#               nav_failed = 尝试了多个候选点仍到不了（路径被堵或区域被占）
#               unchecked  = 区域边界内找不到可用观测点
#       photos             = 该区域拍到的照片张数
#       evidence_dir       = 照片与导航细节所在目录
#   robots.<机器人>.checked  完成数/分配数
#   return_home      succeeded = 已回到自己的充电桩
#   detail_report    该机器人的完整机读报告路径（本文件是给人看的汇总）
#
#   mission.anomaly_count  本次任务发现的异常总数（激光-地图差分检出）
#   anomalies        异常清单，每条含：发现机器人/区域/map 坐标/尺寸/取证照
#       type=center_relocated_prior 表示"巡检点被占被迫改址"——它不是确认的
#       异常物体，而是一条先验线索（有东西占着该去的位置）
#   robots.<机器人>.areas[].area_clear_status  到点检测结果：
#       checked=已检 / skipped_uncertain_pose=定位不稳跳过 /
#       skipped_misaligned=扫描与地图对不齐跳过（均为诚实弃权，非故障）
#
# 注：异常坐标在 map 系，误差地板≈AMCL 定位误差（实测 0.1~0.3m）。
# ==========================================================================
"""

    def write_mission_report(self, mission_dir: Path, route: list[str],
                             plan: dict[str, list[str]], codes: dict[str, int],
                             return_results: dict[str, bool],
                             started_at: float) -> Path:
        """Merge every runner's report.yaml into one annotated, human-first
        file at the top of the mission directory."""
        return_enabled = bool(self.get_parameter('return_home').value)
        mission_anomalies: list[dict] = []
        robots: dict[str, dict] = {}
        for ns, areas in plan.items():
            if not areas:
                robots[ns] = {'status': 'idle', 'allocated_areas': []}
                continue
            # Copy: the same list object reused in mission.allocation would
            # make yaml emit &id/*id anchors in the human-facing file.
            entry: dict = {'allocated_areas': list(areas)}
            runner_report = self._latest_runner_report(mission_dir / ns)
            if runner_report is None:
                entry['status'] = 'no_report'
            else:
                data, report_file = runner_report
                summary = data.get('summary') or {}
                entry['status'] = data.get('status')
                entry['checked'] = (f"{summary.get('checked_count', 0)}"
                                    f"/{summary.get('requested_count', len(areas))}")
                entry['areas'] = [{
                    'area': a.get('area'),
                    'display_name': a.get('display_name'),
                    'status': a.get('status'),
                    'photos': a.get('captured_image_count', 0),
                    'evidence_dir': a.get('evidence_dir'),
                    **({'area_clear_status': a['area_clear_status']}
                       if a.get('area_clear_status') else {}),
                    **({'reason': a['reason']} if a.get('reason') else {}),
                } for a in data.get('areas', [])]
                for anomaly in data.get('anomalies', []):
                    mission_anomalies.append({'robot': ns, **anomaly})
                entry['detail_report'] = str(report_file)
            if not return_enabled:
                entry['return_home'] = 'disabled'
            elif ns in return_results:
                entry['return_home'] = ('succeeded' if return_results[ns]
                                        else 'failed')
            else:
                entry['return_home'] = 'skipped'
            entry['exit_code'] = codes.get(ns)
            robots[ns] = entry

        all_ok = (all(code == 0 for code in codes.values())
                  and (not return_enabled or all(return_results.values())))
        export_anomaly_evidence(mission_dir, mission_anomalies)
        confirmed = [a for a in mission_anomalies
                     if a.get('type') != 'center_relocated_prior']
        mission = {
            'mission': {
                'generated_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
                'duration_sec': round(time.time() - started_at, 1),
                'route_requested': route,
                'allocation': {ns: list(areas) for ns, areas in plan.items()},
                'status': 'completed' if all_ok else 'completed_with_failures',
                'anomaly_count': len(confirmed),
            },
            'anomalies': mission_anomalies,
            'robots': robots,
        }
        mission_dir.mkdir(parents=True, exist_ok=True)
        path = mission_dir / 'report.yaml'
        with path.open('w', encoding='utf-8') as f:
            f.write(self.MISSION_REPORT_GUIDE)
            yaml.safe_dump(mission, f, sort_keys=False, allow_unicode=True)
        (mission_dir / 'SUMMARY.txt').write_text(
            build_summary_text(mission), encoding='utf-8')
        return path

    def _latest_runner_report(self, ns_dir: Path) -> tuple[dict, Path] | None:
        runs = sorted(d for d in ns_dir.glob('inspection_*') if d.is_dir())
        for run_dir in reversed(runs):
            report_file = run_dir / 'report.yaml'
            if report_file.exists():
                with report_file.open(encoding='utf-8') as f:
                    return yaml.safe_load(f) or {}, report_file
        return None

    def _home_goal(self, ns: str) -> NavigateToPose.Goal:
        home = self.robots[ns]['home_pose']
        yaw = float(home.get('yaw', 0.0))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(home['x'])
        goal.pose.pose.position.y = float(home['y'])
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def _start_home(self, ns: str, state: dict):
        send_future = state['client'].send_goal_async(self._home_goal(ns))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{ns}: return-home goal rejected')
            state['state'] = 'failed'
            return
        state['handle'] = handle
        state['result_future'] = handle.get_result_async()
        state['state'] = 'driving'

    def return_all_home(self, ns_list: list[str],
                        timeout_sec: float = 300.0) -> dict[str, bool]:
        """Concurrent return with a doorway mutex: every robot drives home at
        once, but only one at a time may thread the funnel into the
        mother_base bay (simultaneous unmanaged returns wedged there). A
        robot approaching a held gate is cancelled in place and re-sent once
        the holder has cleared the zone."""
        gate_x = float(self.home_gate.get('x', -1.65))
        gate_y = float(self.home_gate.get('y', -3.3))
        gate_radius = float(self.home_gate.get('radius', 1.0))
        hold_radius = gate_radius + 0.7  # stop before physically entering

        states: dict[str, dict] = {}
        for ns in ns_list:
            home = self.robots[ns].get('home_pose') or {}
            if not {'x', 'y'} <= home.keys():
                self.get_logger().warn(
                    f'{ns}: no home_pose in robots.yaml, skipping return')
                continue
            client = ActionClient(self, NavigateToPose, self.robots[ns]['nav_action'])
            if not client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'{ns}: nav action server unavailable for return')
                states[ns] = {'state': 'failed', 'handle': None}
                continue
            states[ns] = {'client': client, 'state': 'init',
                          'handle': None, 'result_future': None}
            self.get_logger().info(
                f"{ns}: returning home x={home['x']} y={home['y']}")
            self._start_home(ns, states[ns])

        def gate_dist(ns: str) -> float:
            px, py = self.robot_poses.get(ns, (math.inf, math.inf))
            return math.hypot(px - gate_x, py - gate_y)

        def past_gate(ns: str) -> bool:
            # Already closer to its dock than the gate is: it has threaded
            # the funnel and must not be held on the inside (observed: a
            # robot paused pointlessly in the bay right after passing).
            home = self.robots[ns]['home_pose']
            hx, hy = float(home['x']), float(home['y'])
            px, py = self.robot_poses.get(ns, (math.inf, math.inf))
            return math.hypot(px - hx, py - hy) < math.hypot(gate_x - hx, gate_y - hy)

        holder = None
        deadline = time.time() + timeout_sec
        while (time.time() < deadline
               and any(s['state'] in ('driving', 'held') for s in states.values())):
            rclpy.spin_once(self, timeout_sec=0.1)
            for ns, st in states.items():
                if st['state'] == 'driving' and st['result_future'].done():
                    result = st['result_future'].result()
                    if result is not None and result.status == 4:  # SUCCEEDED
                        st['state'] = 'done'
                        self.get_logger().info(f'{ns}: return home succeeded')
                    else:
                        st['state'] = 'failed'
                        status = getattr(result, 'status', 'no result')
                        self.get_logger().error(
                            f'{ns}: return home failed (status {status})')
            # The holder keeps the gate while it is inside and still driving.
            if holder is not None and (states[holder]['state'] != 'driving'
                                       or gate_dist(holder) > gate_radius):
                holder = None
            if holder is None:
                inside = [ns for ns, st in states.items()
                          if st['state'] == 'driving' and gate_dist(ns) <= gate_radius]
                if inside:
                    holder = min(inside, key=gate_dist)
            for ns, st in states.items():
                if ns == holder:
                    continue
                if (holder is not None and st['state'] == 'driving'
                        and gate_dist(ns) <= hold_radius and not past_gate(ns)):
                    st['handle'].cancel_goal_async()
                    st['state'] = 'held'
                    self.get_logger().info(
                        f'{ns}: holding before home gate ({holder} is inside)')
                elif holder is None and st['state'] == 'held':
                    self.get_logger().info(f'{ns}: gate clear, resuming return')
                    self._start_home(ns, st)
        for ns, st in states.items():
            if st['state'] in ('driving', 'held'):
                self.get_logger().error(f'{ns}: return home timed out')
                if st.get('handle') is not None:
                    st['handle'].cancel_goal_async()
        return {ns: st['state'] == 'done' for ns, st in states.items()}


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    try:
        code = node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
