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

from task_layer.report_writer import (
    default_report_dir,
    prune_report_dirs,
)


def default_share(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('task_layer')) / 'config' / name)


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.declare_parameter('route', '')
        self.declare_parameter('robots_yaml', default_share('robots.yaml'))
        self.declare_parameter('world_model_path', default_share('world_model.yaml'))
        self.declare_parameter('report_dir', default_report_dir())
        self.declare_parameter('pose_wait_sec', 5.0)
        self.declare_parameter('return_home', True)
        # Per-robot bound on the return-home drive: a robot whose dock/funnel is
        # blocked gives up gracefully and frees the gate instead of wedging.
        self.declare_parameter('return_timeout_sec', 150.0)
        # Overall safety net for the whole post-launch phase (runners are
        # individually bounded by their own B-axis logic, so this only guards
        # against a pathological hang).
        self.declare_parameter('mission_backstop_sec', 1800.0)
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        # Photo-diff baseline, forwarded verbatim to every runner.
        # baseline_record:=true turns this dispatch into a clean-scene baseline
        # patrol (record reference photos, no diff); otherwise each runner diffs
        # its views against baseline_dir. The default dir matches the runner's
        # own default (OLD flat layout: baselines/<area>/<stop>/yawNN.ppm),
        # so GUI/Auto-allocate detection finds the baselines with no extra
        # flags once a baseline has been recorded.
        self.declare_parameter('baseline_record', False)
        self.declare_parameter(
            'baseline_dir',
            str(Path.home() / 'roboinspec_ws' / 'baselines'))

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
            # return so it can arbitrate the shared doorway into the mother_base
            # bay (unmanaged simultaneous returns wedged each other there). Each
            # robot is sent home the instant it finishes. See run_until_all_home().
            command = [
                'ros2', 'run', 'task_layer', 'inspection_runner.py', '--ros-args',
                '-r', f'__ns:=/{ns}',
                '-p', f'use_sim_time:={use_sim_time}',
                '-p', f"route:={','.join(areas)}",
                '-p', 'return_home:=false',
                '-p', f'report_dir:={report_dir}',
                '-p', 'baseline_record:=' + str(
                    bool(self.get_parameter('baseline_record').value)).lower(),
                '-p', f"baseline_dir:={self.get_parameter('baseline_dir').value}",
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

        # Early return: the moment a robot finishes its route it heads home
        # (no waiting for peers); a gate mutex serialises the mother_base funnel
        # for whoever is returning at once. See run_until_all_home().
        codes, return_results = self.run_until_all_home(procs)
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
# 注：v0.3 报告记录"执行与取证"；激光/视觉异常判读将在 P1-5 接入后出现在
#     anomalies 字段中。
# ==========================================================================
"""

    def write_mission_report(self, mission_dir: Path, route: list[str],
                             plan: dict[str, list[str]], codes: dict[str, int],
                             return_results: dict[str, bool],
                             started_at: float) -> Path:
        """Merge every runner's report.yaml into one annotated, human-first
        file at the top of the mission directory."""
        return_enabled = bool(self.get_parameter('return_home').value)
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
                    **({'reason': a['reason']} if a.get('reason') else {}),
                } for a in data.get('areas', [])]
                # Carry the anomaly list (with a friendly display_name resolved
                # from the area map) so the mission report can summarise count +
                # location at the top and relocate the evidence images.
                dn_map = {a.get('area'): a.get('display_name')
                          for a in data.get('areas', [])}
                entry['anomalies'] = [
                    {**an, 'display_name': dn_map.get(an.get('area'), an.get('area'))}
                    for an in data.get('anomalies', [])
                ]
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
        mission = {
            'mission': {
                'generated_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
                'duration_sec': round(time.time() - started_at, 1),
                'route_requested': route,
                'allocation': {ns: list(areas) for ns, areas in plan.items()},
                'status': 'completed' if all_ok else 'completed_with_failures',
            },
            'robots': robots,
        }
        mission_dir.mkdir(parents=True, exist_ok=True)

        # Aggregate every robot's anomalies into one mission-level list and copy
        # each evidence image into a single `anomaly_evidence/` folder (sibling
        # of mission_report.md) for one-stop review. Record count + list on the
        # mission dict so they surface in both files.
        anomalies = self._collect_anomaly_evidence(mission_dir, robots)
        mission['mission']['anomaly_count'] = len(anomalies)
        mission['anomalies'] = anomalies

        # Task 5.3 — two-file mission report layout: mission_details.yaml (machine) +
        # mission_report.md (bilingual Markdown). report.yaml is no longer written.

        # File 1: full machine-readable mission dict.
        details_path = mission_dir / 'mission_details.yaml'
        with details_path.open('w', encoding='utf-8') as f:
            yaml.safe_dump(mission, f, sort_keys=False, allow_unicode=True)

        # File 2: bilingual Markdown mission summary.
        md_path = self._write_mission_markdown(mission, mission_dir, robots, anomalies)

        # Retention: keep only the 10 newest missions.
        prune_report_dirs(mission_dir.parent, 'mission_*', keep=10)

        return md_path

    def _collect_anomaly_evidence(self, mission_dir: Path, robots: dict) -> list[dict]:
        """Copy every anomaly's evidence photo into one mission-level
        `anomaly_evidence/` folder (sibling of mission_report.md) so a reviewer
        can browse all visual anomalies in one place instead of digging through
        each robot's per-area scan folders, where evidence sits mixed in with
        the routine 360° capture frames.

        Originals are left untouched as the full per-robot record (copy, not
        move). Returns a flat, ordered anomaly list; each item records the
        relocated path relative to mission_dir (so the Markdown can link it).
        """
        collected: list[dict] = []
        evidence_root = mission_dir / 'anomaly_evidence'
        for ns, entry in robots.items():
            for an in entry.get('anomalies', []):
                seq = len(collected) + 1
                record = {
                    'index': seq,
                    'robot': ns,
                    'area': an.get('area'),
                    'display_name': an.get('display_name') or an.get('area'),
                    'x': an.get('x'),
                    'y': an.get('y'),
                    'extent': an.get('extent'),
                    'source_photo': an.get('evidence_photo'),
                }
                src = an.get('evidence_photo')
                if src and Path(src).is_file():
                    evidence_root.mkdir(parents=True, exist_ok=True)
                    src_path = Path(src)
                    dest_name = f'{seq:02d}_{ns}_{an.get("area")}_{src_path.name}'
                    dest = evidence_root / dest_name
                    shutil.copy2(src_path, dest)
                    record['evidence_file'] = str(dest.relative_to(mission_dir))
                collected.append(record)
        return collected

    def _write_mission_markdown(
        self,
        mission: dict,
        mission_dir: Path,
        robots: dict,
        anomalies: list[dict] | None = None,
    ) -> Path:
        """Render a simplified bilingual Markdown mission report.

        Task 5.3 — mission-level counterpart of write_markdown_report.
        Reuses the bilingual spirit of MISSION_REPORT_GUIDE.
        """
        m = mission.get('mission') or {}
        status = m.get('status', 'unknown')
        generated_at = m.get('generated_at', '')
        duration_sec = m.get('duration_sec', 0)
        route_requested = m.get('route_requested') or []
        allocation = m.get('allocation') or {}

        STATUS_MAP = {
            'completed': '已完成 / Completed',
            'completed_with_failures': '完成但有失败项 / Completed with failures',
        }
        status_label = STATUS_MAP.get(status, status)
        anomalies = anomalies or []

        lines: list[str] = []
        lines.append('# RoboInspect 联合巡检报告 / Mission Report\n')

        lines.append('## 概要 / Summary\n')
        lines.append(f'- **整体状态 / Overall status**: `{status_label}`')
        lines.append(f'- **发现异常 / Anomalies found**: {len(anomalies)}')
        lines.append(f'- **生成时间 / Generated at**: {generated_at}')
        lines.append(f'- **任务时长 / Duration**: {duration_sec} s')
        if route_requested:
            lines.append(f'- **请求路线 / Requested route**: {", ".join(f"`{r}`" for r in route_requested)}')
        lines.append('')

        # Anomalies up front: count + location of each, with a link to the
        # relocated evidence image under anomaly_evidence/.
        lines.append('## 异常 / Anomalies\n')
        if anomalies:
            lines.append('> 所有异常证据图片已汇总至 `anomaly_evidence/` 目录,便于集中查看。')
            lines.append('> All anomaly evidence images are collected under `anomaly_evidence/` for one-stop review.\n')
            lines.append('| # | 机器人 / Robot | 位置 / Location | 坐标 / Coords (x, y) | 证据 / Evidence |')
            lines.append('|---|---|---|---|---|')
            for a in anomalies:
                ak = a.get('area') or ''
                dn = a.get('display_name') or ak
                loc = f'{dn} (`{ak}`)' if ak else dn
                x, y = a.get('x'), a.get('y')
                coord = (f'({x:.3f}, {y:.3f})'
                         if isinstance(x, (int, float)) and isinstance(y, (int, float))
                         else 'n/a')
                ev = a.get('evidence_file')
                ev_cell = f'`{ev}`' if ev else '_(missing / 缺失)_'
                lines.append(f'| {a.get("index")} | {a.get("robot")} | {loc} | {coord} | {ev_cell} |')
        else:
            lines.append('未发现异常。 / No anomalies detected.')
        lines.append('')

        lines.append('## 任务分配 / Allocation\n')
        if allocation:
            for ns, areas in allocation.items():
                areas_str = ', '.join(f'`{a}`' for a in areas) if areas else '_(none)_'
                lines.append(f'- **{ns}**: {areas_str}')
        else:
            lines.append('_(no allocation)_')
        lines.append('')

        lines.append('## 各机器人结果 / Per-robot Results\n')
        for ns, entry in robots.items():
            lines.append(f'### {ns}\n')
            r_status = entry.get('status', 'unknown')
            lines.append(f'- **状态 / Status**: `{r_status}`')
            checked = entry.get('checked', '?/?')
            lines.append(f'- **完成区域 / Checked**: {checked}')
            rh = entry.get('return_home', 'unknown')
            lines.append(f'- **回桩 / Return home**: `{rh}`')
            exit_code = entry.get('exit_code')
            if exit_code is not None:
                lines.append(f'- **退出码 / Exit code**: {exit_code}')
            areas = entry.get('areas') or []
            if areas:
                lines.append('')
                lines.append('| 区域 / Area | 显示名 / Display Name | 状态 / Status | 照片 / Photos |')
                lines.append('|---|---|---|---|')
                for a in areas:
                    ak = a.get('area') or ''
                    dn = a.get('display_name') or ak
                    ast = a.get('status') or 'unknown'
                    photos = a.get('photos', 0)
                    lines.append(f'| `{ak}` | {dn} | `{ast}` | {photos} |')
            detail_report = entry.get('detail_report', '')
            if detail_report:
                lines.append(f'\n- **详细报告 / Detail report**: `{detail_report}`')
            lines.append('')

        lines.append('## 相关文件 / Related Files\n')
        lines.append(f'- **完整机读报告 / Full machine report**: `{mission_dir / "mission_details.yaml"}`')
        if anomalies:
            lines.append(f'- **异常证据目录 / Anomaly evidence folder**: `{mission_dir / "anomaly_evidence"}`')
        lines.append('')

        md_path = mission_dir / 'mission_report.md'
        md_path.write_text('\n'.join(lines), encoding='utf-8')
        return md_path

    def _latest_runner_report(self, ns_dir: Path) -> tuple[dict, Path] | None:
        # Task 5.3 — read details.yaml (report.yaml removed); use embedded
        # summary_report as the structured view so all downstream field reads
        # (status, summary.checked_count, areas[], return_home, …) keep working.
        runs = sorted(d for d in ns_dir.glob('inspection_*') if d.is_dir())
        for run_dir in reversed(runs):
            report_file = run_dir / 'details.yaml'
            if report_file.exists():
                with report_file.open(encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                view = data.get('summary_report')
                if view is None:
                    # Runner exited before embedding summary_report (e.g. crash):
                    # normalize the full-report area shape (target_area -> area,
                    # scan_samples -> photo count) so the mission report still
                    # reads sane fields instead of None/0.
                    view = {
                        'task': data.get('task'),
                        'status': data.get('status'),
                        'route': data.get('route'),
                        'summary': data.get('summary') or {},
                        'anomalies': data.get('anomalies', []),
                        'return_home': data.get('return_home'),
                        'areas': [
                            {
                                'area': a.get('target_area'),
                                'display_name': a.get('display_name'),
                                'status': a.get('status'),
                                'captured_image_count': len(a.get('scan_samples') or []),
                                'evidence_dir': a.get('evidence_dir'),
                                **({'reason': a['reason']} if a.get('reason') else {}),
                            }
                            for a in (data.get('areas') or [])
                        ],
                    }
                return view, report_file
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

    def _begin_return(self, ns: str, st: dict):
        """A robot has just finished its route: send it home, or mark it done if
        return is disabled / it has no dock."""
        if not bool(self.get_parameter('return_home').value):
            st['state'] = 'done'
            return
        home = self.robots[ns].get('home_pose') or {}
        if not {'x', 'y'} <= home.keys():
            self.get_logger().warn(f'{ns}: no home_pose in robots.yaml, skipping return')
            st['state'] = 'done'
            return
        st['return_attempted'] = True
        client = ActionClient(self, NavigateToPose, self.robots[ns]['nav_action'])
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{ns}: nav action server unavailable for return')
            st['state'] = 'failed'
            return
        st['client'] = client
        st['return_deadline'] = time.time() + float(
            self.get_parameter('return_timeout_sec').value)
        self.get_logger().info(
            f"{ns}: route done, returning home x={home['x']} y={home['y']}")
        self._start_home(ns, st)  # sends goal, sets state 'driving' or 'failed'

    def run_until_all_home(self, procs: dict) -> tuple[dict, dict]:
        """Unified post-launch scheduler. Polls each runner; the instant a robot
        finishes its route it is sent home (EARLY return -- no waiting for
        peers). A single doorway mutex serialises the mother_base funnel for
        whoever is returning concurrently: only one threads the gate at a time;
        another approaching it is cancelled in place and re-sent once the holder
        clears. Each return is bounded by return_timeout_sec so a blocked dock
        gives up gracefully and frees the gate instead of wedging the mission."""
        gate_x = float(self.home_gate.get('x', -1.65))
        gate_y = float(self.home_gate.get('y', -3.3))
        gate_radius = float(self.home_gate.get('radius', 1.0))
        hold_radius = gate_radius + 0.7  # stop before physically entering
        backstop = time.time() + float(self.get_parameter('mission_backstop_sec').value)

        codes: dict = {}
        states = {ns: {'state': 'inspecting', 'client': None, 'handle': None,
                       'result_future': None, 'return_deadline': None,
                       'return_attempted': False}
                  for ns in procs}

        def gate_dist(ns: str) -> float:
            px, py = self.robot_poses.get(ns, (math.inf, math.inf))
            return math.hypot(px - gate_x, py - gate_y)

        def past_gate(ns: str) -> bool:
            # Already closer to its dock than the gate is -> it has threaded the
            # funnel and must not be held on the inside.
            home = self.robots[ns].get('home_pose') or {}
            if not {'x', 'y'} <= home.keys():
                return True
            hx, hy = float(home['x']), float(home['y'])
            px, py = self.robot_poses.get(ns, (math.inf, math.inf))
            return math.hypot(px - hx, py - hy) < math.hypot(gate_x - hx, gate_y - hy)

        holder = None
        while time.time() < backstop:
            rclpy.spin_once(self, timeout_sec=0.1)

            # (a) Runner still inspecting: dispatch it home the instant it exits.
            for ns, (process, log_file) in procs.items():
                if states[ns]['state'] == 'inspecting' and process.poll() is not None:
                    codes[ns] = process.returncode
                    try:
                        log_file.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self.get_logger().info(f'{ns}: finished with code {codes[ns]}')
                    self._begin_return(ns, states[ns])

            # (b) Advance active returns: terminal result, or per-robot timeout.
            for ns, st in states.items():
                if st['state'] != 'driving':
                    continue
                if st['result_future'] is not None and st['result_future'].done():
                    result = st['result_future'].result()
                    if result is not None and result.status == 4:  # SUCCEEDED
                        st['state'] = 'done'
                        self.get_logger().info(f'{ns}: return home succeeded')
                    else:
                        st['state'] = 'failed'
                        status = getattr(result, 'status', 'no result')
                        self.get_logger().error(
                            f'{ns}: return home failed (status {status})')
                elif st['return_deadline'] and time.time() > st['return_deadline']:
                    self.get_logger().error(f'{ns}: return home timed out, giving up')
                    if st['handle'] is not None:
                        st['handle'].cancel_goal_async()
                    st['state'] = 'failed'

            # (c) Doorway mutex across whoever is returning right now.
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
                    if st['handle'] is not None:
                        st['handle'].cancel_goal_async()
                    st['state'] = 'held'
                    self.get_logger().info(
                        f'{ns}: holding before home gate ({holder} is inside)')
                elif holder is None and st['state'] == 'held':
                    self.get_logger().info(f'{ns}: gate clear, resuming return')
                    self._start_home(ns, st)

            # (d) Done once every robot has returned (or failed / was skipped).
            if all(st['state'] in ('done', 'failed') for st in states.values()):
                break

        # Backstop hit: reap/cancel anything still outstanding.
        for ns, (process, log_file) in procs.items():
            st = states[ns]
            if st['state'] == 'inspecting':
                self.get_logger().error(f'{ns}: runner did not finish (mission backstop)')
                if process.poll() is None:
                    process.terminate()
                try:
                    log_file.close()
                except Exception:  # noqa: BLE001
                    pass
            if st['state'] in ('driving', 'held'):
                self.get_logger().error(f'{ns}: return home timed out (mission backstop)')
                if st.get('handle') is not None:
                    st['handle'].cancel_goal_async()
                st['state'] = 'failed'

        for ns, (process, _log_file) in procs.items():
            codes.setdefault(
                ns, process.poll() if process.poll() is not None else 6)
        return_results = {ns: st['state'] == 'done'
                          for ns, st in states.items() if st['return_attempted']}
        return codes, return_results


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
