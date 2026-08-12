#!/usr/bin/env python3
"""Demo audio cues — a standalone "sidecar" that plays a sound at four key
workflow moments. Live-demo only (feat/live-demo-tweaks).

DESIGN: this node is a pure OBSERVER. It only subscribes to topics that already
exist and watches the reports directory; it never publishes a command, calls a
service, or touches the inspection / anomaly-detection code. If it lags, errors,
or is never launched, the patrol behaves exactly as before. Audio is played in
throwaway daemon threads via an external player, so playback cannot block ROS
callbacks either.

The four cues and the existing signals they watch:
  1. ready     -> both robots' nav2 NavigateToPose servers are up AND an
                  amcl_pose has arrived for each (system loaded, fleet ready).
  2. task      -> first robot starts moving (/<ns>/cmd_vel goes non-zero) after
                  ready, i.e. the dispatched mission was accepted and is running.
  3. anomaly   -> a new event arrives on /anomaly_events (the same latched bus
                  the GUI/RViz use to mark a detection).
  4. complete  -> a fresh top-level mission_report.md appears under report_dir.
                  task_allocator writes it only AFTER both robots finished and
                  returned to dock (run_until_all_home), so a single robot
                  ending its own route does NOT trigger this cue.

Drop four audio files (paplay-compatible: .wav / .ogg / .flac) into
~/roboinspec_ws/sounds/ with the names below (override via params if you like).
"""
from __future__ import annotations

import os
import shlex
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose


def _latched_qos() -> QoSProfile:
    # Matches the latched publishers (amcl_pose, /anomaly_events).
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class DemoAudioNode(Node):
    def __init__(self):
        super().__init__('demo_audio_node')
        self.declare_parameter('robots', ['tb3', 'arm'])
        self.declare_parameter('sounds_dir',
                               str(Path.home() / 'roboinspec_ws' / 'sounds'))
        self.declare_parameter('ready_file', '01_ready.wav')
        self.declare_parameter('task_file', '02_task_received.wav')
        self.declare_parameter('anomaly_file', '03_anomaly.wav')
        self.declare_parameter('complete_file', '04_complete.wav')
        # External player. paplay handles wav/ogg/flac. For mp3 install mpg123
        # and set player_cmd:='mpg123 -q'  (the file path is appended as argv).
        self.declare_parameter('player_cmd', 'paplay')
        self.declare_parameter('report_dir',
                               str(Path.home() / 'roboinspec_ws' / 'reports'))
        # Fire cue 2 only once both robots have moved (True) or as soon as the
        # first one moves (False). False is the robust default: a single-robot
        # or empty-route run still gets the cue.
        self.declare_parameter('task_requires_both', False)
        self.declare_parameter('cmd_vel_threshold', 0.02)   # m/s or rad/s
        self.declare_parameter('anomaly_debounce_sec', 4.0)
        self.declare_parameter('poll_period_sec', 1.0)
        self.declare_parameter('startup_grace_sec', 2.5)    # ignore latched replays

        self.robots = [str(ns).strip().strip('/')
                       for ns in (self.get_parameter('robots').value or [])]
        self.sounds_dir = Path(self.get_parameter('sounds_dir').value).expanduser()
        self.player_cmd = shlex.split(str(self.get_parameter('player_cmd').value))
        self.report_dir = Path(self.get_parameter('report_dir').value).expanduser()
        self.task_requires_both = bool(self.get_parameter('task_requires_both').value)
        self.cmd_thresh = float(self.get_parameter('cmd_vel_threshold').value)
        self.anomaly_debounce = float(self.get_parameter('anomaly_debounce_sec').value)
        self.startup_grace = float(self.get_parameter('startup_grace_sec').value)

        self._start_time = time.time()
        # cue latches / state
        self._ready_fired = False
        self._task_fired = False
        self._amcl_seen = {ns: False for ns in self.robots}
        self._moved = {ns: False for ns in self.robots}
        self._last_anomaly_play = 0.0
        self._anomaly_seen: set[str] = set()
        self._report_baseline = self._newest_report_mtime()

        # One NavigateToPose client per robot ONLY to query server availability
        # (server_is_ready); we never send a goal.
        self._nav_clients = {
            ns: ActionClient(self, NavigateToPose,
                             f'/{ns}/navigate_to_pose' if ns else 'navigate_to_pose')
            for ns in self.robots}

        latched = _latched_qos()
        self.create_subscription(String, '/anomaly_events',
                                 self._on_anomaly, latched)
        for ns in self.robots:
            amcl_topic = f'/{ns}/amcl_pose' if ns else '/amcl_pose'
            # PoseWithCovarianceStamped, but we only need the arrival signal:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            self.create_subscription(
                PoseWithCovarianceStamped, amcl_topic,
                lambda _msg, n=ns: self._on_amcl(n), latched)
            cmd_topic = f'/{ns}/cmd_vel' if ns else '/cmd_vel'
            self.create_subscription(
                Twist, cmd_topic, lambda msg, n=ns: self._on_cmd_vel(n, msg), 10)

        self.create_timer(float(self.get_parameter('poll_period_sec').value),
                          self._tick)
        self.get_logger().info(
            'demo_audio_node watching robots=%s sounds_dir=%s player=%s' %
            (self.robots, self.sounds_dir, ' '.join(self.player_cmd)))

    # ---- playback (never blocks ROS; reaped by the daemon thread) ----------
    def _play(self, filename: str, tag: str):
        path = self.sounds_dir / filename
        if not path.is_file():
            self.get_logger().warn('cue [%s]: audio file missing: %s' % (tag, path))
            return
        self.get_logger().info('cue [%s] -> %s' % (tag, path.name))

        def _run():
            try:
                subprocess.run(self.player_cmd + [str(path)],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
            except Exception as exc:  # noqa: BLE001 - audio must never crash us
                self.get_logger().warn('player failed: %s' % exc)
        threading.Thread(target=_run, daemon=True).start()

    # ---- report watching (cue 4) -------------------------------------------
    def _newest_report_mtime(self) -> float:
        # ONLY the top-level mission_report.md, which task_allocator writes after
        # run_until_all_home() -> i.e. AFTER both robots finished and returned to
        # dock, and right as the GUI surfaces the archive dir. Deliberately NOT
        # the per-robot/per-area `report.md` files: those appear the moment a
        # single robot finishes its route and would fire cue 4 too early.
        newest = 0.0
        try:
            for p in self.report_dir.rglob('mission_report.md'):
                newest = max(newest, p.stat().st_mtime)
        except OSError:
            pass
        return newest

    # ---- subscription callbacks --------------------------------------------
    def _on_amcl(self, ns: str):
        self._amcl_seen[ns] = True

    def _on_cmd_vel(self, ns: str, msg: Twist):
        if not self._ready_fired or self._task_fired:
            return
        moving = (abs(msg.linear.x) > self.cmd_thresh
                  or abs(msg.linear.y) > self.cmd_thresh
                  or abs(msg.angular.z) > self.cmd_thresh)
        if moving:
            self._moved[ns] = True
            ready = (all(self._moved.values()) if self.task_requires_both
                     else any(self._moved.values()))
            if ready:
                self._task_fired = True
                self._mission_start = time.time()
                self._play(self.get_parameter('task_file').value, 'task')

    def _on_anomaly(self, msg: String):
        # Drop the latched replay of pre-existing events right after startup.
        if time.time() - self._start_time < self.startup_grace:
            self._anomaly_seen.add(msg.data)
            return
        if msg.data in self._anomaly_seen:
            return
        self._anomaly_seen.add(msg.data)
        now = time.time()
        if now - self._last_anomaly_play < self.anomaly_debounce:
            return
        self._last_anomaly_play = now
        self._play(self.get_parameter('anomaly_file').value, 'anomaly')

    # ---- periodic checks (cues 1 and 4) ------------------------------------
    def _tick(self):
        # Cue 1: fleet ready (nav servers up + amcl seen for every robot).
        if not self._ready_fired:
            servers_up = all(c.server_is_ready() for c in self._nav_clients.values())
            if servers_up and all(self._amcl_seen.values()):
                self._ready_fired = True
                self._play(self.get_parameter('ready_file').value, 'ready')

        # Cue 4: a fresh report appeared after the mission started.
        if self._task_fired:
            newest = self._newest_report_mtime()
            if newest > self._report_baseline + 1e-6:
                self._report_baseline = newest
                self._play(self.get_parameter('complete_file').value, 'complete')
                # Re-arm for the next mission in the same session.
                self._task_fired = False
                self._moved = {ns: False for ns in self.robots}


def main(args=None):
    rclpy.init(args=args)
    node = DemoAudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
