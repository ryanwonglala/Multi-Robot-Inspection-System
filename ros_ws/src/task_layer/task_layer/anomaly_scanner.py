"""Stationary anomaly detection component for the inspection runner (P1-5).

Wraps AreaClearChecker with the full gate chain that passed the P1-4
acceptance (both robots, all five gates): AMCL covariance gate, scan-map
alignment gate, one relocalize-spin retry, peer filter, in-area bounds
filter. Mirrors area_clear_acceptance.py — that harness stays the
reference for gate calibration; change thresholds there first, here second.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
import yaml

from task_layer.area_clear_check import AreaClearChecker, pose_uncertain


def yaw_of(quat) -> float:
    return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


class AnomalyScanner:
    """Owns its own scan/pose subscriptions on the host node; call
    detect_here() while the robot stands at an inspection stop."""

    def __init__(self, node, map_yaml: str, robots_yaml: str,
                 max_pose_std: float = 0.35, min_alignment: float = 0.80,
                 frames: int = 5, settle_sec: float = 1.0):
        self.node = node
        self.max_pose_std = max_pose_std
        self.min_alignment = min_alignment
        self.frames = frames
        self.settle_sec = settle_sec
        self.checker = AreaClearChecker(map_yaml)

        self._scans: list[LaserScan] = []
        self._collecting = False
        node.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._own_pose = None  # (x, y, yaw, covariance)
        node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._on_own_pose, latched)
        self._peer_poses: dict[str, tuple] = {}
        own_ns = node.get_namespace().strip('/')
        with open(robots_yaml, encoding='utf-8') as f:
            robots = (yaml.safe_load(f) or {}).get('robots') or {}
        for peer, info in robots.items():
            if peer == own_ns:
                continue
            node.create_subscription(
                PoseWithCovarianceStamped, info['amcl_pose_topic'],
                lambda msg, peer=peer: self._peer_poses.__setitem__(
                    peer, (msg.pose.pose.position.x, msg.pose.pose.position.y)),
                latched)
        self._cmd = node.create_publisher(Twist, 'cmd_vel', 10)

    def _on_scan(self, msg):
        if self._collecting:
            self._scans.append(msg)

    def _on_own_pose(self, msg):
        p = msg.pose.pose
        self._own_pose = (p.position.x, p.position.y, yaw_of(p.orientation),
                          list(msg.pose.covariance))

    def _settle(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _collect_scans(self) -> list[LaserScan]:
        self._scans = []
        self._collecting = True
        deadline = time.time() + 20.0
        while len(self._scans) < self.frames and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self._collecting = False
        return self._scans[:self.frames]

    def relocalize_spin(self, duration_sec: float = 14.0, w: float = 0.5):
        """AMCL only updates on motion: a robot arriving with inflated
        covariance keeps it forever while parked. One slow full turn feeds
        it fresh geometry. Do NOT spin amid unmapped obstacles in
        geometry-starved areas (east_hall diverged 3.7 m in P1-4) — those
        areas use wall-near viewpoints instead of center stops."""
        msg = Twist()
        msg.angular.z = w
        end = time.time() + duration_sec
        while time.time() < end:
            self._cmd.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._cmd.publish(Twist())

    def detect_here(self, bounds=None) -> dict:
        """Run the gated check at the current (stationary) pose.
        Returns {'status': 'checked'|'skipped_*'|'no_data', 'anomalies': [...],
        'evidence_cells', 'alignment', 'pose_std', 'pose': {x, y, yaw}}."""
        for attempt in (1, 2):
            self._settle(self.settle_sec)
            scans = self._collect_scans()
            if len(scans) < self.frames or self._own_pose is None:
                return {'status': 'no_data', 'anomalies': []}
            x, y, yaw, cov = self._own_pose
            std = round(math.sqrt(max(cov[0], cov[7], 0.0)), 3)
            uncertain = pose_uncertain(cov, self.max_pose_std)
            align = 0.0 if uncertain else sum(
                self.checker.alignment_ratio(s, (x, y, yaw))
                for s in scans) / len(scans)
            if uncertain or align < self.min_alignment:
                if attempt == 1:
                    self.node.get_logger().warn(
                        f'area_clear: pose_std={std} align={align:.2f}'
                        ' — spinning in place to relocalize')
                    self.relocalize_spin()
                    continue
                status = ('skipped_uncertain_pose' if uncertain
                          else 'skipped_misaligned')
                return {'status': status, 'anomalies': [],
                        'pose_std': std, 'alignment': round(align, 3)}
            report = self.checker.check(
                scans, (x, y, yaw),
                peers=list(self._peer_poses.values()), bounds=bounds)
            return {
                'status': 'checked',
                'anomalies': report['anomalies'],
                'evidence_cells': report['evidence_cells'],
                'alignment': round(align, 3),
                'pose_std': std,
                'pose': {'x': round(x, 3), 'y': round(y, 3),
                         'yaw': round(yaw, 4)},
            }
        return {'status': 'no_data', 'anomalies': []}
