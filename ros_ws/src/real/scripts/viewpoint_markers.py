#!/usr/bin/env python3
"""Publish the real TB3 home and authored viewpoints for RViz."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def marker_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class ViewpointMarkers(Node):
    def __init__(self):
        super().__init__('viewpoint_markers')
        self.declare_parameter('world_model_path', '')
        self.declare_parameter('topic', '/waypoints')
        self.declare_parameter('area', 'arena')
        self.declare_parameter('goal_tolerance_m', 0.15)
        self.declare_parameter('publish_period_sec', 1.0)

        model_path = Path(
            str(self.get_parameter('world_model_path').value)
        ).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'world_model_path does not exist: {model_path}')
        with model_path.open(encoding='utf-8') as handle:
            self.world = yaml.safe_load(handle) or {}

        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('topic').value),
            marker_qos(),
        )
        period = float(self.get_parameter('publish_period_sec').value)
        self.timer = self.create_timer(max(0.2, period), self.publish)
        self.publish()

    @staticmethod
    def _base(marker_id: int, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        return marker

    @staticmethod
    def _set_color(marker: Marker, rgba: tuple[float, float, float, float]):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba

    def _pose_markers(
        self,
        marker_id: int,
        label: str,
        x: float,
        y: float,
        color: tuple[float, float, float, float],
    ) -> list[Marker]:
        markers = []

        disc = self._base(marker_id, 'viewpoint_discs', Marker.CYLINDER)
        disc.pose.position.x = x
        disc.pose.position.y = y
        disc.pose.position.z = 0.018
        disc.scale.x = 0.14
        disc.scale.y = 0.14
        disc.scale.z = 0.035
        self._set_color(disc, color)
        markers.append(disc)

        text = self._base(marker_id, 'viewpoint_labels', Marker.TEXT_VIEW_FACING)
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.30
        text.scale.z = 0.14
        text.text = f'{label}\n({x:.3f}, {y:.3f})'
        self._set_color(text, (1.0, 1.0, 1.0, 1.0))
        markers.append(text)

        tolerance = float(self.get_parameter('goal_tolerance_m').value)
        ring = self._base(marker_id, 'viewpoint_tolerance', Marker.LINE_STRIP)
        ring.pose.position.z = 0.045
        ring.scale.x = 0.018
        self._set_color(ring, (color[0], color[1], color[2], 0.75))
        from geometry_msgs.msg import Point
        for index in range(49):
            angle = 2.0 * math.pi * index / 48.0
            point = Point()
            point.x = x + tolerance * math.cos(angle)
            point.y = y + tolerance * math.sin(angle)
            ring.points.append(point)
        markers.append(ring)

        return markers

    def publish(self):
        area_key = str(self.get_parameter('area').value)
        area = (self.world.get('areas') or {}).get(area_key)
        if not area:
            self.get_logger().error(f'area not found in world model: {area_key}')
            return

        markers = []
        home = ((self.world.get('robot_start') or {}).get('pose') or {})
        if 'x' in home and 'y' in home:
            markers.extend(self._pose_markers(
                0,
                'HOME',
                float(home['x']),
                float(home['y']),
                (0.20, 1.0, 0.25, 0.95),
            ))

        for index, viewpoint in enumerate(area.get('viewpoints') or [], 1):
            markers.extend(self._pose_markers(
                index,
                f'VP{index}',
                float(viewpoint['x']),
                float(viewpoint['y']),
                (0.10, 0.75, 1.0, 0.95),
            ))

        now = self.get_clock().now().to_msg()
        for marker in markers:
            marker.header.stamp = now
        self.publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = ViewpointMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
