#!/usr/bin/env python3
"""Anomaly texture markers — a standalone "sidecar" that draws each detected
anomaly in RViz as a custom image instead of the built-in red dot.
Live-demo only (feat/live-demo-tweaks).

DESIGN: pure OBSERVER, like demo_audio_node. It subscribes to the existing
/anomaly_events bus (the same JSON the GUI/RViz already consume) and republishes
a marker on a NEW topic /anomaly_texture_markers. It does NOT modify
inspection_runner, the detector, or /anomaly_events. To show the image in RViz,
point the existing "Anomalies" MarkerArray display at /anomaly_texture_markers
(done in dual_view.rviz). If this node is not running, nothing changes.

RENDERING: rviz2's embedded-texture TRIANGLE_LIST markers render as a black quad
on Humble, so instead we draw a MESH_RESOURCE marker — a flat 1x1 plane whose
material maps the image as a texture. rviz renders textured meshes reliably
(this is how robot models show their skins). The tiny .obj/.mtl are generated at
startup next to the image, so dropping any image and setting `image_path` just
works. If the image is missing, a bright orange cube is drawn instead so an
anomaly is never invisible.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# The original 0.60 m texture obscured nearby detections on the compact real
# arena map.  Keep the same custom icon and coordinates at half that footprint.
DEFAULT_MARKER_SIZE_M = 0.30


def _latched(depth: int = 1) -> QoSProfile:
    # Matches the RViz "Anomalies" display: Transient Local + Reliable.
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class AnomalyTextureNode(Node):
    def __init__(self):
        super().__init__('anomaly_texture_node')
        self.declare_parameter('in_topic', '/anomaly_events')
        self.declare_parameter('out_topic', '/anomaly_texture_markers')
        # The inspection runner sends Marker.DELETEALL here at the beginning
        # of every mission. Mirror that command so Sim-style texture icons do
        # not leak from one randomized stress round into the next.
        self.declare_parameter('control_topic', '/anomaly_markers')
        self.declare_parameter('image_path',
                               str(Path.home() / 'roboinspec_ws' / 'markers' / 'anomaly.png'))
        self.declare_parameter('frame', 'map')
        self.declare_parameter(
            'size', DEFAULT_MARKER_SIZE_M)         # plane edge length (m)
        self.declare_parameter('z', 0.05)          # height of the plane (m)
        self.declare_parameter('upright', False)   # True: standing billboard
        # In-plane rotation of the icon (deg). -90 = clockwise quarter-turn in
        # the top-down view; flip to +90 if it spins the wrong way.
        self.declare_parameter('texture_yaw_deg', -90.0)

        self.image_path = Path(self.get_parameter('image_path').value).expanduser()
        self.frame = str(self.get_parameter('frame').value)
        self.size = float(self.get_parameter('size').value)
        self.z = float(self.get_parameter('z').value)
        self.upright = bool(self.get_parameter('upright').value)
        self.tex_yaw = math.radians(float(self.get_parameter('texture_yaw_deg').value))

        self._markers: list[Marker] = []
        self._seq = 0
        self._mesh_uri = self._ensure_mesh()   # file://...obj or None

        self._pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('out_topic').value), _latched())
        self.create_subscription(
            String, str(self.get_parameter('in_topic').value),
            self._on_event, _latched(depth=20))
        self.create_subscription(
            MarkerArray, str(self.get_parameter('control_topic').value),
            self._on_control, _latched(depth=10))
        self.get_logger().info(
            'anomaly_texture_node: image=%s mesh=%s size=%.2f upright=%s -> %s' %
            (self.image_path, self._mesh_uri, self.size, self.upright,
             self.get_parameter('out_topic').value))

    # ---- generate a textured plane mesh next to the image ------------------
    def _ensure_mesh(self) -> str | None:
        if not self.image_path.is_file():
            self.get_logger().warn('marker image missing (%s); will draw a solid '
                                   'cube until you add it' % self.image_path)
            return None
        d = self.image_path.parent
        tex = self.image_path.name
        obj = d / '_anomaly_plane.obj'
        mtl = d / '_anomaly_plane.mtl'
        # Unit quad in XY centred at origin; double-sided; UVs upright.
        obj.write_text(
            'mtllib _anomaly_plane.mtl\n'
            'o anomaly_plane\n'
            'v -0.5 -0.5 0\nv 0.5 -0.5 0\nv 0.5 0.5 0\nv -0.5 0.5 0\n'
            'vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n'
            'vn 0 0 1\nvn 0 0 -1\n'
            'usemtl anomaly_mat\n'
            'f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n'
            'f 1/1/2 3/3/2 2/2/2\nf 1/1/2 4/4/2 3/3/2\n')
        # Full ambient+diffuse so scene lighting can't darken the icon.
        mtl.write_text(
            'newmtl anomaly_mat\n'
            'Ka 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\n'
            'd 1.0\nillum 1\n'
            'map_Kd %s\n' % tex)
        return obj.as_uri()

    # ---- marker construction ----------------------------------------------
    def _make_marker(self, idx: int, x: float, y: float) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame
        m.ns = 'anomaly_texture'
        m.id = idx
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.w = 1.0

        if self._mesh_uri:
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = self._mesh_uri
            m.mesh_use_embedded_materials = True
            m.scale.x = m.scale.y = m.scale.z = self.size
            if self.upright:
                m.pose.position.z = self.size / 2.0 + self.z
                # rotate the XY plane to stand up (90 deg about X): q=(0.707,0,0,0.707)
                m.pose.orientation.x = 0.70710678
                m.pose.orientation.w = 0.70710678
            else:
                m.pose.position.z = self.z
                # In-plane (about Z) rotation so the icon faces upright in view.
                m.pose.orientation.z = math.sin(self.tex_yaw / 2.0)
                m.pose.orientation.w = math.cos(self.tex_yaw / 2.0)
            # a=0 -> use the mesh's own material/texture, no colour tint.
            m.color.r = m.color.g = m.color.b = m.color.a = 0.0
        else:
            # No image yet -> bright orange cube so the anomaly is still obvious.
            m.type = Marker.CUBE
            m.pose.position.z = 0.25
            m.scale.x = m.scale.y = m.scale.z = max(0.1, self.size * 0.5)
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.0, 0.9
        return m

    def _on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
            x, y = float(ev['x']), float(ev['y'])
        except (ValueError, KeyError, TypeError):
            return
        self._seq += 1
        self._markers.append(self._make_marker(self._seq, x, y))
        # Republish the whole set so a late RViz subscriber gets everything.
        self._pub.publish(MarkerArray(markers=self._markers))

    def _on_control(self, msg: MarkerArray):
        if not any(marker.action == Marker.DELETEALL for marker in msg.markers):
            return
        self._markers.clear()
        self._seq = 0
        clear = Marker()
        clear.header.frame_id = self.frame
        clear.action = Marker.DELETEALL
        self._pub.publish(MarkerArray(markers=[clear]))


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyTextureNode()
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
