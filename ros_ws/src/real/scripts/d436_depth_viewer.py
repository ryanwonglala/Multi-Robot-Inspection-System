#!/usr/bin/env python3
"""Read-only RGB-D distance viewer for the Jetson RealSense D436."""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


WINDOW_NAME = 'D436 RGB-D distance viewer'


def image_to_bgr(message: Image) -> np.ndarray:
    height, width = int(message.height), int(message.width)
    encoding = message.encoding.lower()
    channels = 4 if encoding in ('rgba8', 'bgra8') else 3
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        height, message.step)
    image = rows[:, :width * channels].reshape(height, width, channels)
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == 'bgr8':
        return image.copy()
    raise ValueError(f'unsupported color encoding: {message.encoding}')


def image_to_depth_m(message: Image) -> np.ndarray:
    """Convert 16UC1 millimetres or 32FC1 metres to float32 metres."""
    height, width = int(message.height), int(message.width)
    encoding = message.encoding.upper()
    if encoding in ('16UC1', 'MONO16'):
        dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
        row_elements = int(message.step) // dtype.itemsize
        raw = np.frombuffer(message.data, dtype=dtype).reshape(
            height, row_elements)[:, :width]
        depth = raw.astype(np.float32) * 0.001
        depth[raw == 0] = np.nan
        return depth
    if encoding == '32FC1':
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        row_elements = int(message.step) // dtype.itemsize
        depth = np.frombuffer(message.data, dtype=dtype).reshape(
            height, row_elements)[:, :width].astype(np.float32)
        depth[~np.isfinite(depth) | (depth <= 0.0)] = np.nan
        return depth
    raise ValueError(f'unsupported depth encoding: {message.encoding}')


def robust_distance(depth: np.ndarray, x: int, y: int, radius: int = 4) -> float:
    """Median valid distance around a pixel, robust to stereo holes."""
    height, width = depth.shape
    x0, x1 = max(0, x - radius), min(width, x + radius + 1)
    y0, y1 = max(0, y - radius), min(height, y + radius + 1)
    values = depth[y0:y1, x0:x1]
    values = values[np.isfinite(values) & (values >= 0.20) & (values <= 10.0)]
    return float(np.median(values)) if values.size else math.nan


class D436DepthViewer(Node):
    def __init__(self) -> None:
        super().__init__('d436_depth_viewer')
        self.declare_parameter(
            'color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('display_min_m', 0.20)
        self.declare_parameter('display_max_m', 3.00)
        self.declare_parameter('stale_after_sec', 1.0)

        self._lock = threading.Lock()
        self._color: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._color_time = 0.0
        self._depth_time = 0.0
        self._mouse = (0, 0)
        self._locked: tuple[int, int] | None = None
        self._quit = False
        self._last_error = ''

        self._color_sub = self.create_subscription(
            Image, str(self.get_parameter('color_topic').value),
            self._on_color, qos_profile_sensor_data)
        self._depth_sub = self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._on_depth, qos_profile_sensor_data)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1280, 520)
        cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)
        self.get_logger().info(
            'Read-only viewer ready. Move mouse to measure; '
            'left-click locks/unlocks; q or Esc exits.')

    def _on_color(self, message: Image) -> None:
        try:
            image = image_to_bgr(message)
        except (ValueError, TypeError) as exc:
            self._last_error = str(exc)
            return
        with self._lock:
            self._color = image
            self._color_time = time.monotonic()

    def _on_depth(self, message: Image) -> None:
        try:
            depth = image_to_depth_m(message)
        except (ValueError, TypeError) as exc:
            self._last_error = str(exc)
            return
        with self._lock:
            self._depth = depth
            self._depth_time = time.monotonic()

    def _on_mouse(self, event: int, x: int, y: int, _flags, _param) -> None:
        with self._lock:
            color_width = self._color.shape[1] if self._color is not None else 640
            if x >= color_width:
                return
            self._mouse = (x, y)
            if event == cv2.EVENT_LBUTTONDOWN:
                self._locked = None if self._locked is not None else (x, y)

    @staticmethod
    def _put_label(
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
        color: tuple[int, int, int] = (255, 255, 255),
    ) -> None:
        x, y = origin
        (width, height), baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.56, 1)
        cv2.rectangle(
            image, (x - 4, y - height - 5),
            (x + width + 4, y + baseline + 4), (20, 20, 20), -1)
        cv2.putText(
            image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
            0.56, color, 1, cv2.LINE_AA)

    def _depth_heatmap(self, depth: np.ndarray) -> np.ndarray:
        near = float(self.get_parameter('display_min_m').value)
        far = float(self.get_parameter('display_max_m').value)
        clipped = np.clip(depth, near, far)
        normalized = (far - clipped) / max(1e-6, far - near) * 255.0
        normalized[~np.isfinite(depth)] = 0.0
        heat = cv2.applyColorMap(
            normalized.astype(np.uint8), cv2.COLORMAP_TURBO)
        heat[~np.isfinite(depth)] = (0, 0, 0)
        return heat

    @staticmethod
    def _nearest_in_central_roi(
        depth: np.ndarray,
    ) -> tuple[int, int, float] | None:
        height, width = depth.shape
        x0, x1 = int(width * 0.15), int(width * 0.85)
        y0, y1 = int(height * 0.12), int(height * 0.88)
        roi = depth[y0:y1, x0:x1]
        valid = np.isfinite(roi) & (roi >= 0.25) & (roi <= 3.0)
        if np.count_nonzero(valid) < 25:
            return None
        threshold = float(np.percentile(roi[valid], 2.0))
        candidates = np.argwhere(valid & (roi <= threshold + 0.015))
        if not candidates.size:
            return None
        cy, cx = np.median(candidates, axis=0).astype(int)
        x, y = int(cx + x0), int(cy + y0)
        return x, y, robust_distance(depth, x, y, radius=5)

    def render(self) -> None:
        with self._lock:
            color = None if self._color is None else self._color.copy()
            depth = None if self._depth is None else self._depth.copy()
            color_time, depth_time = self._color_time, self._depth_time
            point = self._locked if self._locked is not None else self._mouse
            locked = self._locked is not None
            error = self._last_error

        if color is None or depth is None:
            canvas = np.zeros((480, 960, 3), dtype=np.uint8)
            missing = []
            if color is None:
                missing.append('RGB')
            if depth is None:
                missing.append('aligned depth')
            self._put_label(
                canvas, f'Waiting for {", ".join(missing)} ...', (30, 55),
                (0, 220, 255))
            if error:
                self._put_label(canvas, error, (30, 90), (0, 80, 255))
            cv2.imshow(WINDOW_NAME, canvas)
            self._handle_key()
            return

        if depth.shape != color.shape[:2]:
            depth = cv2.resize(
                depth, (color.shape[1], color.shape[0]),
                interpolation=cv2.INTER_NEAREST)

        height, width = depth.shape
        x = min(max(0, int(point[0])), width - 1)
        y = min(max(0, int(point[1])), height - 1)
        measured = robust_distance(depth, x, y)
        center = robust_distance(depth, width // 2, height // 2)

        heat = self._depth_heatmap(depth)
        cv2.drawMarker(
            color, (x, y), (0, 255, 255) if locked else (255, 255, 255),
            cv2.MARKER_CROSS, 22, 2)
        value = (
            'no valid depth' if not math.isfinite(measured)
            else f'{measured:.3f} m')
        prefix = 'LOCK ' if locked else ''
        self._put_label(
            color, f'{prefix}({x},{y}): {value}', (12, 27),
            (0, 255, 255) if locked else (255, 255, 255))

        cx, cy = width // 2, height // 2
        cv2.drawMarker(
            color, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 18, 1)
        center_text = (
            'Center: --' if not math.isfinite(center)
            else f'Center: {center:.3f} m')
        self._put_label(color, center_text, (12, 55), (0, 255, 0))

        nearest = self._nearest_in_central_roi(depth)
        if nearest is not None and math.isfinite(nearest[2]):
            nx, ny, distance = nearest
            cv2.circle(color, (nx, ny), 8, (0, 0, 255), 2)
            cv2.circle(heat, (nx, ny), 8, (255, 255, 255), 2)
            self._put_label(
                color, f'Near ROI: {distance:.3f} m', (12, 83),
                (0, 80, 255))

        now = time.monotonic()
        stale_after = float(self.get_parameter('stale_after_sec').value)
        if now - color_time > stale_after or now - depth_time > stale_after:
            self._put_label(
                color, 'STALE STREAM', (width - 155, 27), (0, 0, 255))

        valid_pct = float(np.mean(np.isfinite(depth)) * 100.0)
        self._put_label(
            heat, f'Depth {valid_pct:.1f}% valid | black=no depth', (12, 27))
        self._put_label(
            heat, '0.2 m (red)  <---->  3.0 m (blue)', (12, 55))
        cv2.imshow(WINDOW_NAME, np.hstack((color, heat)))
        self._handle_key()

    def _handle_key(self) -> None:
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self._quit = True

    @property
    def should_quit(self) -> bool:
        return self._quit

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = D436DepthViewer()
    try:
        while rclpy.ok() and not node.should_quit:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.render()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
