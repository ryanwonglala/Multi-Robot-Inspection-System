#!/usr/bin/env python3
"""
RealSense V4L2 color ROI intrusion alarm.

This intentionally avoids pyrealsense2 so it can run on Jetson systems where
the RealSense SDK Python bindings are not installed. It reads the D4xx Z16 depth
stream from /dev/video0 for detection and the YUYV color stream from /dev/video4
for the visible camera image.
"""

import argparse
import json
import os
import re
import select
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np


DEFAULT_CONFIG = Path.home() / ".config" / "realsense_roi_alarm" / "config.json"
DEFAULT_BACKGROUND = Path.home() / ".config" / "realsense_roi_alarm" / "background_roi.npz"


class RoiSelector:
    def __init__(self, roi=None):
        self.roi = roi
        self.dragging = False
        self.start = None
        self.current = None

    def callback(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start = (x, y)
            self.current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            self.current = (x, y)
            self.roi = normalized_roi(self.start, self.current)

    def preview_roi(self):
        if self.dragging and self.start and self.current:
            return normalized_roi(self.start, self.current)
        return self.roi

    def detection_roi(self):
        if self.dragging:
            return None
        return self.roi


def normalized_roi(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    x = min(x1, x2)
    y = min(y1, y2)
    w = abs(x2 - x1)
    h = abs(y2 - y1)
    if w < 4 or h < 4:
        return None
    return [int(x), int(y), int(w), int(h)]


def clamp_roi(roi, width, height):
    if not roi:
        return None
    x, y, w, h = [int(v) for v in roi]
    x = max(0, min(x, width - 1))
    y = max(0, min(y, height - 1))
    w = max(1, min(w, width - x))
    h = max(1, min(h, height - y))
    return [x, y, w, h]


def parse_roi(value):
    if not value:
        return None
    parts = [int(v.strip()) for v in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("ROI must be x,y,w,h")
    return parts


def load_config(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as exc:
        print(f"Warning: failed to load config {path}: {exc}", file=sys.stderr)
        return {}


def save_config(path, roi, args):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "roi": roi,
        "depth_device": args.depth_device,
        "color_device": args.color_device,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "near_mm": args.near_mm,
        "far_mm": args.far_mm,
        "delta_mm": args.delta_mm,
        "min_pixels": args.min_pixels,
        "min_ratio": args.min_ratio,
        "min_alarm_contour": args.min_alarm_contour,
        "color_delta": args.color_delta,
        "absolute_color_delta": args.absolute_color_delta,
        "hold_sec": args.hold_sec,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"Saved config: {path}")


class V4L2CtlRawReader:
    def __init__(self, device, width, height, fps, pixelformat, bytes_per_pixel):
        self.device = device
        self.width = width
        self.height = height
        self.pixelformat = pixelformat
        self.frame_size = width * height * bytes_per_pixel
        self.read_timeout_sec = 2.0
        self.proc = None
        self.start(fps)

    def start(self, fps):
        cmd = [
            "v4l2-ctl",
            "-d",
            self.device,
            f"--set-fmt-video=width={self.width},height={self.height},pixelformat={self.pixelformat}",
            f"--set-parm={fps}",
            "--stream-mmap",
            "--stream-to=-",
            "--stream-count=1000000000",
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if self.proc.stdout is None:
            raise RuntimeError("Failed to open v4l2-ctl stdout")

    def read(self):
        if self.proc is None or self.proc.stdout is None:
            return False, None

        chunks = []
        remaining = self.frame_size
        deadline = time.monotonic() + self.read_timeout_sec
        fd = self.proc.stdout.fileno()
        while remaining > 0:
            timeout = max(0.0, deadline - time.monotonic())
            if timeout == 0:
                return False, None
            readable, _, _ = select.select([fd], [], [], timeout)
            if not readable:
                return False, None
            chunk = os.read(fd, remaining)
            if not chunk:
                return False, None
            chunks.append(chunk)
            remaining -= len(chunk)

        raw = b"".join(chunks)
        return True, raw

    def release(self):
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.proc.stdout:
            self.proc.stdout.close()
        self.proc = None


class V4L2CtlDepthReader(V4L2CtlRawReader):
    def __init__(self, device, width, height, fps):
        super().__init__(device, width, height, fps, "Z16 ", 2)

    def read(self):
        ok, raw = super().read()
        if not ok:
            return False, None
        depth = np.frombuffer(raw, dtype=np.uint16).reshape((self.height, self.width))
        return True, depth


class V4L2CtlColorReader(V4L2CtlRawReader):
    def __init__(self, device, width, height, fps):
        super().__init__(device, width, height, fps, "YUYV", 2)

    def read(self):
        ok, raw = super().read()
        if not ok:
            return False, None
        yuyv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 2))
        bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        return True, bgr


class OpenCVColorReader:
    def __init__(self, device, width, height, fps):
        capture_device = device
        match = re.fullmatch(r"/dev/video(\d+)", str(device))
        if match:
            capture_device = int(match.group(1))

        self.cap = cv2.VideoCapture(capture_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open color device: {device}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

    def read(self):
        ok, frame = self.cap.read()
        return ok, frame

    def release(self):
        self.cap.release()


def frame_to_depth_mm(frame, width, height):
    if frame is None:
        return None

    if frame.dtype == np.uint16:
        depth = frame
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        return depth

    # Some OpenCV/V4L2 builds expose Z16 as packed uint8 bytes. Rebuild it.
    if frame.dtype == np.uint8:
        raw = frame.reshape(-1)
        expected = width * height * 2
        if raw.size >= expected:
            depth = raw[:expected].view(np.uint16).reshape((height, width))
            return depth

    return None


def colorize_depth(depth_mm, near_mm, far_mm):
    clipped = np.clip(depth_mm, near_mm, far_mm)
    scaled = ((clipped - near_mm) * 255.0 / max(1, far_mm - near_mm)).astype(np.uint8)
    scaled[depth_mm == 0] = 0
    # Near objects should look hotter/brighter.
    scaled = 255 - scaled
    return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)


def get_roi_depth(depth_mm, roi):
    x, y, w, h = roi
    return depth_mm[y : y + h, x : x + w]


def load_background(path, expected_roi):
    try:
        data = np.load(path, allow_pickle=False)
        roi = data["roi"].astype(np.int32).tolist()
        if roi == expected_roi:
            depth = data["depth"].astype(np.uint16) if "depth" in data.files else None
            color = data["color"].astype(np.uint8) if "color" in data.files else None
            print(f"Loaded background: {path}")
            return depth, color
        print("Saved background ROI differs from current ROI; ignoring background.")
    except FileNotFoundError:
        pass
    except Exception as exc:
        print(f"Warning: failed to load background {path}: {exc}", file=sys.stderr)
    return None, None


def save_background(path, roi, background_depth, background_color):
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"roi": np.asarray(roi, dtype=np.int32)}
    if background_depth is not None:
        payload["depth"] = background_depth.astype(np.uint16)
    if background_color is not None:
        payload["color"] = background_color.astype(np.uint8)
    np.savez_compressed(path, **payload)
    print(f"Saved background: {path}")


def capture_background(depth_cap, color_cap, width, height, roi, samples, near_mm, far_mm):
    depth_frames = []
    color_frames = []
    print("Capturing empty background...")
    for _ in range(samples):
        ok, frame = depth_cap.read()
        color_ok, color_frame = color_cap.read()
        if not ok or not color_ok:
            continue
        depth = frame_to_depth_mm(frame, width, height)
        if depth is None:
            continue
        roi_depth = get_roi_depth(depth, roi)
        x, y, w, h = roi
        roi_color = color_frame[y : y + h, x : x + w]
        valid = (roi_depth >= near_mm) & (roi_depth <= far_mm)
        cleaned = np.where(valid, roi_depth, 0).astype(np.uint16)
        depth_frames.append(cleaned)
        color_frames.append(roi_color.astype(np.uint8))
        time.sleep(0.03)

    if not depth_frames or not color_frames:
        raise RuntimeError("Could not capture any background frames")

    depth_stack = np.stack(depth_frames, axis=0)
    color_stack = np.stack(color_frames, axis=0)
    background_depth = np.median(depth_stack, axis=0).astype(np.uint16)
    background_color = np.median(color_stack, axis=0).astype(np.uint8)
    print("Background captured.")
    return background_depth, background_color


def run_alarm_command(command):
    if not command:
        return
    try:
        subprocess.Popen(command, shell=True)
    except Exception as exc:
        print(f"Warning: alarm command failed: {exc}", file=sys.stderr)


def refine_mask(mask_u8):
    kernel = np.ones((5, 5), np.uint8)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)
    return mask_u8


def detect_visible_object_without_background(roi_color, args):
    gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    median = int(np.median(gray))
    diff = cv2.absdiff(gray, np.full(gray.shape, median, dtype=np.uint8))
    _, mask_u8 = cv2.threshold(diff, args.absolute_color_delta, 255, cv2.THRESH_BINARY)
    return refine_mask(mask_u8)


def detect_intrusion(roi_depth, roi_color, depth_background, color_background, args):
    valid = (roi_depth >= args.near_mm) & (roi_depth <= args.far_mm)

    if depth_background is not None and depth_background.shape == roi_depth.shape:
        bg_valid = depth_background > 0
        closer = bg_valid & valid & ((depth_background.astype(np.int32) - roi_depth.astype(np.int32)) >= args.delta_mm)
        # Also catch objects appearing where the background had no valid depth.
        new_valid = (~bg_valid) & valid
        depth_mask = closer | new_valid
    else:
        depth_mask = valid

    depth_mask_u8 = refine_mask(depth_mask.astype(np.uint8) * 255)

    color_mask_u8 = None
    if color_background is not None and color_background.shape == roi_color.shape:
        bg_gray = cv2.cvtColor(color_background, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
        bg_gray = cv2.GaussianBlur(bg_gray, (5, 5), 0)
        frame_gray = cv2.GaussianBlur(frame_gray, (5, 5), 0)
        diff = cv2.absdiff(frame_gray, bg_gray)
        _, color_mask_u8 = cv2.threshold(diff, args.color_delta, 255, cv2.THRESH_BINARY)
        color_mask_u8 = refine_mask(color_mask_u8)
    else:
        color_mask_u8 = detect_visible_object_without_background(roi_color, args)

    # Use color differences for box placement when a color background exists.
    # Depth and color sensors are offset, so depth-only boxes can drift on close objects.
    mask_u8 = color_mask_u8 if color_mask_u8 is not None else depth_mask_u8
    alarm_mask_u8 = cv2.bitwise_or(mask_u8, depth_mask_u8)

    count = int(np.count_nonzero(alarm_mask_u8))
    area = int(alarm_mask_u8.shape[0] * alarm_mask_u8.shape[1])
    ratio = count / max(1, area)
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    max_contour = 0
    for contour in contours:
        contour_area = int(cv2.contourArea(contour))
        max_contour = max(max_contour, contour_area)
        if contour_area < args.min_box_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        contour_mask = np.zeros(mask_u8.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
        contour_depth_mask = (contour_mask > 0) & (roi_depth >= args.near_mm) & (roi_depth <= args.far_mm)
        depths = roi_depth[contour_depth_mask]
        median_box_mm = int(np.median(depths)) if depths.size else 0
        boxes.append((x, y, w, h, contour_area, median_box_mm))
    triggered_by_area = count >= args.min_pixels and ratio >= args.min_ratio
    triggered_by_contour = max_contour >= args.min_alarm_contour
    triggered = triggered_by_area or triggered_by_contour

    valid_depths = roi_depth[(mask_u8 > 0) & (roi_depth >= args.near_mm) & (roi_depth <= args.far_mm)]
    median_mm = int(np.median(valid_depths)) if valid_depths.size else 0
    return triggered, mask_u8, boxes, count, ratio, max_contour, median_mm


def draw_status(view, roi, boxes, triggered, hold_seconds, count, ratio, max_contour, median_mm, mode, args):
    h, w = view.shape[:2]
    if roi:
        x, y, rw, rh = roi
        color = (0, 0, 255) if triggered else (0, 255, 0)
        cv2.rectangle(view, (x, y), (x + rw, y + rh), color, 2)
        for bx, by, bw, bh, contour_area, box_median_mm in boxes:
            p1 = (x + bx, y + by)
            p2 = (x + bx + bw, y + by + bh)
            cv2.rectangle(view, p1, p2, (0, 255, 255), 2)
            label = f"object {box_median_mm}mm"
            label_y = max(18, p1[1] - 6)
            cv2.putText(view, label, (p1[0], label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(view, label, (p1[0], label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

    if triggered:
        overlay = view.copy()
        cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 180), -1)
        view[:] = cv2.addWeighted(overlay, 0.22, view, 0.78, 0)
        cv2.putText(view, "ALARM", (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

    lines = [
        f"mode:{mode} color_delta:{args.color_delta}/{args.absolute_color_delta} depth:{args.near_mm}-{args.far_mm}mm",
        f"pixels:{count} ratio:{ratio:.3f} contour:{max_contour} hold:{hold_seconds:.1f}/{args.hold_sec:.1f}s",
        "drag ROI | b background | s save | r reset ROI | q quit",
    ]
    y = h - 70
    for line in lines:
        cv2.putText(view, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(view, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        y += 22


def parse_args():
    parser = argparse.ArgumentParser(description="RealSense depth ROI intrusion alarm")
    parser.add_argument("--depth-device", default="/dev/video0", help="RealSense Z16 depth video device")
    parser.add_argument("--color-device", default="/dev/video4", help="RealSense YUYV color video device")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--roi", type=parse_roi, default=None, help="Initial ROI as x,y,w,h")
    parser.add_argument("--near-mm", type=int, default=200, help="Ignore depth closer than this")
    parser.add_argument("--far-mm", type=int, default=1800, help="Ignore depth farther than this")
    parser.add_argument("--delta-mm", type=int, default=120, help="Object must be this much closer than background")
    parser.add_argument("--min-pixels", type=int, default=1200, help="Minimum alarm pixels inside ROI")
    parser.add_argument("--min-ratio", type=float, default=0.015, help="Minimum alarm pixel ratio inside ROI")
    parser.add_argument("--min-box-area", type=int, default=120, help="Minimum contour area to draw an object box")
    parser.add_argument("--min-alarm-contour", type=int, default=180, help="Alarm if the largest detected contour reaches this area")
    parser.add_argument("--color-delta", type=int, default=30, help="Color difference threshold after background capture")
    parser.add_argument("--absolute-color-delta", type=int, default=45, help="Visible object threshold before background capture")
    parser.add_argument("--hold-sec", type=float, default=5.0, help="Object must remain detected this many seconds before alarm")
    parser.add_argument("--hold-frames", type=int, default=3, help="Consecutive triggered frames before alarm")
    parser.add_argument("--clear-frames", type=int, default=6, help="Consecutive clear frames before alarm clears")
    parser.add_argument("--background-samples", type=int, default=20)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--background-file", type=Path, default=DEFAULT_BACKGROUND)
    parser.add_argument("--no-load-background", action="store_true")
    parser.add_argument("--headless", action="store_true", help="Do not open a GUI window")
    parser.add_argument("--test-frames", type=int, default=0, help="Read this many frames and exit")
    parser.add_argument("--alarm-command", default="", help="Optional shell command to run when alarm starts")
    parser.add_argument("--alarm-repeat-sec", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    roi = args.roi or config.get("roi")

    depth_cap = V4L2CtlDepthReader(args.depth_device, args.width, args.height, args.fps)
    color_cap = OpenCVColorReader(args.color_device, args.width, args.height, args.fps)

    selector = RoiSelector(clamp_roi(roi, args.width, args.height))
    depth_background = None
    color_background = None
    if selector.roi and not args.no_load_background:
        depth_background, color_background = load_background(args.background_file, selector.roi)
    last_roi = selector.roi[:] if selector.roi else None

    window = "RealSense Color ROI Alarm"
    if not args.headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window, selector.callback)

    alarm_active = False
    triggered_streak = 0
    triggered_since = None
    hold_seconds = 0.0
    clear_streak = 0
    last_alarm_time = 0.0
    frame_count = 0
    read_failures = 0

    try:
        while True:
            ok, frame = depth_cap.read()
            if not ok:
                read_failures += 1
                print("Warning: failed to read depth frame", file=sys.stderr)
                if args.test_frames and read_failures >= 5:
                    raise RuntimeError("Too many depth read failures during test")
                time.sleep(0.05)
                continue

            color_ok, color_frame = color_cap.read()
            if not color_ok:
                read_failures += 1
                print("Warning: failed to read color frame", file=sys.stderr)
                if args.test_frames and read_failures >= 5:
                    raise RuntimeError("Too many color read failures during test")
                time.sleep(0.05)
                continue

            depth = frame_to_depth_mm(frame, args.width, args.height)
            if depth is None:
                raise RuntimeError(f"Unsupported frame format from {args.depth_device}: dtype={frame.dtype}, shape={frame.shape}")

            read_failures = 0
            frame_count += 1
            display_roi = clamp_roi(selector.preview_roi(), depth.shape[1], depth.shape[0])
            roi = clamp_roi(selector.detection_roi(), depth.shape[1], depth.shape[0])
            if roi != last_roi:
                depth_background = None
                color_background = None
                last_roi = roi[:] if roi else None
            view = color_frame.copy()

            triggered = False
            count = 0
            ratio = 0.0
            max_contour = 0
            median_mm = 0
            boxes = []

            if roi:
                roi_depth = get_roi_depth(depth, roi)
                x, y, rw, rh = roi
                roi_color = color_frame[y : y + rh, x : x + rw]
                triggered, mask, boxes, count, ratio, max_contour, median_mm = detect_intrusion(
                    roi_depth, roi_color, depth_background, color_background, args
                )

                if not args.headless:
                    red = np.zeros_like(view[y : y + rh, x : x + rw])
                    red[:, :, 2] = 255
                    mask_bool = mask.astype(bool)
                    view_roi = view[y : y + rh, x : x + rw]
                    if np.any(mask_bool):
                        view_roi[mask_bool] = (view_roi[mask_bool] * 0.45 + red[mask_bool] * 0.55).astype(np.uint8)

            if triggered:
                triggered_streak += 1
                if triggered_since is None:
                    triggered_since = time.monotonic()
                hold_seconds = time.monotonic() - triggered_since
                clear_streak = 0
            else:
                clear_streak += 1
                triggered_streak = 0
                triggered_since = None
                hold_seconds = 0.0

            if not alarm_active and triggered_streak >= args.hold_frames and hold_seconds >= args.hold_sec:
                alarm_active = True
                last_alarm_time = 0.0
                print(
                    f"ALARM start: held={hold_seconds:.1f}s pixels={count} "
                    f"ratio={ratio:.3f} median={median_mm}mm",
                    flush=True,
                )

            if alarm_active and clear_streak >= args.clear_frames:
                alarm_active = False
                print("ALARM cleared", flush=True)

            if alarm_active and (time.monotonic() - last_alarm_time) >= args.alarm_repeat_sec:
                print(f"ALARM active: pixels={count} ratio={ratio:.3f} median={median_mm}mm", flush=True)
                run_alarm_command(args.alarm_command)
                last_alarm_time = time.monotonic()

            if args.test_frames and frame_count >= args.test_frames:
                print(
                    f"TEST OK frames={frame_count} roi={roi} alarm={alarm_active} "
                    f"pixels={count} ratio={ratio:.3f} median={median_mm}mm"
                )
                break

            if args.headless:
                time.sleep(0.001)
                continue

            draw_status(
                view,
                display_roi,
                boxes,
                alarm_active,
                hold_seconds,
                count,
                ratio,
                max_contour,
                median_mm,
                "background+color" if color_background is not None else ("background" if depth_background is not None else "absolute+color"),
                args,
            )
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if key == ord("r"):
                selector.roi = None
                depth_background = None
                color_background = None
                print("ROI reset.")
            if key == ord("s") and roi:
                save_config(args.config, roi, args)
            if key == ord("b") and roi:
                depth_background, color_background = capture_background(
                    depth_cap,
                    color_cap,
                    args.width,
                    args.height,
                    roi,
                    args.background_samples,
                    args.near_mm,
                    args.far_mm,
                )
                save_background(args.background_file, roi, depth_background, color_background)

    finally:
        depth_cap.release()
        color_cap.release()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
