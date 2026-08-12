#!/usr/bin/env python3
"""
Gate a robot-arm command on RealSense ROI object dwell.

Default workflow:
1. Wait for the TurtleBot3 ready text signal.
2. Watch the saved RealSense ROI directly.
3. If a red object remains detected in the ROI for --hold-sec seconds,
   send the arm command.

The older TurtleBot3-arrival gate is still available with
--trigger-mode arrival. The ROI/background/detection code reuses
realsense_roi_alarm.py. Use that tool first to draw the ROI, press b to
capture background, and press s to save.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from realsense_roi_alarm import (
    DEFAULT_BACKGROUND,
    DEFAULT_CONFIG,
    OpenCVColorReader,
    V4L2CtlDepthReader,
    clamp_roi,
    draw_status,
    frame_to_depth_mm,
    get_roi_depth,
    load_background,
    load_config,
    parse_roi,
)


DEFAULT_TB3_READY_TEXT = "Ready, waiting for recognition results"
TRUE_WORDS = {
    "1",
    "true",
    "yes",
    "y",
    "arrived",
    "reached",
    "done",
    "ok",
    "load_unload_arrived",
    DEFAULT_TB3_READY_TEXT.lower(),
}
FALSE_WORDS = {"0", "false", "no", "n", "reset", "left", "departed", "cancel"}
DEFAULT_AUTO_CLEAR_COMMAND = (
    "cd ${SOARM_ROOT:-/home/nvidia/Multi-Robot-Inspection-System/so-arm101} && "
    "SOARM_PORT=${SOARM_PORT:-/dev/ttyACM0} .venv/bin/python auto_clear.py"
)


class YoloOverlay:
    def __init__(self, args, logger):
        self.args = args
        self.logger = logger
        self.enabled = bool(args.yolo and args.display)
        self.model = None
        self.device = "cpu"
        self.half = False
        self.last_infer_time = 0.0
        self.smoothed_fps = 0.0
        self.detections = []

        if self.enabled:
            self.load_model()

    def load_model(self):
        try:
            import torch
            from ultralytics import YOLO

            cuda_available = torch.cuda.is_available()
            if self.args.yolo_device == "auto":
                self.device = "0" if cuda_available else "cpu"
            else:
                self.device = self.args.yolo_device
            self.half = bool(self.args.yolo_half) and self.device != "cpu"
            self.model = YOLO(self.args.yolo_model)
            self.logger.info(
                f"YOLO overlay ready: model={self.args.yolo_model}, "
                f"device={self.device}, half={self.half}"
            )
        except Exception as exc:
            self.enabled = False
            self.model = None
            self.logger.warning(f"YOLO overlay disabled: {exc}")

    def update(self, frame):
        if not self.enabled or self.model is None:
            return self.detections

        now = time.monotonic()
        if now - self.last_infer_time < 1.0 / max(0.5, self.args.yolo_rate):
            return self.detections

        start = time.perf_counter()
        try:
            result = self.model.predict(
                source=frame,
                conf=self.args.yolo_conf,
                iou=self.args.yolo_iou,
                imgsz=self.args.yolo_imgsz,
                device=self.device,
                half=self.half,
                verbose=False,
            )[0]
        except Exception as exc:
            self.logger.warning(f"YOLO inference failed: {exc}")
            self.last_infer_time = now
            return self.detections

        elapsed = max(1e-6, time.perf_counter() - start)
        instant_fps = 1.0 / elapsed
        self.smoothed_fps = instant_fps if self.smoothed_fps == 0.0 else 0.85 * self.smoothed_fps + 0.15 * instant_fps
        self.last_infer_time = now

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                x1, y1, x2, y2 = [int(round(v)) for v in box.xyxy[0].tolist()]
                detections.append(
                    {
                        "label": str(result.names[class_id]),
                        "confidence": confidence,
                        "bbox": (x1, y1, x2, y2),
                    }
                )
        self.detections = detections
        return detections

    def draw(self, view):
        if not self.enabled:
            return

        for det in self.detections:
            x1, y1, x2, y2 = det["bbox"]
            label = f"{det['label']} {det['confidence']:.2f}"
            cv2.rectangle(view, (x1, y1), (x2, y2), (255, 180, 0), 2)
            label_y = max(18, y1 - 7)
            cv2.putText(view, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(view, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 0), 1, cv2.LINE_AA)

        text = f"YOLO {len(self.detections)} objects"
        if self.smoothed_fps > 0:
            text += f" {self.smoothed_fps:.1f} FPS"
        cv2.putText(view, text, (18, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, text, (18, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 180, 0), 1, cv2.LINE_AA)


class TurtlebotArmGate(Node):
    def __init__(self, args):
        super().__init__("turtlebot3_load_arm_gate")
        self.args = args
        self.arrived = args.trigger_mode == "vision"
        self.command_sent = False
        self.detected_since = None
        self.hold_seconds = 0.0
        self.last_status_time = 0.0

        self.arrival_sub = None
        self.ready_text = args.arrival_ready_text.strip().lower()
        if args.trigger_mode == "arrival" and args.arrival_type == "bool":
            self.arrival_sub = self.create_subscription(Bool, args.arrival_topic, self.bool_arrival_cb, 10)
        elif args.trigger_mode == "arrival":
            self.arrival_sub = self.create_subscription(String, args.arrival_topic, self.string_arrival_cb, 10)

        self.arm_pub = self.create_publisher(String, args.arm_command_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)

        if args.trigger_mode == "vision":
            self.get_logger().info(
                f"Camera direct trigger enabled: detect_mode={args.detect_mode}, hold={args.hold_sec:.1f}s; "
                f"arm command topic={args.arm_command_topic}"
            )
        else:
            self.get_logger().info(
                f"Waiting for TurtleBot3 arrival on {args.arrival_topic} ({args.arrival_type}); "
                f"ready_text={args.arrival_ready_text!r}; arm command topic={args.arm_command_topic}"
            )

    def bool_arrival_cb(self, msg):
        self.set_arrived(bool(msg.data), source="bool")

    def string_arrival_cb(self, msg):
        text = msg.data.strip().lower()
        if text in TRUE_WORDS or (self.ready_text and self.ready_text in text):
            self.set_arrived(True, source=msg.data)
        elif text in FALSE_WORDS:
            self.set_arrived(False, source=msg.data)

    def set_arrived(self, value, source):
        if value == self.arrived:
            return
        self.arrived = value
        self.detected_since = None
        self.hold_seconds = 0.0
        if not value:
            self.command_sent = False
        self.get_logger().info(f"TurtleBot3 arrived={self.arrived} source={source}")

    def send_arm_command(self, detail):
        if self.command_sent and self.args.single_shot:
            return

        msg = String()
        msg.data = self.args.arm_command
        for _ in range(max(1, self.args.arm_publish_count)):
            self.arm_pub.publish(msg)
            time.sleep(0.02)

        self.command_sent = True
        if self.args.arm_shell_command:
            proc = subprocess.Popen(self.args.arm_shell_command, shell=True)
            self.get_logger().info(f"Started shell command pid={proc.pid}: {self.args.arm_shell_command}")
        self.get_logger().info(f"Arm command sent: {self.args.arm_command}; {detail}")

    def publish_status(self, state, **extra):
        now = time.monotonic()
        if now - self.last_status_time < self.args.status_period:
            return
        self.last_status_time = now

        msg = String()
        msg.data = json.dumps(
            {
                "state": state,
                "trigger_mode": self.args.trigger_mode,
                "arrived": self.arrived,
                "command_sent": self.command_sent,
                "hold_seconds": round(self.hold_seconds, 2),
                **extra,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)


def draw_gate_overlay(view, state, trigger_mode, detect_mode, arrived, command_sent):
    if command_sent:
        text = "ARM COMMAND SENT"
        color = (0, 200, 255)
    elif trigger_mode == "vision":
        target = "RED OBJECT" if detect_mode == "red" else "ROBOT"
        if state == "holding_target":
            text = f"{target} IN ROI - HOLDING"
            color = (0, 255, 0)
        else:
            text = f"WAITING FOR {target} IN ROI"
            color = (0, 255, 255)
    elif arrived:
        text = "TURTLEBOT3 ARRIVED - CHECKING ROI"
        color = (0, 255, 0)
    else:
        text = "WAITING FOR TURTLEBOT3 ARRIVAL"
        color = (0, 255, 255)

    cv2.putText(view, text, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, text, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA)
    cv2.putText(view, f"state:{state}", (18, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, f"state:{state}", (18, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv2.LINE_AA)


def build_detection_args(args):
    return SimpleNamespace(
        near_mm=args.near_mm,
        far_mm=args.far_mm,
        delta_mm=args.delta_mm,
        min_pixels=args.min_pixels,
        min_ratio=args.min_ratio,
        min_box_area=args.min_box_area,
        min_alarm_contour=args.min_alarm_contour,
        color_delta=args.color_delta,
        absolute_color_delta=args.absolute_color_delta,
        hold_sec=args.hold_sec,
    )


def refine_red_mask(mask_u8):
    kernel = np.ones((5, 5), np.uint8)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)
    return mask_u8


def detect_red_object(roi_depth, roi_color, args):
    hsv = cv2.cvtColor(roi_color, cv2.COLOR_BGR2HSV)
    lower_red_1 = np.array([0, args.red_s_min, args.red_v_min], dtype=np.uint8)
    upper_red_1 = np.array([args.red_hue1_max, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([args.red_hue2_min, args.red_s_min, args.red_v_min], dtype=np.uint8)
    upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)
    mask_u8 = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red_1, upper_red_1),
        cv2.inRange(hsv, lower_red_2, upper_red_2),
    )
    mask_u8 = refine_red_mask(mask_u8)

    count = int(np.count_nonzero(mask_u8))
    area = int(mask_u8.shape[0] * mask_u8.shape[1])
    ratio = count / max(1, area)
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    max_contour = 0
    for contour in contours:
        contour_area = int(cv2.contourArea(contour))
        max_contour = max(max_contour, contour_area)
        if contour_area < args.min_box_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        contour_mask = np.zeros(mask_u8.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
        contour_depth_mask = (
            (contour_mask > 0)
            & (roi_depth >= args.near_mm)
            & (roi_depth <= args.far_mm)
        )
        depths = roi_depth[contour_depth_mask]
        median_box_mm = int(np.median(depths)) if depths.size else 0
        boxes.append((bx, by, bw, bh, contour_area, median_box_mm))

    valid_depths = roi_depth[(mask_u8 > 0) & (roi_depth >= args.near_mm) & (roi_depth <= args.far_mm)]
    median_mm = int(np.median(valid_depths)) if valid_depths.size else 0
    triggered = (
        count >= args.red_min_pixels
        and ratio >= args.red_min_ratio
        and max_contour >= args.red_min_contour
    )
    return triggered, mask_u8, boxes, count, ratio, max_contour, median_mm


def detect_roi_target(roi_depth, roi_color, depth_background, color_background, args, detection_args):
    if args.detect_mode == "red":
        return detect_red_object(roi_depth, roi_color, args)

    from realsense_roi_alarm import detect_intrusion

    return detect_intrusion(roi_depth, roi_color, depth_background, color_background, detection_args)


def list_v4l2_formats(device):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-formats-ext"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5.0,
        )
    except Exception:
        return ""
    return result.stdout or ""


def find_realsense_devices():
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--list-devices"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5.0,
        )
    except Exception:
        return None, None

    devices = []
    in_realsense_block = False
    for raw_line in (result.stdout or "").splitlines():
        line = raw_line.strip()
        if not line:
            in_realsense_block = False
            continue
        if not raw_line.startswith((" ", "\t")):
            in_realsense_block = "realsense" in line.lower()
            continue
        if in_realsense_block and line.startswith("/dev/video"):
            devices.append(line.split()[0])

    depth_device = None
    color_device = None
    for device in devices:
        formats = list_v4l2_formats(device)
        if depth_device is None and ("16-bit Depth" in formats or "'Z16 '" in formats):
            depth_device = device
        if color_device is None and ("YUYV 4:2:2" in formats or "'YUYV'" in formats):
            color_device = device

    return depth_device, color_device


def resolve_camera_devices(args):
    if args.depth_device != "auto" and args.color_device != "auto":
        return

    depth_device, color_device = find_realsense_devices()
    if args.depth_device == "auto":
        if not depth_device:
            raise RuntimeError("Could not auto-detect RealSense depth device. Pass --depth-device /dev/videoN.")
        args.depth_device = depth_device
    if args.color_device == "auto":
        if not color_device:
            raise RuntimeError("Could not auto-detect RealSense color device. Pass --color-device /dev/videoN.")
        args.color_device = color_device

    print(f"Auto-selected RealSense devices: depth={args.depth_device} color={args.color_device}", file=sys.stderr)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="RealSense ROI dwell + arm command gate")
    parser.add_argument(
        "--trigger-mode",
        choices=("vision", "arrival"),
        default="vision",
        help="vision: camera ROI triggers directly; arrival: wait for TurtleBot3 arrival topic first",
    )
    parser.add_argument("--arrival-topic", default="/turtlebot3/load_unload_arrived")
    parser.add_argument("--arrival-type", choices=("bool", "string"), default="bool")
    parser.add_argument(
        "--arrival-ready-text",
        default=DEFAULT_TB3_READY_TEXT,
        help="String message treated as TurtleBot3 arrival/ready signal",
    )
    parser.add_argument("--arm-command-topic", default="/arm/command")
    parser.add_argument("--arm-command", default="start_load_unload")
    parser.add_argument(
        "--arm-shell-command",
        default=DEFAULT_AUTO_CLEAR_COMMAND,
        help="Shell command to run after the ROS arm command; pass an empty string to disable",
    )
    parser.add_argument("--arm-publish-count", type=int, default=3)
    parser.add_argument("--status-topic", default="/load_unload_gate/status")
    parser.add_argument("--status-period", type=float, default=1.0)
    parser.add_argument("--single-shot", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument(
        "--detect-mode",
        choices=("red", "roi-change"),
        default="red",
        help="red: only red pixels inside ROI trigger; roi-change: old depth/color-change logic",
    )
    parser.add_argument("--depth-device", default="auto")
    parser.add_argument("--color-device", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--loop-hz", type=float, default=15.0)

    parser.add_argument("--roi", type=parse_roi, default=None, help="ROI as x,y,w,h; overrides saved config")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--background-file", type=Path, default=DEFAULT_BACKGROUND)
    parser.add_argument("--no-load-background", action="store_true")

    parser.add_argument("--near-mm", type=int, default=200)
    parser.add_argument("--far-mm", type=int, default=1800)
    parser.add_argument("--delta-mm", type=int, default=120)
    parser.add_argument("--min-pixels", type=int, default=1200)
    parser.add_argument("--min-ratio", type=float, default=0.015)
    parser.add_argument("--min-box-area", type=int, default=120)
    parser.add_argument("--min-alarm-contour", type=int, default=180)
    parser.add_argument("--color-delta", type=int, default=30)
    parser.add_argument("--absolute-color-delta", type=int, default=45)
    parser.add_argument("--red-hue1-max", type=int, default=12)
    parser.add_argument("--red-hue2-min", type=int, default=168)
    parser.add_argument("--red-s-min", type=int, default=70)
    parser.add_argument("--red-v-min", type=int, default=40)
    parser.add_argument("--red-min-pixels", type=int, default=180)
    parser.add_argument("--red-min-ratio", type=float, default=0.001)
    parser.add_argument("--red-min-contour", type=int, default=100)
    parser.add_argument("--hold-sec", type=float, default=5.0)
    parser.add_argument("--display", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--window-name", default="TurtleBot3 Load/Unload Arm Gate")

    parser.add_argument("--yolo", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--yolo-model",
        default=os.environ.get("YOLO_MODEL", "/home/nvidia/yolo11n.pt"),
    )
    parser.add_argument("--yolo-conf", type=float, default=0.35)
    parser.add_argument("--yolo-iou", type=float, default=0.45)
    parser.add_argument("--yolo-imgsz", type=int, default=640)
    parser.add_argument("--yolo-rate", type=float, default=5.0)
    parser.add_argument("--yolo-device", default="auto")
    parser.add_argument("--yolo-half", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv)
    resolve_camera_devices(args)
    config = load_config(args.config)
    roi = args.roi or config.get("roi")
    if not roi:
        roi = [
            int(args.width * 0.22),
            int(args.height * 0.28),
            int(args.width * 0.56),
            int(args.height * 0.42),
        ]
        print(
            f"No saved ROI found; using default ROI {roi}. "
            "Pass --roi x,y,w,h to override.",
            file=sys.stderr,
        )

    roi = clamp_roi(roi, args.width, args.height)
    depth_background = None
    color_background = None
    if not args.no_load_background:
        depth_background, color_background = load_background(args.background_file, roi)

    detection_args = build_detection_args(args)
    depth_cap = V4L2CtlDepthReader(args.depth_device, args.width, args.height, args.fps)
    color_cap = OpenCVColorReader(args.color_device, args.width, args.height, args.fps)

    rclpy.init(args=ros_args)
    node = TurtlebotArmGate(args)
    yolo_overlay = YoloOverlay(args, node.get_logger())

    if args.display:
        cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)

    period = 1.0 / max(0.5, args.loop_hz)
    state = "waiting_target" if args.trigger_mode == "vision" else "waiting_arrival"
    try:
        while rclpy.ok():
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)

            ok, depth_frame = depth_cap.read()
            color_ok, color_frame = color_cap.read()
            if not ok or not color_ok:
                node.publish_status("camera_read_failed")
                time.sleep(0.05)
                continue

            depth = frame_to_depth_mm(depth_frame, args.width, args.height)
            if depth is None:
                node.publish_status("bad_depth_frame")
                continue

            view = color_frame.copy()
            yolo_detections = yolo_overlay.update(color_frame) if args.display else []
            x, y, w, h = roi
            boxes = []
            count = 0
            ratio = 0.0
            max_contour = 0
            median_mm = 0
            object_detected = False
            mask = np.zeros((h, w), dtype=np.uint8)

            roi_depth = get_roi_depth(depth, roi)
            roi_color = color_frame[y : y + h, x : x + w]
            object_detected, mask, boxes, count, ratio, max_contour, median_mm = detect_roi_target(
                roi_depth, roi_color, depth_background, color_background, args, detection_args
            )

            gate_open = args.trigger_mode == "vision" or node.arrived
            if gate_open and not (node.command_sent and args.single_shot):
                if object_detected:
                    if node.detected_since is None:
                        node.detected_since = time.monotonic()
                    node.hold_seconds = time.monotonic() - node.detected_since
                    state = "holding_target" if args.trigger_mode == "vision" else "holding_object"
                else:
                    node.detected_since = None
                    node.hold_seconds = 0.0
                    state = "waiting_target" if args.trigger_mode == "vision" else "waiting_object"

                if object_detected and node.hold_seconds >= args.hold_sec:
                    detail = f"hold={node.hold_seconds:.1f}s contour={max_contour} ratio={ratio:.3f} median={median_mm}mm"
                    node.send_arm_command(detail)
                    state = "arm_command_sent"
            elif node.command_sent and args.single_shot:
                state = "arm_command_sent"
            else:
                node.detected_since = None
                node.hold_seconds = 0.0
                state = "waiting_arrival"

            if args.display:
                if np.any(mask):
                    red = np.zeros_like(view[y : y + h, x : x + w])
                    red[:, :, 2] = 255
                    mask_bool = mask.astype(bool)
                    view_roi = view[y : y + h, x : x + w]
                    view_roi[mask_bool] = (view_roi[mask_bool] * 0.45 + red[mask_bool] * 0.55).astype(np.uint8)

                if args.detect_mode == "red":
                    mode = "red-only"
                else:
                    mode = "background+color" if color_background is not None else (
                        "background" if depth_background is not None else "absolute+color"
                    )
                yolo_overlay.draw(view)
                draw_status(
                    view,
                    roi,
                    boxes,
                    node.command_sent,
                    node.hold_seconds,
                    count,
                    ratio,
                    max_contour,
                    median_mm,
                    mode,
                    detection_args,
                )
                draw_gate_overlay(view, state, args.trigger_mode, args.detect_mode, node.arrived, node.command_sent)
                cv2.imshow(args.window_name, view)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            node.publish_status(
                state,
                object_detected=object_detected,
                detect_mode=args.detect_mode,
                pixels=count,
                ratio=round(ratio, 4),
                contour=max_contour,
                median_mm=median_mm,
                yolo_count=len(yolo_detections),
                yolo_labels=[det["label"] for det in yolo_detections],
            )

            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        depth_cap.release()
        color_cap.release()
        if args.display:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
