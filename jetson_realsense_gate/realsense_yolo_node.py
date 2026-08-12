#!/usr/bin/env python3

import json
import os
import threading
import time
from typing import Optional

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class RealSenseYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("realsense_yolo_node")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "model_path", os.environ.get("YOLO_MODEL", "/home/nvidia/yolo11n.pt")
        )
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("inference_rate", 10.0)
        self.declare_parameter("device", "auto")
        self.declare_parameter("half_precision", True)
        self.declare_parameter("display", True)
        self.declare_parameter("publish_annotated", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.inference_rate = max(
            0.5, float(self.get_parameter("inference_rate").value)
        )
        requested_device = str(self.get_parameter("device").value)
        self.display = bool(self.get_parameter("display").value)
        self.publish_annotated = bool(
            self.get_parameter("publish_annotated").value
        )

        cuda_available = torch.cuda.is_available()
        self.device = (
            "0" if requested_device == "auto" and cuda_available else requested_device
        )
        if self.device == "auto":
            self.device = "cpu"
        self.half_precision = bool(
            self.get_parameter("half_precision").value
        ) and self.device != "cpu"

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame: Optional[object] = None
        self.last_frame_stamp = None
        self.last_processed_stamp = None
        self.smoothed_fps = 0.0

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, sensor_qos
        )
        self.detections_pub = self.create_publisher(String, "/yolo/detections", 10)
        self.annotated_pub = self.create_publisher(
            Image, "/yolo/annotated_image", sensor_qos
        )
        self.timer = self.create_timer(1.0 / self.inference_rate, self.process_frame)

        if self.display:
            cv2.namedWindow("RealSense YOLO", cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "YOLO ready: "
            f"topic={self.image_topic}, device={self.device}, "
            f"half={self.half_precision}, conf={self.confidence:.2f}"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.frame_lock:
            self.latest_frame = frame
            self.last_frame_stamp = stamp

    def process_frame(self) -> None:
        with self.frame_lock:
            if self.latest_frame is None:
                return
            if self.last_frame_stamp == self.last_processed_stamp:
                return
            frame = self.latest_frame.copy()
            frame_stamp = self.last_frame_stamp

        start = time.perf_counter()
        try:
            result = self.model.predict(
                source=frame,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                device=self.device,
                half=self.half_precision,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        elapsed = max(1e-6, time.perf_counter() - start)
        instant_fps = 1.0 / elapsed
        if self.smoothed_fps == 0.0:
            self.smoothed_fps = instant_fps
        else:
            self.smoothed_fps = 0.85 * self.smoothed_fps + 0.15 * instant_fps

        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                detections.append(
                    {
                        "class_id": class_id,
                        "label": str(result.names[class_id]),
                        "confidence": round(confidence, 4),
                        "bbox_xyxy": [
                            round(x1, 1),
                            round(y1, 1),
                            round(x2, 1),
                            round(y2, 1),
                        ],
                    }
                )

        summary = String()
        summary.data = json.dumps(
            {
                "frame_stamp": frame_stamp,
                "inference_fps": round(self.smoothed_fps, 2),
                "count": len(detections),
                "detections": detections,
            },
            separators=(",", ":"),
        )
        self.detections_pub.publish(summary)

        annotated = result.plot()
        cv2.putText(
            annotated,
            f"YOLO {self.smoothed_fps:.1f} FPS | {len(detections)} objects",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if self.publish_annotated:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.annotated_pub.publish(annotated_msg)

        if self.display:
            cv2.imshow("RealSense YOLO", annotated)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                rclpy.shutdown()

        self.last_processed_stamp = frame_stamp

    def destroy_node(self) -> bool:
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealSenseYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
