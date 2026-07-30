"""从本机相机帧服务(scripts/camera_server.py)获取帧。"""

import json
import urllib.request

import cv2
import numpy as np

BASE = "http://127.0.0.1:8765"


def get_frame(timeout: float = 3.0) -> np.ndarray:
    """返回最新一帧 BGR 图像。服务未运行时抛出 ConnectionError。"""
    try:
        with urllib.request.urlopen(f"{BASE}/frame.jpg", timeout=timeout) as r:
            data = r.read()
    except OSError as e:
        raise ConnectionError(
            "相机服务不可达。请在终端运行: .venv/bin/python scripts/camera_server.py --index <N>"
        ) from e
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("帧解码失败")
    return frame


def get_status(timeout: float = 3.0) -> dict:
    with urllib.request.urlopen(f"{BASE}/status", timeout=timeout) as r:
        return json.loads(r.read())
