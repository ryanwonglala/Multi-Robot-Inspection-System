"""相机帧服务：独占持有 WebCam，通过 HTTP 在本机提供最新帧。

必须从「终端」启动（终端拥有摄像头权限），保持运行:
    .venv/bin/python scripts/camera_server.py --index 0

首次使用先枚举相机、确认臂上 WebCam 的 index:
    .venv/bin/python scripts/camera_server.py --list

接口:
    GET http://127.0.0.1:8765/           -> 浏览器实时预览页
    GET http://127.0.0.1:8765/stream     -> MJPEG 实时流
    GET http://127.0.0.1:8765/frame.jpg  -> 最新一帧 JPEG
    GET http://127.0.0.1:8765/status     -> JSON 状态
"""

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

HOST, PORT = "127.0.0.1", 8765
OUT_DIR = Path(__file__).parent.parent / "calibration"
ROI_FILE = Path(__file__).parent.parent / "config" / "roi.json"


def open_camera(index: int) -> cv2.VideoCapture:
    """Open the webcam with the native backend for the current platform."""
    if sys.platform == "darwin":
        return cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)

    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return cap


class RoiWatch:
    """工作区框(config/roi.json), 文件变了自动重载。只画在预览流上。"""

    def __init__(self):
        self.mtime = 0.0
        self.roi = None

    def get(self):
        try:
            mt = ROI_FILE.stat().st_mtime
            if mt != self.mtime:
                r = json.loads(ROI_FILE.read_text())
                self.roi = (r["x"], r["y"], r["w"], r["h"])
                self.mtime = mt
        except (OSError, json.JSONDecodeError, KeyError):
            pass
        return self.roi


def list_cameras() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    found = []
    for idx in range(4):
        cap = open_camera(idx)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok:
                h, w = frame.shape[:2]
                out = OUT_DIR / f"camera_{idx}_snapshot.jpg"
                cv2.imwrite(str(out), frame)
                print(f"相机 index={idx}: {w}x{h} -> 快照 {out.name}")
                found.append(idx)
        cap.release()
    print(f"\n可用 index: {found}；查看 calibration/ 下快照，确定臂上 WebCam 后用 --index 启动服务")


class FrameGrabber(threading.Thread):
    """后台线程持续取帧，保证 /frame.jpg 拿到的总是最新帧。"""

    def __init__(self, index: int):
        super().__init__(daemon=True)
        self.cap = open_camera(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开相机 index={index}（确认从终端运行且已授权摄像头）")
        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.view: bytes | None = None  # 带工作区框的预览版(仅 /stream 用)
        self.shape = None
        self.count = 0
        self.roi_watch = RoiWatch()

    def run(self):
        while True:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            roi = self.roi_watch.get()
            view_buf = None
            if roi:
                vis = frame.copy()
                x, y, w, h = roi
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.putText(vis, "work zone", (x + 8, max(y - 10, 25)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                ok2, vb = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
                view_buf = vb.tobytes() if ok2 else None
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()
                    self.view = view_buf or self.jpeg
                    self.shape = frame.shape
                    self.count += 1


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="枚举相机并保存快照")
    parser.add_argument("--index", type=int, default=0, help="相机 index")
    args = parser.parse_args()

    if args.list:
        list_cameras()
        return

    grabber = FrameGrabber(args.index)
    grabber.start()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # 静默访问日志
            pass

        def do_GET(self):
            if self.path == "/frame.jpg":
                with grabber.lock:
                    data = grabber.jpeg
                if data is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path == "/":
                body = (
                    "<title>SO-ARM101 相机</title>"
                    "<body style='margin:0;background:#111;display:grid;place-items:center;min-height:100vh'>"
                    "<img src='/stream' style='max-width:100vw;max-height:100vh'></body>"
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()
                try:
                    last = -1
                    while True:
                        with grabber.lock:
                            data, count = grabber.view or grabber.jpeg, grabber.count
                        if data is not None and count != last:
                            last = count
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                            self.wfile.write(data)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif self.path == "/status":
                with grabber.lock:
                    body = json.dumps(
                        {"index": args.index, "shape": grabber.shape, "frames": grabber.count}
                    ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

    print(f"相机服务已启动: http://{HOST}:{PORT}  (index={args.index}, Ctrl+C 退出)")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
