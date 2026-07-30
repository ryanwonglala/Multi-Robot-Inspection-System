"""通过 AVFoundation 原生接口请求摄像头授权，并等待用户在系统弹框上作出选择。

用法: .venv/bin/python scripts/00_request_camera_permission.py
"""

import threading

import AVFoundation as AVF

STATUS_NAMES = {0: "notDetermined(未询问)", 1: "restricted(受限)", 2: "denied(已拒绝)", 3: "authorized(已授权)"}

status = AVF.AVCaptureDevice.authorizationStatusForMediaType_(AVF.AVMediaTypeVideo)
print(f"当前授权状态: {STATUS_NAMES.get(status, status)}")

if status == 0:
    done = threading.Event()
    result = {}

    def handler(granted):
        result["granted"] = granted
        done.set()

    print("已发起授权请求，等待系统弹框（最长 120 秒）……请在弹框上点“允许”")
    AVF.AVCaptureDevice.requestAccessForMediaType_completionHandler_(AVF.AVMediaTypeVideo, handler)
    if done.wait(timeout=120):
        print(f"用户选择: {'允许 ✓' if result['granted'] else '拒绝 ✗'}")
    else:
        print("等待超时，未收到用户选择（可能弹框未出现）")
elif status == 3:
    print("已授权，无需操作 ✓")
else:
    print("处于拒绝/受限状态：需要在 系统设置→隐私与安全性→摄像头 中手动开启，或 tccutil reset 后重试")
