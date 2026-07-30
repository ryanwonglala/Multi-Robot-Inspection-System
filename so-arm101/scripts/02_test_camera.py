"""冒烟测试：枚举可用相机，抓取一帧保存到 calibration/ 目录。

首次运行 macOS 会弹出摄像头权限请求，需要允许终端访问。
用法: .venv/bin/python scripts/02_test_camera.py
"""

from pathlib import Path

import cv2

OUT_DIR = Path(__file__).parent.parent / "calibration"
OUT_DIR.mkdir(exist_ok=True)

found = []
for idx in range(4):
    cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        cap.release()
        continue
    ok, frame = cap.read()
    if ok:
        h, w = frame.shape[:2]
        fps = cap.get(cv2.CAP_PROP_FPS)
        out = OUT_DIR / f"camera_{idx}_snapshot.jpg"
        cv2.imwrite(str(out), frame)
        print(f"相机 index={idx}: {w}x{h} @ {fps:.0f}fps -> 已保存 {out.name}")
        found.append(idx)
    cap.release()

if not found:
    print("未找到可用相机。请检查: 系统设置 -> 隐私与安全性 -> 摄像头 中终端的权限")
else:
    print(f"\n可用相机 index: {found}")
    print("请打开 calibration/ 下的快照确认哪个 index 是臂上的 WebCam（另一个通常是 Mac 内置相机）")
