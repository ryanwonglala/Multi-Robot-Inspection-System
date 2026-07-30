"""验证观察位姿：home -> observe -> 抓帧保存 -> 回 home 断电。

用法: .venv/bin/python scripts/05_verify_observe.py
"""

import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, load_pose, read_joints, shutdown, smooth_goto
from soarm.camera_client import get_frame

ROOT = Path(__file__).parent.parent
observe = load_pose("observe")

robot = connect()
print("前往观察位……")
smooth_goto(robot, observe, duration=3.0)
time.sleep(1.0)
actual = read_joints(robot)
print("到位误差:", {k: round(actual[k] - observe[k], 1) for k in observe})

frame = get_frame()
out = ROOT / "calibration" / "observe_view.jpg"
cv2.imwrite(str(out), frame)
print(f"观察位画面已保存: {out.name}")

print("回 home 断电……")
shutdown(robot)
print("完成")
