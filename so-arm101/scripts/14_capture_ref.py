"""采集空场参考照（参考帧差分检测的基准）。

用法: 把工作面清空（白纸上不留任何物品）后运行
    .venv/bin/python scripts/14_capture_ref.py

臂到观察位拍一张存 calibration/observe_ref.jpg。手指入镜是正常且必要的——
它属于"固定背景"的一部分，差分时自动免疫。
灯光变化、白纸挪动、观察位重录后都必须重拍。
"""

import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, load_pose, shutdown, smooth_goto
from soarm.camera_client import get_frame

ROOT = Path(__file__).parent.parent
OUT = ROOT / "calibration" / "observe_ref.jpg"

observe = load_pose("observe")
robot = connect()
print("前往观察位……")
smooth_goto(robot, observe, duration=3.0)
time.sleep(1.0)

# 多帧中值合成, 压掉传感器噪声与瞬时反光
frames = []
for _ in range(5):
    frames.append(get_frame().astype(np.float32))
    time.sleep(0.15)
ref = np.median(np.stack(frames), axis=0).astype(np.uint8)
cv2.imwrite(str(OUT), ref)
print(f"参考照已保存: {OUT}")

shutdown(robot)
print("完成。现在可以把目标物摆回工作面了。")
