"""示教采样：采集 (球的像素坐标, 抓取关节角) 样本对。

在终端运行（需要键盘交互）:
    .venv/bin/python scripts/06_collect_samples.py

每轮流程:
  1. 臂在观察位自动检测球的像素坐标（工作区只放一颗球）
  2. 臂释放扭矩，你手把手把夹爪摆到"正好能夹住球"的位姿
  3. 按 Enter 记录 -> 臂自动回观察位，把球挪到新位置继续
  4. 输入 q 结束（样本已随手保存，中途退出不丢数据）

样本追加写入 calibration/samples.json。
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import JOINTS, connect, hold_current, load_pose, read_joints, shutdown, smooth_goto
from soarm.camera_client import get_frame
from soarm.vision import detect_blobs

SAMPLES_FILE = Path(__file__).parent.parent / "calibration" / "samples.json"

samples = json.loads(SAMPLES_FILE.read_text()) if SAMPLES_FILE.exists() else []
print(f"已有样本 {len(samples)} 条（继续追加）")

observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

while True:
    cmd = input(f"\n[样本{len(samples)}] 放好球后按 Enter 检测 (q=结束) > ").strip().lower()
    if cmd == "q":
        break
    time.sleep(0.3)
    blobs = detect_blobs(get_frame())
    if len(blobs) != 1:
        print(f"检测到 {len(blobs)} 个目标，采样时工作区只放一颗球，调整后重试")
        continue
    px, py = blobs[0].cx, blobs[0].cy
    print(f"球的像素坐标: ({px:.0f}, {py:.0f})")

    robot.bus.disable_torque()
    input("臂已释放——把夹爪摆到正好能夹住球的位姿（指尖包住球，不用夹紧），按 Enter 记录 > ")
    joints = read_joints(robot)
    samples.append({"pixel": [round(px, 1), round(py, 1)], "joints": {j: round(joints[j], 2) for j in JOINTS}})
    SAMPLES_FILE.write_text(json.dumps(samples, indent=2, ensure_ascii=False))
    print(f"已记录，共 {len(samples)} 条。臂回观察位……")

    hold_current(robot)
    smooth_goto(robot, observe, duration=2.5)

print(f"采样结束，共 {len(samples)} 条 -> {SAMPLES_FILE.name}")
shutdown(robot)
