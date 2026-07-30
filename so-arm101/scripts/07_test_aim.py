"""对位测试 + 全局偏置标定（逐关节直控版）。

在终端运行:
    .venv/bin/python scripts/07_test_aim.py

每轮: 放球 -> Enter -> 臂移动到预测位姿 -> 按键逐关节微调(可长按连续调):
    底座 a/z | 肩 w/s | 肘 e/d | 腕俯仰 r/f | 腕旋转 t/g   (上排加/下排减)
    Enter = 本位置满意回观察位    x = 保存并结束
调整量累积为全局偏置，实时保存 config/offsets.json，对所有抓取生效。
"""

import json
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, goto_exact, load_pose, shutdown, smooth_goto
from soarm.camera_client import get_frame
from soarm.mapping import MAP_JOINTS, PixelToJoints
from soarm.vision import detect_blobs

OFFSETS_FILE = Path(__file__).parent.parent / "config" / "offsets.json"
GRIPPER_OPEN = 30.0
STEP = 0.8  # 每次击键度数（终端连发约5次/秒 -> 长按约4°/s）

KEYMAP = {  # 按键 -> (关节, 方向)
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
}

offsets = {j: 0.0 for j in MAP_JOINTS}
if OFFSETS_FILE.exists():
    offsets.update(json.loads(OFFSETS_FILE.read_text()))


def apply(target: dict) -> dict:
    return {j: target[j] + offsets.get(j, 0.0) for j in target}


def show_offsets() -> str:
    return " ".join(f"{j.split('_')[0][:4]}{offsets[j]:+.1f}" for j in MAP_JOINTS)


def jog(robot, base_target) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("微调: 底座a/z 肩w/s 肘e/d 腕俯r/f 腕旋t/g | Enter=满意 | x=结束")
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch in ("\n", "\r"):
                return "next"
            if ch == "x":
                return "quit"
            if ch not in KEYMAP:
                continue
            joint, sign = KEYMAP[ch]
            offsets[joint] += sign * STEP
            OFFSETS_FILE.write_text(json.dumps(offsets, indent=2))
            t = apply(base_target)
            robot.send_action({**{f"{j}.pos": v for j, v in t.items()}, "gripper.pos": GRIPPER_OPEN})
            sys.stdout.write(f"\r偏置: {show_offsets()}   ")
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


print("当前全局偏置:", offsets)
mapping = PixelToJoints()
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

while True:
    cmd = input("\n放好球后按 Enter 对位 (x=结束) > ").strip().lower()
    if cmd == "x":
        break
    time.sleep(0.3)
    blobs = detect_blobs(get_frame())
    if len(blobs) != 1:
        print(f"检测到 {len(blobs)} 个目标，请只放一颗球")
        continue
    px, py = blobs[0].cx, blobs[0].cy
    base_target = mapping(px, py)
    print(f"像素({px:.0f},{py:.0f})，带偏置移动……")
    goto_exact(robot, {**apply(base_target), "gripper": GRIPPER_OPEN}, duration=3.0)

    if jog(robot, base_target) == "quit":
        break
    smooth_goto(robot, observe, duration=3.0)

smooth_goto(robot, observe, duration=2.0)
shutdown(robot)
print("结束，最终偏置:", offsets)
