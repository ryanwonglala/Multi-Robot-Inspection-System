"""通电验证式重采样：预测对位 -> 逐关节微调到完美 -> 确认即存为新样本。

在终端运行:
    .venv/bin/python scripts/08_refine_samples.py

每轮: 放球 -> Enter -> 臂按旧映射+跟随偏置对位 -> 微调:
    底座 a/z | 肩 w/s | 肘 e/d | 腕俯仰 r/f | 腕旋转 t/g   (上排加/下排减)
    Enter = 对准无误 -> 存入 calibration/samples_v2.json -> 回观察位
    x = 结束
跟随偏置 = 上一位置的修正量，自动带到下一位置，越调越省力。
目标: 9-12 个铺满工作区的验证样本。
"""

import json
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, goto_exact, load_pose, read_joints, shutdown, smooth_goto
from soarm.camera_client import get_frame
from soarm.mapping import MAP_JOINTS, PixelToJoints
from soarm.vision import detect_blobs, workspace_roi

CALIB = Path(__file__).parent.parent / "calibration"
V2_FILE = CALIB / "samples_v2.json"
GRIPPER_OPEN = 30.0
STEP = 0.8
AIM_LIFT = 8.0  # 初始对位抬高量(肩减小=抬)。换了更高的工作面(盆底)时防止按旧高度怼进底面

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
}

samples_v2 = json.loads(V2_FILE.read_text()) if V2_FILE.exists() else []
offsets = {j: 0.0 for j in MAP_JOINTS}  # 跟随偏置(仅会话内, 不落盘)


def jog(robot, command) -> str:
    """微调 command(原地修改)，返回 'ok' 或 'quit'。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("微调: 底座a/z 肩w/s 肘e/d 腕俯r/f 腕旋t/g | Enter=对准存样 | x=结束")
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch in ("\n", "\r"):
                return "ok"
            if ch == "x":
                return "quit"
            if ch not in KEYMAP:
                continue
            joint, sign = KEYMAP[ch]
            command[joint] += sign * STEP
            robot.send_action(
                {**{f"{j}.pos": v for j, v in command.items()}, "gripper.pos": GRIPPER_OPEN}
            )
            sys.stdout.write(
                "\r指令: " + " ".join(f"{j.split('_')[0][:4]}{command[j]:+.1f}" for j in MAP_JOINTS) + "   "
            )
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


# 引导映射: 优先当前 v2, 其次归档的桌面样本, 最后 v1 —— 只用于初始对位, 新样本写入 V2_FILE
for _f in (V2_FILE, CALIB / "samples_v2_desk.json", CALIB / "samples.json"):
    if _f.exists():
        mapping = PixelToJoints(_f)
        print(f"引导映射: {_f.name}")
        break
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)
print(f"已有验证样本 {len(samples_v2)} 条")

while True:
    cmd = input(f"\n[验证样本{len(samples_v2)}] 放好球后按 Enter (x=结束) > ").strip().lower()
    if cmd == "x":
        break
    time.sleep(0.3)
    blobs = detect_blobs(get_frame(), roi=workspace_roi())
    if len(blobs) != 1:
        print(f"检测到 {len(blobs)} 个目标，请只放一颗球")
        continue
    px, py = blobs[0].cx, blobs[0].cy
    base = mapping(px, py)
    command = {j: base[j] + offsets[j] for j in MAP_JOINTS}
    if all(v == 0.0 for v in offsets.values()):
        command["shoulder_lift"] -= AIM_LIFT  # 首个样本抬高对位, 由你手动降到位
    print(f"像素({px:.0f},{py:.0f})，对位中……")
    goto_exact(robot, {**command, "gripper": GRIPPER_OPEN}, duration=3.0)

    if jog(robot, command) == "quit":
        break

    offsets = {j: command[j] - base[j] for j in MAP_JOINTS}  # 修正量带到下一轮
    samples_v2.append({"pixel": [round(px, 1), round(py, 1)],
                       "joints": {j: round(command[j], 2) for j in MAP_JOINTS}})
    V2_FILE.write_text(json.dumps(samples_v2, indent=2, ensure_ascii=False))
    print(f"已存验证样本 #{len(samples_v2)-1}，回观察位……")
    smooth_goto(robot, observe, duration=3.0)

shutdown(robot)
print(f"结束，共 {len(samples_v2)} 条验证样本 -> {V2_FILE.name}")
