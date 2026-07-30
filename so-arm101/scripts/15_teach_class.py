"""按类示教抓取参数：ref(抓取点像素) + angle_ref(朝向基准) + depth_delta(深度差)。

用法: 白纸上只放一件该类物品, 在终端运行
    .venv/bin/python scripts/15_teach_class.py

流程: 观察位检测+自动分类 -> 臂按映射对位(带全局深度偏置, 偏高8°起步)
  -> 你用按键调到完美抓取位(底座a/z 肩w降/s升 肘e/d 腕俯r/f), Enter 确认
  -> 抬到悬停高度记录该类的基准像素位与朝向 -> 写入 config/target.json classes。
几何量(雅可比/腕旋增益)不在此标, 那是 10 号的事, 全类共用。
"""

import json
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import numpy as np

from soarm.arm import connect, goto_exact, load_pose, read_joints, shutdown, smooth_goto
from soarm.camera_client import get_frame
from soarm.mapping import PixelToJoints
from soarm.vision import classify_blob, detect_blobs, servo_roi, workspace_roi

TARGET_FILE = Path(__file__).parent.parent / "config" / "target.json"
SERVO_FILE = Path(__file__).parent.parent / "config" / "servo.json"
GRIPPER_OPEN = 30.0
STEP = 0.8
SERVO_MAX_AREA = 400_000

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
}


def jog(robot, command, base_sl: float) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("调到完美抓取位: 底座a/z 肩w降/s升 肘e/d 腕俯r/f 腕旋t/g | Enter=确认 | x=放弃")

    def show():
        d = command["shoulder_lift"] - base_sl
        sys.stdout.write(f"\r深度: 相对带偏置预测 {d:+.1f}°（正=更深）    ")
        sys.stdout.flush()

    show()
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
            show()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


target_cfg = json.loads(TARGET_FILE.read_text())
servo = json.loads(SERVO_FILE.read_text())
mapping = PixelToJoints()
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

input("\n白纸上只放一件要示教的物品，按 Enter 开始 > ")
time.sleep(0.3)
frame = get_frame()
blobs = detect_blobs(frame, roi=workspace_roi())
tagged = [(classify_blob(frame, b), b) for b in blobs]
tagged = [(n, b) for n, b in tagged if n is not None]
if len(tagged) != 1:
    print(f"需要恰好 1 件已定义类物品，当前识别到 {len(tagged)} 件: {[n for n, _ in tagged]}")
    shutdown(robot)
    sys.exit(1)
cls_name, b = tagged[0]
print(f"识别为 [{cls_name}]，对位……")

target = mapping(b.cx, b.cy)
# 偏置只在旧库引导期叠加(v3 新库自带实测深度, 见 09 同款逻辑)
from soarm.mapping import SAMPLES_FILE  # noqa: E402
bias = servo.get("depth_bias", 0.0) if SAMPLES_FILE.name != "samples_v3.json" else 0.0
base_sl = target["shoulder_lift"] + bias  # 带全局偏置的预测
command = dict(target)
command["shoulder_lift"] = base_sl - 16.0  # 大幅偏高起步(偏置链有逐点残差, 曾-11仍触桌), 手动降到位
goto_exact(robot, {**command, "gripper": GRIPPER_OPEN}, duration=3.0)

if jog(robot, command, base_sl) != "ok":
    shutdown(robot)
    sys.exit(0)

depth_delta = round(command["shoulder_lift"] - base_sl, 2)
print(f"类深度差(示教-带偏置预测): {depth_delta:+.1f}°")
grasp_joints = {j: round(command[j], 2) for j in
                ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")}

# 抬到悬停高度, 记录该类的基准像素位与朝向
hover = servo.get("hover_delta", 8.0)
command["shoulder_lift"] -= hover
smooth_goto(robot, {"shoulder_lift": command["shoulder_lift"]}, duration=1.0)
time.sleep(0.6)
seg_kw = ({"hsv_lo": tuple(target_cfg["classes"][cls_name]["servo_hsv"][0]),
           "hsv_hi": tuple(target_cfg["classes"][cls_name]["servo_hsv"][1])}
          if target_cfg["classes"][cls_name].get("servo_hsv") else {"seg": "notwhite"})
blobs2 = detect_blobs(get_frame(), **seg_kw, max_area=SERVO_MAX_AREA,
                      roi=servo_roi((960, 540)))
if not blobs2:
    print("悬停高度腕相机看不到目标，示教失败")
    shutdown(robot)
    sys.exit(1)
bb = blobs2[0]
ref = [round(bb.cx, 1), round(bb.cy, 1)]
no_roll = target_cfg["classes"][cls_name].get("no_roll")
angle_ref = round(bb.angle, 1) if (bb.elongation >= 1.15 and not no_roll) else None
print(f"基准像素位 {ref}  面积{bb.area:.0f}  朝向基准 {angle_ref}"
      f"{'' if angle_ref is not None else '(近圆形, 不做朝向对齐)'}")

cls = target_cfg["classes"][cls_name]
cls["ref"] = ref
cls["angle_ref"] = angle_ref
cls["depth_delta"] = depth_delta
cls["grasp_joints"] = grasp_joints  # 完整示教姿态(垂直纪律的载体, 俯仰常数K由此而来)
TARGET_FILE.write_text(json.dumps(target_cfg, ensure_ascii=False, indent=2))
print(f"[{cls_name}] 参数已写入 target.json")

smooth_goto(robot, observe, duration=3.0)
shutdown(robot)
