"""视觉伺服标定：基准像素位 + 关节-像素雅可比。

在终端运行:
    .venv/bin/python scripts/10_servo_calib.py

流程:
  1. 把球放在工作区中部, Enter -> 臂按映射对位
  2. 用按键把两指调到完美跨住球(底座a/z 肩w/s 肘e/d 腕俯r/f), Enter 确认
  3. 程序自动: 记录球在腕相机中的基准像素位 -> 底座/肘各动±2°测像素响应
     -> 生成雅可比 -> 保存 config/servo.json
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
from soarm.vision import detect_blobs, servo_roi, workspace_roi

SERVO_FILE = Path(__file__).parent.parent / "config" / "servo.json"
GRIPPER_OPEN = 30.0
STEP = 0.8
SERVO_MAX_AREA = 400_000  # 腕相机贴近看球, 面积远大于观察位

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
}


def detect_near(ref=None):
    """检测球（放宽面积上限）；多目标时取离 ref 最近的。"""
    blobs = detect_blobs(get_frame(), seg="notwhite",
                         max_area=SERVO_MAX_AREA,
                         roi=servo_roi(ref if ref is not None else (960, 540)))
    if not blobs:
        return None
    if ref is None:
        return blobs[0]
    return min(blobs, key=lambda b: (b.cx - ref[0]) ** 2 + (b.cy - ref[1]) ** 2)


def jog(robot, command, target_sl: float | None = None) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("调到两指完美跨住目标: 底座a/z 肩w降/s升 肘e/d 腕俯r/f | Enter=确认 | x=放弃")

    def show_depth():
        if target_sl is not None:
            d = command["shoulder_lift"] - target_sl
            sys.stdout.write(f"\r深度: 相对映射预测 {d:+.1f}°（正=更深）    ")
            sys.stdout.flush()

    show_depth()
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch in ("\n", "\r"):
                # 防呆: 确认位若比映射预测还高, 大概率是忘了下潜(此处曾连坑两次)
                if target_sl is not None and command["shoulder_lift"] < target_sl:
                    d = command["shoulder_lift"] - target_sl
                    print(f"\n⚠ 当前比映射预测还高 {-d:.1f}°——指尖真的已跨在目标两侧了吗?")
                    print("  再按一次 Enter 强制确认, 按其他键继续调")
                    while True:
                        r2, _, _ = select.select([sys.stdin], [], [], 0.5)
                        if r2:
                            ch2 = sys.stdin.read(1)
                            break
                    if ch2 in ("\n", "\r"):
                        return "ok"
                    show_depth()
                    continue
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
            show_depth()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


mapping = PixelToJoints()
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

input("\n把球放在工作区中部，按 Enter 对位 > ")
time.sleep(0.3)
blobs = detect_blobs(get_frame(), roi=workspace_roi())
if len(blobs) != 1:
    print(f"观察位检测到 {len(blobs)} 个目标，请只放一颗球")
    shutdown(robot)
    sys.exit(1)
target = mapping(blobs[0].cx, blobs[0].cy)
command = dict(target)
command["shoulder_lift"] -= 8.0  # 对高起步(工作面可能比映射假设的高), 手动降到位
goto_exact(robot, {**command, "gripper": GRIPPER_OPEN}, duration=3.0)

if jog(robot, command, target_sl=target["shoulder_lift"]) != "ok":
    shutdown(robot)
    sys.exit(0)

# 深度偏置: 标定确认的抓取肩角 - 映射预测肩角 (自采首潜直接带偏置到位)
depth_bias = round(command["shoulder_lift"] - target["shoulder_lift"], 2)
print(f"深度偏置(标定-预测): {depth_bias:+.1f}°")

# 1) 抬升到悬停高度(指尖离球3-4cm), 在此记录基准像素位
#    伺服将在悬停高度进行(指尖够不着球, 微调不会推动球), 收敛后垂直下潜合爪
HOVER_DELTA = 8.0  # 肩减小=抬起
command["shoulder_lift"] -= HOVER_DELTA
smooth_goto(robot, {"shoulder_lift": command["shoulder_lift"]}, duration=1.0)
time.sleep(0.6)
b = detect_near()
if b is None:
    print("悬停高度腕相机看不到球，标定失败")
    shutdown(robot)
    sys.exit(1)
ref = [b.cx, b.cy]
print(f"悬停基准像素位: ({ref[0]:.0f}, {ref[1]:.0f})  面积{b.area:.0f}")

# 2) 雅可比: 底座/肘 各 ±2° 测像素响应(在悬停高度, 不会碰球)
J = []
for joint in ("shoulder_pan", "elbow_flex"):
    deltas = []
    for d in (+2.0, -2.0):
        test = {**command, joint: command[joint] + d}
        smooth_goto(robot, test, duration=0.8)
        time.sleep(0.5)
        bb = detect_near(ref)
        if bb is None:
            print(f"{joint} 扰动后丢失目标，标定失败")
            shutdown(robot)
            sys.exit(1)
        deltas.append([(bb.cx - ref[0]) / d, (bb.cy - ref[1]) / d])
        smooth_goto(robot, command, duration=0.8)
        time.sleep(0.3)
    col = np.mean(deltas, axis=0)
    J.append(col.tolist())
    print(f"{joint}: 1° -> 像素位移 ({col[0]:+.1f}, {col[1]:+.1f})")

# 3) 朝向自适应: 记录完美预备位的目标图像角度 + 腕旋转->图像角增益
b0 = detect_near(ref)
angle_ref = round(b0.angle, 1) if b0 else 0.0
roll_neutral = round(command["wrist_roll"], 2)
test = {**command, "wrist_roll": command["wrist_roll"] + 8.0}
smooth_goto(robot, test, duration=0.8); time.sleep(0.5)
b1 = detect_near(ref)
roll_gain = 0.0
if b1 is not None:
    da = (b1.angle - angle_ref + 90) % 180 - 90
    roll_gain = round(da / 8.0, 3)
smooth_goto(robot, command, duration=0.8); time.sleep(0.3)
print(f"朝向基准 {angle_ref:.0f}° 腕旋增益 {roll_gain:+.2f} (|增益|<0.3 视为不可用)")

# J 列向量为各关节的像素响应 -> 存 2x2 矩阵 [dpx/dq]
Jm = np.array(J).T  # 2x2: 行=像素xy, 列=关节
if abs(np.linalg.det(Jm)) < 1e-3:
    print("雅可比接近奇异，标定失败（两关节像素响应太相似）")
    shutdown(robot)
    sys.exit(1)

SERVO_FILE.write_text(json.dumps({
    "ref": [round(ref[0], 1), round(ref[1], 1)],
    "jacobian": [[round(v, 3) for v in row] for row in Jm.tolist()],
    "joints": ["shoulder_pan", "elbow_flex"],
    "hover_delta": HOVER_DELTA,
    "depth_bias": depth_bias,
    "angle_ref": angle_ref,
    "roll_gain": roll_gain if abs(roll_gain) >= 0.3 else 0.0,
    "roll_neutral": roll_neutral,
}, indent=2))
print(f"标定完成 -> {SERVO_FILE.name}")

smooth_goto(robot, observe, duration=3.0)
shutdown(robot)
