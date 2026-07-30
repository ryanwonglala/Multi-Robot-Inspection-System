"""触发式抓取测试：自动对位+伺服后暂停，合爪时机与紧度由你控制。

在终端运行:
    .venv/bin/python scripts/11_grasp_step.py

每轮: 放球 -> Enter -> 自动对位+伺服精修 -> 进入手控模式:
    c = 合爪(慢)      o = 张爪
    g = 紧度+2(更紧)  h = 紧度-2(更松)     当前紧度实时显示
    微调: 底座a/z 肩w/s 肘e/d 腕俯r/f
    Enter = 认为已夹稳 -> 提起+运输+投放+复位
    x = 放弃本轮(张爪回观察位)
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

from soarm.arm import (
    connect, goto_exact, load_pose, read_joints, shutdown, smooth_goto, transport_to_drop,
)
from soarm.camera_client import get_frame
from soarm.mapping import PixelToJoints
from soarm.vision import detect_blobs, servo_roi, workspace_roi

SERVO_FILE = Path(__file__).parent.parent / "config" / "servo.json"
GRIPPER_OPEN = 30.0
SERVO_MAX_AREA = 400_000
SERVO_TOL_PX = 12
SERVO_ITERS = 6
STEP = 0.8

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
}

grip = 5.0


SERVO_CFG = json.loads(SERVO_FILE.read_text()) if SERVO_FILE.exists() else None
HOVER = SERVO_CFG.get("hover_delta", 0.0) if SERVO_CFG else 0.0


def servo_refine(robot, command):
    if not SERVO_CFG:
        return
    servo = SERVO_CFG
    ref = np.array(servo["ref"]); Jm = np.array(servo["jacobian"]); sj = servo["joints"]
    for it in range(SERVO_ITERS):
        time.sleep(0.4)
        blobs = detect_blobs(get_frame(), seg="notwhite", max_area=SERVO_MAX_AREA, roi=servo_roi(ref))
        if not blobs:
            print("  伺服: 未见目标")
            return
        bb = min(blobs, key=lambda x: (x.cx - ref[0]) ** 2 + (x.cy - ref[1]) ** 2)
        err = np.array([bb.cx, bb.cy]) - ref
        print(f"  伺服#{it}: 偏差 ({err[0]:+.0f},{err[1]:+.0f})")
        if np.hypot(*err) <= SERVO_TOL_PX:
            return
        dq = np.clip(np.linalg.solve(Jm, -err) * 0.8, -3.0, 3.0)
        for j, d in zip(sj, dq):
            command[j] += float(d)
        smooth_goto(robot, command, duration=0.6)


def manual(robot, command) -> str:
    global grip
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print(f"手控: j=下潜 c合爪 o张爪 g紧+ h紧- p=存伺服基准 | 微调a/z w/s e/d r/f | Enter=继续 | x=放弃 (紧度{grip:.0f})")
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch in ("\n", "\r"):
                return "go"
            if ch == "x":
                return "abort"
            if ch == "j":
                command["shoulder_lift"] += HOVER  # 下潜到抓取高度
                smooth_goto(robot, {"shoulder_lift": command["shoulder_lift"]}, duration=1.2)
                print("\n已下潜")
            elif ch == "p":
                blobs = detect_blobs(get_frame(), max_area=SERVO_MAX_AREA)
                if not blobs and SERVO_FILE.exists():
                    print("\n未检测到球，基准未更新")
                    continue
                servo = json.loads(SERVO_FILE.read_text())
                old_ref = servo["ref"]
                bb = min(blobs, key=lambda x: (x.cx - old_ref[0]) ** 2 + (x.cy - old_ref[1]) ** 2)
                servo["ref"] = [round(bb.cx, 1), round(bb.cy, 1)]
                SERVO_FILE.write_text(json.dumps(servo, indent=2))
                print(f"\n伺服基准已更新: {old_ref} -> {servo['ref']}")
            elif ch == "c":
                smooth_goto(robot, {"gripper": grip}, duration=2.0)
                command["gripper"] = grip  # 微调时不泄夹压
            elif ch == "o":
                smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
                command["gripper"] = GRIPPER_OPEN
            elif ch == "g":
                grip = max(0.0, grip - 2.0)
            elif ch == "h":
                grip = min(GRIPPER_OPEN, grip + 2.0)
            elif ch in KEYMAP:
                joint, sign = KEYMAP[ch]
                command[joint] += sign * STEP
                robot.send_action({f"{j}.pos": v for j, v in command.items()})
                continue
            else:
                continue
            sys.stdout.write(f"\r当前紧度: {grip:.0f}    ")
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


mapping = PixelToJoints()
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

while True:
    cmd = input("\n放好球后按 Enter (x=结束) > ").strip().lower()
    if cmd == "x":
        break
    time.sleep(0.3)
    blobs = detect_blobs(get_frame(), roi=workspace_roi())
    if len(blobs) < 1:
        print("未检测到目标")
        continue
    b = blobs[0]
    target = mapping(b.cx, b.cy)
    print(f"目标({b.cx:.0f},{b.cy:.0f})，悬停对位+伺服……")
    aim = {**target, "shoulder_lift": target["shoulder_lift"] - HOVER, "gripper": GRIPPER_OPEN}
    goto_exact(robot, aim, duration=3.0)
    command = read_joints(robot)
    command["gripper"] = GRIPPER_OPEN
    servo_refine(robot, command)

    act = manual(robot, command)
    if act == "go":
        transport_to_drop(robot)
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
        print("已投放")
    else:
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
        print("已放弃")
    smooth_goto(robot, observe, duration=3.5)

shutdown(robot)
print(f"结束（最终紧度 {grip:.0f}，如果找到了稳定值告诉我写进默认）")
