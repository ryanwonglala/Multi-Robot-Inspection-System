"""手动抓取投放驾驶舱（当前架构版, 取代过时的 11 号）。

在终端运行:
    .venv/bin/python scripts/19_manual_grasp.py

每轮流程:
  把物体放绿框内 -> Enter -> 自动: 检测+分类 -> 映射对位到悬停高度(垂直姿态)
  -> 伺服精修 -> 进入手控模式:
     微调: 底座a/z  肩w降/s升  肘e/d  腕俯r/f  腕旋t/g   (每按0.8°)
     j = 下潜一档(肩+2°, 腕俯自动补偿保持垂直)   u = 上升一档
     c = 接触即停合爪(用该类实测宽度区间判定)     o = 张爪
     Enter = 认可当前夹持 -> 运输 -> 罐口投放 -> 回观察位
     x = 放弃本轮(张爪回观察位)
  主提示符下 q = 结束(回 home 断电)。
深度参考: 手控中实时显示肩角相对映射预测的差值。
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
    connect, goto_exact, grip_close, load_pose, read_joints, shutdown, smooth_goto,
    transport_to_drop,
)
from soarm.camera_client import get_frame
from soarm.mapping import PixelToJoints
from soarm.vision import classify_blob, detect_blobs, servo_roi, workspace_roi

SERVO = json.loads((Path(__file__).parent.parent / "config" / "servo.json").read_text())
CLASSES = json.loads((Path(__file__).parent.parent / "config" / "target.json").read_text()).get("classes", {})
GRIPPER_OPEN = 30.0
STEP = 0.8
DIVE = 2.0          # j/u 每档下潜/上升量
SERVO_MAX_AREA = 400_000
SERVO_TOL_PX = 12

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
}


def cls_seg_kwargs(cls) -> dict:
    if cls.get("servo_hsv"):
        lo, hi = cls["servo_hsv"]
        return {"hsv_lo": tuple(lo), "hsv_hi": tuple(hi)}
    return {"seg": "notwhite"}


def vertical_wf(cls, sl, el):
    gj = cls.get("grasp_joints")
    if not gj:
        return None
    K = gj["shoulder_lift"] + gj["elbow_flex"] + gj["wrist_flex"]
    return K - sl - el


def servo_refine(robot, command, cls, ref):
    Jm = np.array(SERVO["jacobian"])
    sj = SERVO["joints"]
    for it in range(8):
        time.sleep(0.4)
        blobs = detect_blobs(get_frame(), **cls_seg_kwargs(cls),
                             max_area=SERVO_MAX_AREA, roi=servo_roi(ref))
        if not blobs:
            print("  伺服: 未见目标(手动接管)")
            return
        bb = min(blobs, key=lambda x: (x.cx - ref[0]) ** 2 + (x.cy - ref[1]) ** 2)
        err = np.array([bb.cx, bb.cy]) - ref
        print(f"  伺服#{it}: 偏差 ({err[0]:+.0f},{err[1]:+.0f})")
        if np.hypot(*err) <= SERVO_TOL_PX:
            return
        dq = np.clip(np.linalg.solve(Jm, -err) * 0.8, -4.0, 4.0)
        for j, d in zip(sj, dq):
            command[j] += float(d)
        smooth_goto(robot, command, duration=0.6)


def manual(robot, command, cls, base_sl) -> str:
    hold = cls.get("hold", [4.0, 16.0])
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("手控: j下潜/u上升(带垂直补偿) c合爪 o张爪 | 微调a/z w/s e/d r/f t/g | Enter=运输投放 | x=放弃")

    def show(extra=""):
        d = command["shoulder_lift"] - base_sl
        sys.stdout.write(f"\r深度: 相对预测 {d:+.1f}°  {extra}      ")
        sys.stdout.flush()

    show()
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
            if ch in ("j", "u"):
                delta = DIVE if ch == "j" else -DIVE
                command["shoulder_lift"] += delta
                wf = vertical_wf(cls, command["shoulder_lift"], command["elbow_flex"])
                move = {"shoulder_lift": command["shoulder_lift"]}
                if wf is not None:
                    command["wrist_flex"] = wf
                    move["wrist_flex"] = wf
                smooth_goto(robot, move, duration=0.6)
                show()
            elif ch == "c":
                held, g, load = grip_close(robot, hold_min_opening=hold[0], hold_max_opening=hold[1])
                command["gripper"] = max(g - 1.2, 0.5)  # 与咬合目标一致, 微调时不泄压
                show(f"合爪: {'夹住' if held else '未夹住'} 开合{g:.1f} 负载{load}")
            elif ch == "o":
                smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
                command["gripper"] = GRIPPER_OPEN
                show("已张爪")
            elif ch in KEYMAP:
                joint, sign = KEYMAP[ch]
                command[joint] += sign * STEP
                robot.send_action({f"{j}.pos": v for j, v in command.items()})
                show()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


mapping = PixelToJoints()
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

while True:
    cmd = input("\n物体放绿框内后按 Enter (q=结束) > ").strip().lower()
    if cmd == "q":
        break
    time.sleep(0.4)
    frame = get_frame()
    blobs = detect_blobs(frame, roi=workspace_roi())
    if not blobs:
        print("绿框内未检测到目标")
        continue
    b = blobs[0]
    cls_name = classify_blob(frame, b)
    cls = CLASSES.get(cls_name, {}) if cls_name else {}
    print(f"目标[{cls_name or '未定义类'}] ({b.cx:.0f},{b.cy:.0f}) 面积{b.area:.0f}")

    target = mapping(b.cx, b.cy)
    base_sl = target["shoulder_lift"] + cls.get("depth_delta", 0.0)
    hover_sl = base_sl - SERVO.get("hover_delta", 8.0)
    wf = vertical_wf(cls, base_sl, target["elbow_flex"])
    aim = {**target, "shoulder_lift": hover_sl, "gripper": GRIPPER_OPEN}
    if wf is not None:
        aim["wrist_flex"] = wf
    if SERVO.get("roll_neutral") is not None:
        aim["wrist_roll"] = SERVO["roll_neutral"]
    goto_exact(robot, aim, duration=3.0)

    command = read_joints(robot)
    command["gripper"] = GRIPPER_OPEN
    ref = np.array(cls["ref"]) if cls.get("ref") else np.array(SERVO["ref"])
    servo_refine(robot, command, cls, ref)

    act = manual(robot, command, cls, base_sl)
    if act == "go":
        transport_to_drop(robot)
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
        print("已投放")
    else:
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
        print("已放弃")
    smooth_goto(robot, observe, duration=3.5)

shutdown(robot)
print("结束")
