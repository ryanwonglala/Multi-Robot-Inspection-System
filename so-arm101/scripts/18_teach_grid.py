"""示教网格：人工教出工作区内的干净样本库（深度的唯一事实来源）。

用法(终端):
    .venv/bin/python scripts/18_teach_grid.py

每轮: 把目标物放到绿框内一个新位置 -> Enter -> 臂对位(高位安全起步)
  -> 按键微调到完美垂直抓取位(底座a/z 肩w降/s升 肘e/d 腕俯r/f)
  -> Enter 确认 -> (像素,关节) 直接存入 samples_v3.json -> 下一轮
q 结束。建议 6 个点: 绿框四角靠内 + 中心 + 任一处。全程夹爪垂直纪律。
样本是"人工教的完美抓取", 无噪声无偏置; 执行时深度=平面拟合(你教的深度)。
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
from soarm.vision import classify_blob, detect_blobs, workspace_roi

CALIB = Path(__file__).parent.parent / "calibration"
V3_FILE = CALIB / "samples_v3.json"
GRIPPER_OPEN = 30.0
STEP = 0.8
SAFE_LIFT_COLD = 16.0  # 模板起步时的安全抬高(库空, 对位不可信)
SAFE_LIFT_WARM = 4.0   # 映射接管后的安全抬高(库≥3条, 对位已可信, 少按几下w)

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
}


def jog(robot, command) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("微调到完美垂直抓取位: 底座a/z 肩w降/s升 肘e/d 腕俯r/f 腕旋t/g | Enter=存样本 | x=跳过")
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                continue
            ch = sys.stdin.read(1).lower()
            if ch in ("\n", "\r"):
                return "ok"
            if ch == "x":
                return "skip"
            if ch not in KEYMAP:
                continue
            joint, sign = KEYMAP[ch]
            command[joint] += sign * STEP
            robot.send_action(
                {**{f"{j}.pos": v for j, v in command.items()}, "gripper.pos": GRIPPER_OPEN}
            )
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


samples = json.loads(V3_FILE.read_text()) if V3_FILE.exists() else []
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)
print(f"样本库现有 {len(samples)} 条")

while True:
    cmd = input("\n把目标放到绿框内新位置, Enter=开始本轮 (q=结束) > ").strip().lower()
    if cmd == "q":
        break
    time.sleep(0.4)
    frame = get_frame()
    blobs = detect_blobs(frame, roi=workspace_roi())
    if not blobs:
        print("未检测到目标")
        continue
    b = blobs[0]
    print(f"目标[{classify_blob(frame, b) or '?'}] 像素({b.cx:.0f},{b.cy:.0f})")
    try:
        mapping = PixelToJoints(V3_FILE) if len(samples) >= 3 else None
    except Exception:
        mapping = None
    if mapping and len(samples) >= 3:
        target = mapping(b.cx, b.cy)
    else:
        # 库太小: 用该类(或任一类)已示教的抓取姿态当起点, 省大量手动按键
        classes = json.loads((Path(__file__).parent.parent / "config" / "target.json").read_text()).get("classes", {})
        cn = classify_blob(frame, b)
        gj = (classes.get(cn, {}) or {}).get("grasp_joints") or next(
            (c["grasp_joints"] for c in classes.values() if c.get("grasp_joints")), None)
        if gj:
            target = dict(gj)
        else:
            from soarm.arm import read_joints
            target = read_joints(robot)
            target.pop("gripper", None)
    command = dict(target)
    command["shoulder_lift"] -= SAFE_LIFT_WARM if (mapping and len(samples) >= 3) else SAFE_LIFT_COLD
    goto_exact(robot, {**command, "gripper": GRIPPER_OPEN}, duration=3.0)

    if jog(robot, command) != "ok":
        smooth_goto(robot, observe, duration=3.0)
        continue

    samples.append({"pixel": [round(b.cx, 1), round(b.cy, 1)],
                    "joints": {j: round(command[j], 2) for j in MAP_JOINTS}})
    V3_FILE.write_text(json.dumps(samples, indent=2, ensure_ascii=False))
    print(f"✓ 已存 ({len(samples)} 条): 像素({b.cx:.0f},{b.cy:.0f}) 肩{command['shoulder_lift']:.1f}")

    # 抬起回观察位, 进入下一轮
    smooth_goto(robot, {"shoulder_lift": command["shoulder_lift"] - 20.0}, duration=1.2)
    smooth_goto(robot, observe, duration=3.0)

shutdown(robot)
print(f"示教结束, 样本库共 {len(samples)} 条")
