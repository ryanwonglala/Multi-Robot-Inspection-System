"""示教网格：人工教出工作区内的干净样本库（深度的唯一事实来源）。

用法(终端):
    .venv/bin/python scripts/18_teach_grid.py

每轮: 把目标物放到绿框内一个新位置 -> Enter -> 臂对位(高位安全起步)
  -> 按键微调到完美垂直抓取位(底座a/z 肩w降/s升 肘e/d 腕俯r/f)
  -> Enter 确认 -> (像素,关节) 直接存入 samples_v3.json -> 下一轮
q 结束。建议 6 个点: 绿框四角靠内 + 中心 + 任一处。全程夹爪垂直纪律。
样本是"人工教的完美抓取", 无噪声无偏置; 执行时深度=平面拟合(你教的深度)。
"""

import argparse
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

_ap = argparse.ArgumentParser()
_ap.add_argument("--lift", type=float, default=None,
                 help="起步安全抬高(度), 覆盖自动值。教示教库覆盖不到的区域(外推)时建议 12~16")
_args = _ap.parse_args()

KEYMAP = {
    "a": ("shoulder_pan", +1), "z": ("shoulder_pan", -1),
    "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", +1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", +1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", +1), "g": ("wrist_roll", -1),
}


def test_grip(robot, hold) -> float:
    """试夹一次并报告开合度, 用于存样本前验证这个深度真能兜住物体。

    只夹不提: grip_close 接触即停, 不持续堵转。返回接触开合度。
    加这个键的由来: 首轮示教有 4 个点"看着对准了"但深度不足, 执行时只咬到方块边缘,
    直到抓取阶段才暴露。存样本前先夹一次, 当场就能发现。
    """
    from soarm.arm import grip_close
    held, opening, load = grip_close(robot, hold_min_opening=hold[0], hold_max_opening=hold[1])
    print(f"\n  试夹: 开合度={opening:.2f} 负载={load} 判定={'夹住' if held else '未夹住'}"
          f" (区间{hold})  ——夹不实就按 o 张爪、继续降深度")
    return opening


def jog(robot, command, k_const: float | None = None, hold=None) -> str:
    """按键微调。k_const 给定时锁定夹爪俯仰: 动肩/肘自动补偿腕俯仰保持 K 恒定。

    执行时 09 的 vertical_wf 会把腕俯仰强制算成 K-肩-肘, 样本里的腕俯仰根本不被使用。
    所以各点 K 不一致 = 你在俯仰角 A 下教的深度, 执行时被掰成俯仰角 B, 深度全废。
    这里把该纪律做进工具, 不靠人脑保持。r/f 仍可直接改腕俯仰(即改 K), 用于调进刀角。
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    lock = "锁定" if k_const is not None else "自由"
    grip_cmd = GRIPPER_OPEN
    print(f"微调到完美抓取位[俯仰{lock}]: 底座a/z 肩w降/s升 肘e/d 腕俯r/f(改K) 腕旋t/g")
    print("  c=试夹一次(验证深度够不够)  o=张爪  |  Enter=存样本  x=跳过")
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
            if ch == "c" and hold is not None:
                grip_cmd = test_grip(robot, hold)  # 记住实际开合度, 免得下次微调又把爪张开
                continue
            if ch == "o":
                grip_cmd = GRIPPER_OPEN
                robot.send_action({"gripper.pos": grip_cmd})
                print("\n  已张爪")
                continue
            if ch not in KEYMAP:
                continue
            joint, sign = KEYMAP[ch]
            command[joint] += sign * STEP
            if k_const is not None and joint in ("shoulder_lift", "elbow_flex"):
                command["wrist_flex"] = k_const - command["shoulder_lift"] - command["elbow_flex"]
            robot.send_action(
                {**{f"{j}.pos": v for j, v in command.items()}, "gripper.pos": grip_cmd}
            )
            k_now = command["shoulder_lift"] + command["elbow_flex"] + command["wrist_flex"]
            dev = "" if k_const is None else f" 基准{k_const:6.2f} 差{k_now - k_const:+5.2f}"
            print(
                f"\r  肩{command['shoulder_lift']:7.2f} 肘{command['elbow_flex']:7.2f}"
                f" 腕俯{command['wrist_flex']:7.2f} 腕旋{command['wrist_roll']:6.2f}"
                f" | K={k_now:6.2f}{dev}   ",
                end="", flush=True,
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
    cn = classify_blob(frame, b)
    print(f"目标[{cn or '?'}] 像素({b.cx:.0f},{b.cy:.0f})")
    classes = json.loads(
        (Path(__file__).parent.parent / "config" / "target.json").read_text()
    ).get("classes", {})
    gj = (classes.get(cn, {}) or {}).get("grasp_joints") or next(
        (c["grasp_joints"] for c in classes.values() if c.get("grasp_joints")), None)
    # 俯仰锁定基准 K, 与 09_grasp 的 vertical_wf 同源(该类示教姿态的 肩+肘+腕俯)。
    # lock_pitch=false 的类不锁: 执行时腕俯仰也由映射预测, 示教就该自由调并如实入库。
    if (classes.get(cn, {}) or {}).get("lock_pitch") is False:
        k_const = None
    else:
        k_const = (gj["shoulder_lift"] + gj["elbow_flex"] + gj["wrist_flex"]) if gj else None
    try:
        mapping = PixelToJoints(V3_FILE) if len(samples) >= 3 else None
    except Exception:
        mapping = None
    if mapping and len(samples) >= 3:
        target = mapping(b.cx, b.cy)
        # 映射预测的腕俯仰不可信: 执行时 vertical_wf 会覆盖它, 起点就按 K 算才与执行一致
        if k_const is not None:
            target["wrist_flex"] = k_const - target["shoulder_lift"] - target["elbow_flex"]
    elif gj:
        # 库太小: 用该类(或任一类)已示教的抓取姿态当起点, 省大量手动按键
        target = dict(gj)
    else:
        from soarm.arm import read_joints
        target = read_joints(robot)
        target.pop("gripper", None)
    command = dict(target)
    auto_lift = SAFE_LIFT_WARM if (mapping and len(samples) >= 3) else SAFE_LIFT_COLD
    lift = _args.lift if _args.lift is not None else auto_lift
    command["shoulder_lift"] -= lift
    print(f"  起步安全抬高 {lift:.1f}°" + ("" if _args.lift is None else " (命令行指定)"))
    goto_exact(robot, {**command, "gripper": GRIPPER_OPEN}, duration=3.0)

    hold = (classes.get(cn, {}) or {}).get("hold", [4.0, 16.0])
    verdict = jog(robot, command, k_const, hold)
    # 无论存不存, 离开抓取位前一定先张爪: 试夹后爪是闭的, 直接抬起会把物体带走,
    # 且 observe 位姿自带 gripper 值, 跟着位姿下发会在回程途中重新闭合。
    smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.5)
    if verdict != "ok":
        smooth_goto(robot, {k: v for k, v in observe.items() if k != "gripper"}, duration=3.0)
        smooth_goto(robot, {"gripper": observe["gripper"]}, duration=0.6)
        continue

    samples.append({"pixel": [round(b.cx, 1), round(b.cy, 1)],
                    "joints": {j: round(command[j], 2) for j in MAP_JOINTS}})
    V3_FILE.write_text(json.dumps(samples, indent=2, ensure_ascii=False))
    print(f"✓ 已存 ({len(samples)} 条): 像素({b.cx:.0f},{b.cy:.0f}) 肩{command['shoulder_lift']:.1f}")

    # 抬起回观察位, 进入下一轮
    smooth_goto(robot, {"shoulder_lift": command["shoulder_lift"] - 20.0}, duration=1.2)
    smooth_goto(robot, {k: v for k, v in observe.items() if k != "gripper"}, duration=3.0)
    smooth_goto(robot, {"gripper": observe["gripper"]}, duration=0.6)

shutdown(robot)
print(f"示教结束, 样本库共 {len(samples)} 条")
