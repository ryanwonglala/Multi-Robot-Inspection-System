"""全自动清空托盘：一次启动，自动完成全部 检测->抓取->运输->投放，直到无目标。

用法(终端): SOARM_PORT=/dev/ttyACM0 .venv/bin/python auto_clear.py [选项]
  --step        每轮开始前等确认(首次跑强烈建议)
  --max N       最多处理几个(默认 10, 防跑飞)
  --retry N     单个目标失败重试次数(默认 2, 用完就跳过该位置)
  --hover D     悬停抬高度数(默认 15)
  --order       left(默认) / right / area  —— 抓取顺序
  --no-drop     只抓不投放(抓起验证后原地放回), 用于安全排练

与 aim_dryrun.py 用同一套映射/depth_delta/运输编排。运输后的返回走对称路径
(回过渡位->转体->观察位)，不用 09 那句直接 smooth_goto(observe)——新场地要横扫 64°。
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from soarm.arm import (
    connect, goto_exact, grip_close, grip_load, load_pose, read_joints, shutdown,
    smooth_goto, transport_to_drop,
)
from soarm.camera_client import get_frame
from soarm.mapping import MAP_JOINTS, PixelToJoints
from soarm.vision import classify_blob, detect_blobs, workspace_roi

SHORT = {"shoulder_pan": "底座", "shoulder_lift": "肩", "elbow_flex": "肘",
         "wrist_flex": "腕俯", "wrist_roll": "腕旋"}
GRIPPER_OPEN = 30.0
LIFT = 8.0              # 抓起后的提离高度
SAME_SPOT = 120.0       # 像素距离小于此值视为同一个目标(用于跳过已放弃的位置)

ap = argparse.ArgumentParser()
ap.add_argument("--step", action="store_true")
ap.add_argument("--max", type=int, default=10)
ap.add_argument("--retry", type=int, default=2)
ap.add_argument("--hover", type=float, default=15.0)
ap.add_argument("--order", choices=["left", "right", "area"], default="left")
ap.add_argument("--no-drop", action="store_true")
args = ap.parse_args()

CLASSES = json.loads((ROOT / "config" / "target.json").read_text()).get("classes", {})
V3 = ROOT / "calibration" / "samples_v3.json"
LOG = ROOT / "calibration" / "attempts_log.jsonl"

samples = json.loads(V3.read_text())
mapping = PixelToJoints(V3)
J = np.array([[s["joints"][j] for j in MAP_JOINTS] for s in samples])
J_MIN, J_MAX = J.min(axis=0) - 12.0, J.max(axis=0) + 12.0

observe = load_pose("observe")
transit = load_pose("transit")
roi = workspace_roi()


def log(pixel, cls, result, opening=None, load=None):
    rec = {"pixel": [round(pixel[0], 1), round(pixel[1], 1)], "cls": cls,
           "result": result, "mode": "auto", "t": time.time()}
    if opening is not None:
        rec["opening"] = round(opening, 2)
    if load is not None:
        rec["load"] = load
    with open(LOG, "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def scan():
    """回观察位后重新检测。返回 [(blob, 类名)]，只保留已定义类。"""
    time.sleep(0.4)
    frame = get_frame()
    out = []
    for b in detect_blobs(frame, roi=roi):
        c = classify_blob(frame, b)
        if c:
            out.append((b, c))
    return out


def back_to_observe(robot, via_transit: bool):
    """回观察位。via_transit=True 用于从投放位返回(需先收高再转体)。"""
    if via_transit:
        smooth_goto(robot, dict(transit), duration=2.0)
        smooth_goto(robot, {"shoulder_pan": observe["shoulder_pan"]}, duration=2.5)
    smooth_goto(robot, {k: v for k, v in observe.items() if k != "gripper"}, duration=2.5)
    smooth_goto(robot, {"gripper": observe["gripper"]}, duration=0.6)


print(f"样本 {len(samples)} 条，模型 {mapping.model}，ROI {roi}")
print(f"顺序={args.order} 上限={args.max} 重试={args.retry} 悬停={args.hover}° "
      f"{'[只抓不投放]' if args.no_drop else ''}")

robot = connect()
smooth_goto(robot, observe, duration=3.0)
print("已在观察位。\n")

done = 0            # 成功投放数
skipped = []        # 放弃的目标像素
stall = 0           # 投放后检出数没减少的次数
prev_n = None

try:
    while done < args.max:
        found = scan()
        todo = [(b, c) for b, c in found
                if all((b.cx - p[0]) ** 2 + (b.cy - p[1]) ** 2 > SAME_SPOT ** 2 for p in skipped)]
        print(f"[第{done+1}轮] 检出 {len(found)} 个"
              + (f"（其中 {len(found)-len(todo)} 个已放弃，跳过）" if len(found) != len(todo) else ""))
        for b, c in todo:
            print(f"    ({b.cx:7.1f},{b.cy:6.1f}) 面积={b.area:6.0f} {c}")

        if not todo:
            print("\n✓ 工作区已清空，任务完成")
            break

        # 卡死保护: 上一轮投放成功但检出数没减少
        if prev_n is not None and len(todo) >= prev_n:
            stall += 1
            print(f"  ! 检出数未减少（{prev_n} -> {len(todo)}），第 {stall} 次")
            if stall >= 2:
                print("✗ 连续两轮无进展，中止（方块可能没被真正带走）")
                break
        else:
            stall = 0

        if args.step:
            if input("  回车=开始本轮, q=结束 > ").strip().lower() == "q":
                break

        key = {"left": lambda t: t[0].cx, "right": lambda t: -t[0].cx,
               "area": lambda t: -t[0].area}[args.order]
        b, cls = sorted(todo, key=key)[0]
        prev_n = len(todo)

        pred = mapping(b.cx, b.cy)
        bad = [j for k, j in enumerate(MAP_JOINTS) if not (J_MIN[k] <= pred[j] <= J_MAX[k])]
        if bad:
            print(f"  ✗ 预测超出示教包络 {bad}，放弃该目标")
            log((b.cx, b.cy), cls, "out_of_envelope")
            skipped.append((b.cx, b.cy))
            prev_n = None
            continue

        dd = (CLASSES.get(cls, {}) or {}).get("depth_delta", 0.0)
        hold = (CLASSES.get(cls, {}) or {}).get("hold", [4.0, 16.0])
        grasp_sl = pred["shoulder_lift"] + dd
        print(f"  目标 ({b.cx:.0f},{b.cy:.0f}) {cls}  抓取深度 肩={grasp_sl:.2f}")

        ok = False
        for attempt in range(args.retry + 1):
            if attempt:
                print(f"  重试 {attempt}/{args.retry}")
            aim = {**pred, "shoulder_lift": grasp_sl - args.hover, "gripper": GRIPPER_OPEN}
            goto_exact(robot, aim, duration=3.0)
            time.sleep(0.3)
            smooth_goto(robot, {"shoulder_lift": grasp_sl}, duration=1.2)
            time.sleep(0.4)
            held, opening, load = grip_close(
                robot, hold_min_opening=hold[0], hold_max_opening=hold[1])
            before = read_joints(robot)["gripper"]
            smooth_goto(robot, {"shoulder_lift": grasp_sl - LIFT}, duration=1.2)
            time.sleep(0.6)
            after = read_joints(robot)["gripper"]
            ok = after > hold[0] * 0.6
            print(f"    合爪 开合度={opening:.2f} 负载={load} | 提起后 {before:.2f}->{after:.2f}"
                  f"  {'✓ 夹住' if ok else '✗ 空夹'}")
            if ok:
                break
            smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.6)
            back_to_observe(robot, via_transit=False)

        if not ok:
            print(f"  ✗ 重试 {args.retry} 次仍失败，放弃该位置")
            log((b.cx, b.cy), cls, "empty", opening, load)
            skipped.append((b.cx, b.cy))
            prev_n = None
            continue

        if args.no_drop:
            print("  [--no-drop] 原地放回")
            smooth_goto(robot, {"shoulder_lift": grasp_sl}, duration=1.0)
            smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.6)
            back_to_observe(robot, via_transit=False)
            log((b.cx, b.cy), cls, "success_nodrop", opening, load)
            skipped.append((b.cx, b.cy))   # 没搬走, 标记避免重复抓同一个
            done += 1
            prev_n = None
            continue

        print("  运输投放……")
        transport_to_drop(robot)                                    # 全程不碰夹爪
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
        time.sleep(0.3)
        back_to_observe(robot, via_transit=True)
        done += 1
        log((b.cx, b.cy), cls, "success", opening, load)
        print(f"  ✓ 第 {done} 个投放完成\n")

except KeyboardInterrupt:
    print("\n\n! 已中断。臂将收到过渡位再回 home（若手上夹着方块会在 home 处落下）")
    try:
        smooth_goto(robot, dict(transit), duration=2.0)
    except Exception:
        pass

print(f"\n===== 汇总 =====")
print(f"成功投放 {done} 个，放弃 {len(skipped)} 个")
for p in skipped:
    print(f"  放弃: 像素({p[0]:.0f},{p[1]:.0f})")
print("回 home 断电……")
shutdown(robot)
print("完成")
