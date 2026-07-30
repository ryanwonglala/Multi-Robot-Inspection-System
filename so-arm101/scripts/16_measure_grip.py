"""夹取宽度实测标定：夹爪当卡尺, 量出每类物体的合法开合度区间。

用法(在终端跑, 每类一次):
    .venv/bin/python scripts/16_measure_grip.py --cls 积木

流程: 臂到观察位(悬空安全) -> 你把物品用手递到两指之间(按将来抓取的姿态,
多换几个方向量出最宽/最窄情形) -> Enter 合爪测量 -> 读数 -> Enter 张爪
-> 重复; q 结束后自动把 [最小-1.5, 最大+1.5] 写入该类的 hold 区间。
顺便把底板/托盘边也量一次(不写库), 确认它和物体区间分得开。
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, grip_close, load_pose, shutdown, smooth_goto

TARGET_FILE = Path(__file__).parent.parent / "config" / "target.json"
GRIPPER_OPEN = 30.0

parser = argparse.ArgumentParser()
parser.add_argument("--cls", default=None, help="要写入的类名(积木/笔袋/手串); 不给则只测不写")
args = parser.parse_args()

cfg = json.loads(TARGET_FILE.read_text())
if args.cls and args.cls not in cfg.get("classes", {}):
    print(f"未知类名 {args.cls}, 现有: {list(cfg.get('classes', {}))}")
    sys.exit(1)

robot = connect()
smooth_goto(robot, load_pose("observe"), duration=3.0)
smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=1.0)

vals = []
print("\n把物品递到两指之间(按抓取姿态), Enter=合爪测量, q=结束")
while True:
    cmd = input(f"[已测{len(vals)}次] > ").strip().lower()
    if cmd == "q":
        break
    held, opening, load = grip_close(robot, hold_min_opening=0.0, hold_max_opening=100.0)
    print(f"  开合度 {opening:.1f}  负载 {load}")
    vals.append(opening)
    input("  Enter=张爪取回 > ")
    smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
    time.sleep(0.2)

if vals:
    lo, hi = min(vals) - 1.5, max(vals) + 1.5
    print(f"\n实测 {len(vals)} 次: {[f'{v:.1f}' for v in vals]}")
    print(f"建议区间: [{lo:.1f}, {hi:.1f}]")
    if args.cls:
        cfg["classes"][args.cls]["hold"] = [round(max(lo, 1.0), 1), round(hi, 1)]
        TARGET_FILE.write_text(json.dumps(cfg, ensure_ascii=False, indent=2))
        print(f"已写入 [{args.cls}] 的 hold 区间")

shutdown(robot)
