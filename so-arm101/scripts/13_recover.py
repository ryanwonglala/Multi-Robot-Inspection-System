"""急停后的恢复：从冻结状态温和复位。

用法:
    .venv/bin/python scripts/13_recover.py          # 若夹着东西先在原地放下 -> 回观察位 -> 回home断电
    .venv/bin/python scripts/13_recover.py --free   # 仅释放扭矩(手动接管臂, 注意扶住)
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, grip_load, load_pose, read_joints, shutdown, smooth_goto

parser = argparse.ArgumentParser()
parser.add_argument("--free", action="store_true")
args = parser.parse_args()

robot = connect()  # 带防冲击: 目标先钉当前位置

if args.free:
    robot.bus.disable_torque()
    print("扭矩已释放(注意扶住臂), 退出")
    sys.exit(0)

cur = read_joints(robot)
held = grip_load(robot) >= 80
if held:
    print(f"检测到夹爪夹着东西(开合{cur['gripper']:.1f}), 原地缓慢放下……")
    smooth_goto(robot, {"shoulder_lift": cur["shoulder_lift"] + 6.0}, duration=1.5)  # 轻降
    smooth_goto(robot, {"gripper": 30.0}, duration=1.2)
    smooth_goto(robot, {"shoulder_lift": cur["shoulder_lift"] - 10.0}, duration=1.2)

print("回观察位……")
smooth_goto(robot, load_pose("observe"), duration=4.0)
time.sleep(0.3)
print("回 home 断电……")
shutdown(robot)
print("恢复完成")
