"""首次运动测试：腕旋转小幅摆动 + 夹爪开合，全程平滑低速。

不移动大臂关节，碰撞风险极低。
用法: .venv/bin/python scripts/03_test_motion.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import connect, read_joints, smooth_goto

robot = connect()
start = read_joints(robot)
print("起始位姿:", {k: round(v, 1) for k, v in start.items()})

print("腕旋转 +10° ...")
smooth_goto(robot, {"wrist_roll": start["wrist_roll"] + 10}, duration=1.5)
print("腕旋转 -10° ...")
smooth_goto(robot, {"wrist_roll": start["wrist_roll"] - 10}, duration=1.5)
print("腕旋转回位 ...")
smooth_goto(robot, {"wrist_roll": start["wrist_roll"]}, duration=1.0)

print("夹爪张开 ...")
smooth_goto(robot, {"gripper": 60}, duration=1.5)
print("夹爪闭合回位 ...")
smooth_goto(robot, {"gripper": start["gripper"]}, duration=1.5)

end = read_joints(robot)
drift = {k: round(end[k] - start[k], 2) for k in start}
print("结束位姿与起始的偏差:", drift)

robot.disconnect()
print("测试完成，扭矩已释放")
