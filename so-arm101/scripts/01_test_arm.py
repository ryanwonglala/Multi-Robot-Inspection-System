"""冒烟测试：扫描舵机总线，确认 6 个舵机在线并读取当前位置。

不涉及标定、不会使能扭矩，只做只读操作，可放心运行。
用法: .venv/bin/python scripts/01_test_arm.py
"""

import os
import sys

# 串口: macOS=/dev/cu.usbmodem*, Jetson/Linux=/dev/ttyACM*; 用 SOARM_PORT 覆盖
_DEFAULT_PORT = "/dev/ttyACM0" if sys.platform.startswith("linux") else "/dev/cu.usbmodem5AE70447161"
PORT = os.environ.get("SOARM_PORT", _DEFAULT_PORT)

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# SO-ARM101 follower: 6 个 STS3215，ID 1-6
MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

motors = {
    name: Motor(id=i + 1, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100)
    for i, name in enumerate(MOTOR_NAMES)
}

bus = FeetechMotorsBus(port=PORT, motors=motors)
bus.connect(handshake=False)
print(f"串口已打开: {PORT}")

found = bus.broadcast_ping()
print(f"总线扫描结果 (id: model_number): {found}")

missing = [m.id for m in motors.values() if m.id not in (found or {})]
if missing:
    print(f"警告: 未响应的舵机 ID: {missing}")
else:
    print("6 个舵机全部在线 ✓")
    positions = {name: bus.read("Present_Position", name, normalize=False) for name in motors}
    print("当前原始位置 (0-4095):")
    for name, pos in positions.items():
        print(f"  {name:15s} {pos}")

bus.disconnect()
print("测试完成，串口已关闭")
