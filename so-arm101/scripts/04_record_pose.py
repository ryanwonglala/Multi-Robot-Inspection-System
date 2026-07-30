"""记录一个命名位姿（触发式）：摆好臂后按 Enter 记录。

在终端里运行（需要键盘交互）:
    .venv/bin/python scripts/04_record_pose.py <位姿名>   (默认 observe)

按 Enter 记录当前姿势（可反复，覆盖上一次）；输入 q 回车保存退出。
记录结果写入 config/poses.json。
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from soarm.arm import JOINTS, connect, read_joints

POSES_FILE = Path(__file__).parent.parent / "config" / "poses.json"
name = sys.argv[1] if len(sys.argv) > 1 else "observe"

robot = connect()
robot.bus.disable_torque()
print("扭矩已释放，可以自由摆臂。")
print("摆好后按 Enter 记录（可反复记录覆盖）；输入 q 回车保存退出。\n")

candidate = None
while True:
    cmd = input("Enter=记录当前姿势, q=保存退出 > ").strip().lower()
    pos = read_joints(robot)
    if cmd == "q":
        break
    candidate = {j: round(pos[j], 2) for j in JOINTS}
    print("已记录:", candidate, "\n")

if candidate is None:
    print("未记录任何姿势，退出（poses.json 未改动）")
else:
    POSES_FILE.parent.mkdir(exist_ok=True)
    poses = json.loads(POSES_FILE.read_text()) if POSES_FILE.exists() else {}
    poses[name] = candidate
    POSES_FILE.write_text(json.dumps(poses, indent=2, ensure_ascii=False))
    print(f"已保存 '{name}': {candidate}")

robot.disconnect()
print("完成（扭矩释放状态，放稳臂再离开）")
