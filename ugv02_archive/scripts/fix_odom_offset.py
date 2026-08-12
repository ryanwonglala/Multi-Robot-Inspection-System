from pathlib import Path
import re

p = Path.home() / "ros2_ws/src/wave_rover_driver/wave_rover_driver/wave_rover_node.py"
s = p.read_text()

bak = p.with_name(p.name + ".before_odom_offset_fix")
if not bak.exists():
    bak.write_text(s)

if "import math" not in s:
    lines = s.splitlines()
    insert_at = 0
    for i, line in enumerate(lines):
        if line.startswith("import ") or line.startswith("from "):
            insert_at = i + 1
    lines.insert(insert_at, "import math")
    s = "\n".join(lines) + "\n"

if "self.odom_yaw_offset = math.radians" not in s:
    m = re.search(r"(\n\s*def __init__\(self[^\)]*\):\n\s*super\(\).__init__\([^\n]*\)\n)", s)
    if not m:
        m = re.search(r"(\n\s*def __init__\(self[^\)]*\):\n)", s)
    if not m:
        raise SystemExit("找不到 __init__，需要手动看文件")

    indent = "        "
    block = (
        f'{indent}self.declare_parameter("odom_yaw_offset_deg", -90.0)\n'
        f'{indent}self.odom_yaw_offset = math.radians(float(self.get_parameter("odom_yaw_offset_deg").value))\n'
    )
    s = s[:m.end()] + block + s[m.end():]

p.write_text(s)
print("fixed:", p)
print("backup:", bak)
