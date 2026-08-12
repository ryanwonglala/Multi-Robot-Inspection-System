from pathlib import Path
import re

p = Path.home() / "ros2_ws/src/wave_rover_driver/wave_rover_driver/wave_rover_node.py"
s = p.read_text()
bak = p.with_name(p.name + ".before_odom_yaw_offset")
if not bak.exists():
    bak.write_text(s)

if "import math" not in s:
    s = s.replace("import time", "import time\nimport math", 1)

# Add yaw offset parameter after /odom publisher
if "odom_yaw_offset_deg" not in s:
    markers = [
        'self.odom_pub = self.create_publisher(Odometry, "/odom", 10)',
        "self.odom_pub = self.create_publisher(Odometry, '/odom', 10)",
    ]
    for m in markers:
        if m in s:
            s = s.replace(
                m,
                m + '\n        self.declare_parameter("odom_yaw_offset_deg", -90.0)\n'
                    '        self.odom_yaw_offset = math.radians(float(self.get_parameter("odom_yaw_offset_deg").value))',
                1
            )
            break
else:
    s = re.sub(
        r'self\.declare_parameter\("odom_yaw_offset_deg",\s*[-0-9.]+\)',
        'self.declare_parameter("odom_yaw_offset_deg", -90.0)',
        s
    )

# Rotate odom position integration if it uses self.yaw directly
if "math.cos(self.yaw + self.odom_yaw_offset)" not in s:
    s = s.replace("math.cos(self.yaw)", "math.cos(self.yaw + self.odom_yaw_offset)")
if "math.sin(self.yaw + self.odom_yaw_offset)" not in s:
    s = s.replace("math.sin(self.yaw)", "math.sin(self.yaw + self.odom_yaw_offset)")

# Force odom quaternion to use corrected yaw before publishing
needle = "        self.odom_pub.publish(odom)"
block = """        odom_yaw_for_ros = self.yaw + self.odom_yaw_offset
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(odom_yaw_for_ros / 2.0)
        odom.pose.pose.orientation.w = math.cos(odom_yaw_for_ros / 2.0)
"""
if "odom_yaw_for_ros = self.yaw + self.odom_yaw_offset" not in s and needle in s:
    s = s.replace(needle, block + needle, 1)

# Make TF use the same corrected orientation
needle2 = "        self.tf_broadcaster.sendTransform(t)"
block2 = """        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
"""
if "t.transform.rotation.x = odom.pose.pose.orientation.x" not in s and needle2 in s:
    s = s.replace(needle2, block2 + needle2, 1)

p.write_text(s)
print("patched:", p)
print("backup:", bak)
