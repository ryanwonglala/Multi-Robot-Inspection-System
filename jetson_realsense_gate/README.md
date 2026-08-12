# Jetson RealSense Gate

Jetson-side integration gate for the RoboInspect physical demo. It combines a
TurtleBot3 arrival signal, an Intel RealSense region-of-interest (ROI) check,
and the standalone SO-ARM101 response command.

> **Status:** commissioned prototype. Keep an operator present whenever the
> gate can start physical arm motion.

## Responsibilities

The default launcher:

1. waits for the configured ROS 2 arrival message;
2. monitors the saved RealSense ROI;
3. requires a red target to remain inside the ROI for five seconds; and
4. starts `so-arm101/auto_clear.py` once per accepted event.

The gate publishes status on `/load_unload_gate/status`. The optional YOLO
path is a visual overlay only and is disabled by default.

## Contents

- `turtlebot3_load_arm_gate.py` — ROS 2 arrival/vision gate and arm trigger.
- `realsense_roi_alarm.py` — V4L2 depth/color capture, ROI calibration, and
  standalone intrusion test. It intentionally does not require
  `pyrealsense2`.
- `realsense_yolo_node.py` — optional ROS 2 YOLO image overlay/reference node.
- `scripts/run_turtlebot3_load_arm_gate.sh` — default arrival-gated launcher.
- `scripts/run_realsense_roi_alarm.sh` — ROI calibration launcher.

## Requirements

- Ubuntu on Jetson with ROS 2 Humble and `rclpy`.
- Intel RealSense exposed as V4L2 depth and color devices.
- Python 3 with OpenCV and NumPy.
- A working SO-ARM101 environment for automatic clearing.
- `torch` and `ultralytics` only when the optional YOLO overlay is enabled.

The launchers source `/opt/ros/humble/setup.bash` and, when present,
`~/ros2_ws/install/setup.bash`.

## Calibrate the ROI

From the repository root on Jetson:

```bash
DISPLAY=:1 ./jetson_realsense_gate/scripts/run_realsense_roi_alarm.sh
```

Draw the ROI with the mouse, press `b` to capture a clean background, and
press `s` to save. The default local files are:

```text
~/.config/realsense_roi_alarm/config.json
~/.config/realsense_roi_alarm/background_roi.npz
```

These device- and site-specific files are not committed. Use `--depth-device`,
`--color-device`, or `--roi x,y,w,h` when device numbering or placement
changes.

## Run the gate

```bash
SOARM_ROOT="$PWD/so-arm101" \
ROS_DOMAIN_ID=2 DISPLAY=:1 \
./jetson_realsense_gate/scripts/run_turtlebot3_load_arm_gate.sh
```

The launcher expects a `std_msgs/msg/String` on
`/turtlebot3/load_unload_arrived` containing:

```text
Ready, waiting for recognition results
```

All script options are available with:

```bash
python3 jetson_realsense_gate/turtlebot3_load_arm_gate.py --help
python3 jetson_realsense_gate/realsense_roi_alarm.py --help
```

To enable the optional overlay, install its dependencies and provide the model
explicitly:

```bash
YOLO_MODEL=/path/to/model.pt \
./jetson_realsense_gate/scripts/run_turtlebot3_load_arm_gate.sh --yolo
```

## Safety and deployment limits

- Calibrate the ROI again after moving the camera, changing resolution, or
  changing the work surface.
- Verify `SOARM_ROOT` and `SOARM_PORT` before allowing a trigger.
- Keep the arm workspace clear and test perception without arm motion first.
- Do not treat the YOLO overlay as a safety interlock.
- This module is a supervised demo integration, not a certified safety system.
