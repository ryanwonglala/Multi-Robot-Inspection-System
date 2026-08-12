# Jetson RealSense Gate

This module runs on the Jetson and connects the TurtleBot3 arrival signal,
RealSense ROI recognition, and SO-ARM101 auto-clear workflow.

## Workflow

1. Wait for the TurtleBot3 ready signal on ROS 2:

   ```text
   Ready, waiting for recognition results
   ```

2. Read the RealSense color/depth streams.
3. Detect a red object inside the configured ROI.
4. If the red object stays in the ROI for 5 seconds, run the SO-ARM101
   `auto_clear.py` script.

## Files

- `turtlebot3_load_arm_gate.py`
  - Main gate script. Subscribes to the ready signal, detects the red ROI
    target, and launches the arm command.
- `realsense_roi_alarm.py`
  - ROI drawing, RealSense capture, and calibration helper.
- `realsense_yolo_node.py`
  - Optional YOLO overlay/reference node.
- `scripts/run_turtlebot3_load_arm_gate.sh`
  - Default launcher for the full TurtleBot3 + RealSense + SO-ARM gate.
- `scripts/run_realsense_roi_alarm.sh`
  - Launcher for ROI calibration.

## Run

From the repository root on Jetson:

```bash
ROS_DOMAIN_ID=30 DISPLAY=:1 ./jetson_realsense_gate/scripts/run_turtlebot3_load_arm_gate.sh
```

To calibrate or redraw the ROI:

```bash
DISPLAY=:1 ./jetson_realsense_gate/scripts/run_realsense_roi_alarm.sh
```

## Environment

The gate defaults to the Jetson deployment path:

```text
/home/nvidia/Multi-Robot-Inspection-System/so-arm101
```

Override it with `SOARM_ROOT` if the repository is cloned elsewhere:

```bash
SOARM_ROOT=/path/to/Multi-Robot-Inspection-System/so-arm101 \
ROS_DOMAIN_ID=30 DISPLAY=:1 \
./jetson_realsense_gate/scripts/run_turtlebot3_load_arm_gate.sh
```

If the YOLO model is stored somewhere else, set `YOLO_MODEL`:

```bash
YOLO_MODEL=/path/to/yolo11n.pt \
ROS_DOMAIN_ID=30 DISPLAY=:1 \
./jetson_realsense_gate/scripts/run_turtlebot3_load_arm_gate.sh
```
