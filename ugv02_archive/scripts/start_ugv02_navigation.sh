#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

set -u

MAP_FILE="${MAP_FILE:-$HOME/maps/ugv02_map.yaml}"
NAV_PARAMS="${NAV_PARAMS:-$HOME/nav2_ugv02_params.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$HOME/nav2_ugv02.rviz}"
UGV_PORT="${UGV_PORT:-/dev/ttyTHS1}"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyCH341USB0}"
START_RVIZ="${START_RVIZ:-true}"

cleanup() {
  echo
  echo "Stopping UGV02 navigation helper processes..."
  jobs -pr | xargs -r kill
}
trap cleanup INT TERM EXIT

if pgrep -f '[a]sync_slam_toolbox_node' >/dev/null; then
  echo "SLAM Toolbox is still running. Stop the mapping stack before navigation."
  exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
  echo "Map file not found: $MAP_FILE"
  exit 1
fi
if [ ! -f "$NAV_PARAMS" ]; then
  echo "Nav2 parameter file not found: $NAV_PARAMS"
  exit 1
fi
if [ ! -e "$LIDAR_PORT" ]; then
  echo "RPLIDAR port not found: $LIDAR_PORT"
  exit 1
fi

ugv_count="$(pgrep -fc '[u]gv02_opensource_node.py' || true)"
if [ "$ugv_count" -eq 0 ]; then
  echo "Starting UGV02 base node on $UGV_PORT..."
  python3 "$HOME/ugv02_opensource_node.py" --ros-args \
    -p serial_port:="$UGV_PORT" \
    -p control_mode:=velocity \
    -p odom_feedback_units:=centimeters \
    -p encoder_swap:=False \
    -p use_gyro_heading:=True &
  echo "Keep the robot still for 5 seconds while the IMU settles..."
  sleep 5
elif [ "$ugv_count" -gt 1 ]; then
  echo "More than one UGV02 base node is running. Stop duplicates first."
  exit 1
else
  echo "Using the existing UGV02 base node."
fi

echo "Starting static TF base_link -> laser..."
# Calibrated 2026-07-28 against the TB3 lab_arena map from two
# scan-matched poses before/after an in-place Jetson rotation.  The previous
# x=+0.12,y=0 transform made the inferred base center move around the laser
# and placed the footprint inside the north wall.
ros2 run tf2_ros static_transform_publisher \
  --x -0.012 --y -0.037 --z 0.18 --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id laser &

echo "Starting RPLIDAR A1..."
ros2 launch sllidar_ros2 sllidar_a1_launch.py \
  serial_port:="$LIDAR_PORT" \
  serial_baudrate:=115200 \
  frame_id:=laser \
  inverted:=false \
  angle_compensate:=true \
  scan_mode:=Sensitivity &
sleep 3

echo "Starting Nav2 with map: $MAP_FILE"
ros2 launch nav2_bringup bringup_launch.py \
  slam:=False \
  map:="$MAP_FILE" \
  params_file:="$NAV_PARAMS" \
  use_sim_time:=False \
  autostart:=True \
  use_composition:=False &
sleep 8

if [ "$START_RVIZ" = "true" ] && [ -n "${DISPLAY:-}" ]; then
  echo "Starting RViz..."
  rviz2 -d "$RVIZ_CONFIG" &
else
  echo "RViz was not started. In a VNC terminal run:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  rviz2 -d $RVIZ_CONFIG"
fi

echo
echo "Navigation is ready for initial localization."
echo "In RViz: 2D Pose Estimate -> set position and heading -> Nav2 Goal."
echo "Keep this terminal open. Press Ctrl+C to stop navigation."
wait
