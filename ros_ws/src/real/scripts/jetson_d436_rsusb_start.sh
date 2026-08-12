#!/usr/bin/env bash
# Start only the D436 RGB/depth/IMU stack on the Jetson.
# This script does not start the base, LiDAR, Nav2, teleop, or any cmd_vel source.

set -e

RSUSB_PREFIX="${RSUSB_PREFIX:-/home/nvidia/.local/realsense-rsusb-2.58.1}"
REALSENSE_ROS_OVERLAY="${REALSENSE_ROS_OVERLAY:-/home/nvidia/realsense-ros-rsusb-ws/install}"

if [[ ! -r "${RSUSB_PREFIX}/lib/librealsense2.so.2.58" ]]; then
  echo "Missing RSUSB librealsense at ${RSUSB_PREFIX}" >&2
  exit 2
fi

if [[ ! -r "${REALSENSE_ROS_OVERLAY}/setup.bash" ]]; then
  echo "Missing RealSense ROS overlay at ${REALSENSE_ROS_OVERLAY}" >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
source "${REALSENSE_ROS_OVERLAY}/setup.bash"

export LD_LIBRARY_PATH="${RSUSB_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_accel:=true \
  enable_gyro:=true \
  unite_imu_method:=2 \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  rgb_camera.power_line_frequency:=1 \
  "$@"
