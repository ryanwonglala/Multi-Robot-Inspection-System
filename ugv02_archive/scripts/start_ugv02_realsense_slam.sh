#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

set -u

SLAM_PARAMS="${SLAM_PARAMS:-$HOME/slam_ugv02_slow.yaml}"
UGV_PORT="${UGV_PORT:-/dev/ttyTHS1}"

cleanup() {
  echo
  echo "Stopping SLAM helper processes..."
  jobs -pr | xargs -r kill
}
trap cleanup INT TERM EXIT

ugv_count="$(pgrep -fc 'ugv02_opensource_node.py' || true)"
if [ "$ugv_count" -gt 1 ]; then
  echo "Found $ugv_count ugv02_opensource_node.py processes."
  echo "Close the extra UGV02 node terminals with Ctrl+C, then run this script again."
  exit 1
fi

if [ "$ugv_count" -eq 0 ]; then
  echo "Starting UGV02 base node on $UGV_PORT..."
  python3 "$HOME/ugv02_opensource_node.py" --ros-args -p serial_port:="$UGV_PORT" &
  sleep 4
else
  echo "Using existing UGV02 base node."
fi

if [ ! -f "$SLAM_PARAMS" ]; then
  echo "SLAM params file not found: $SLAM_PARAMS"
  exit 1
fi

echo "Starting static TF base_link -> camera_camera_link..."
ros2 run tf2_ros static_transform_publisher \
  --x 0.12 --y 0.0 --z 0.22 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id camera_camera_link &
sleep 1

echo "Starting RealSense depth camera..."
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=true \
  base_frame_id:=camera_link &
sleep 7

echo "Starting depth image -> /scan..."
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args \
  -r depth:=/camera/camera/depth/image_rect_raw \
  -r depth_camera_info:=/camera/camera/depth/camera_info \
  -r scan:=/scan \
  -p output_frame:=camera_depth_frame \
  -p scan_height:=20 \
  -p range_min:=0.45 \
  -p range_max:=5.0 &
sleep 3

echo "Starting slam_toolbox with $SLAM_PARAMS..."
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:="$SLAM_PARAMS" &
sleep 3

echo
echo "SLAM stack is starting. In another terminal, verify:"
echo "  ros2 topic list -t"
echo "  ros2 topic hz /scan"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo
echo "Drive slowly with WASD or a low /cmd_vel publisher. Press Ctrl+C here to stop SLAM helpers."
wait
