#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

set -u

SLAM_PARAMS="${SLAM_PARAMS:-$HOME/slam_ugv02_slow.yaml}"
UGV_PORT="${UGV_PORT:-/dev/ttyTHS1}"
UGV_CONTROL_MODE="${UGV_CONTROL_MODE:-velocity}"
UGV_USE_GYRO_HEADING="${UGV_USE_GYRO_HEADING:-True}"
UGV_ENCODER_SWAP="${UGV_ENCODER_SWAP:-False}"
UGV_ODOM_UNITS="${UGV_ODOM_UNITS:-centimeters}"
UGV_LEFT_SPEED_RATE="${UGV_LEFT_SPEED_RATE:-1.0}"
UGV_RIGHT_SPEED_RATE="${UGV_RIGHT_SPEED_RATE:-1.0}"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyCH341USB0}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
LIDAR_LAUNCH="${LIDAR_LAUNCH:-sllidar_a1_launch.py}"
LIDAR_SCAN_MODE="${LIDAR_SCAN_MODE:-Sensitivity}"

cleanup() {
  echo
  echo "Stopping lidar SLAM helper processes..."
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
  python3 "$HOME/ugv02_opensource_node.py" --ros-args \
    -p serial_port:="$UGV_PORT" \
    -p control_mode:="$UGV_CONTROL_MODE" \
    -p odom_feedback_units:="$UGV_ODOM_UNITS" \
    -p encoder_swap:="$UGV_ENCODER_SWAP" \
    -p left_speed_rate:="$UGV_LEFT_SPEED_RATE" \
    -p right_speed_rate:="$UGV_RIGHT_SPEED_RATE" \
    -p use_gyro_heading:="$UGV_USE_GYRO_HEADING" &
  echo "Keep the robot still for 5 seconds while the IMU gyro bias settles..."
  sleep 5
else
  echo "Using existing UGV02 base node."
  echo "If TF yaw drifts after repeated in-place rotations, restart this script after stopping the existing UGV02 node."
fi

if [ ! -e "$LIDAR_PORT" ]; then
  echo "Lidar port not found: $LIDAR_PORT"
  echo "Current serial devices:"
  find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyCH341USB*' -o -name 'ttyACM*' \) -print
  exit 1
fi

if [ ! -f "$SLAM_PARAMS" ]; then
  echo "SLAM params file not found: $SLAM_PARAMS"
  exit 1
fi

if pgrep -f '/sllidar_node' >/dev/null || pgrep -f 'async_slam_toolbox_node' >/dev/null; then
  echo "Lidar SLAM already appears to be running."
  echo "Do not start a second copy; one RPLIDAR serial port can only be used by one driver."
  echo "Check it with:"
  echo "  ros2 topic echo /scan --once"
  echo "  ros2 topic echo /map --once"
  exit 0
fi

echo "Starting static TF base_link -> laser..."
ros2 run tf2_ros static_transform_publisher \
  --x 0.12 --y 0.0 --z 0.18 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id laser &
sleep 1

echo "Starting RPLIDAR A1 on $LIDAR_PORT @ $LIDAR_BAUD..."
ros2 launch sllidar_ros2 "$LIDAR_LAUNCH" \
  serial_port:="$LIDAR_PORT" \
  serial_baudrate:="$LIDAR_BAUD" \
  frame_id:=laser \
  inverted:=false \
  angle_compensate:=true \
  scan_mode:="$LIDAR_SCAN_MODE" &
sleep 5

echo "Starting slam_toolbox with $SLAM_PARAMS..."
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:="$SLAM_PARAMS" &
sleep 3

if [ -f "$HOME/odom_to_path.py" ] && ! pgrep -f '/odom_to_path.py' >/dev/null; then
  echo "Starting odom path publisher on /odom_path..."
  python3 "$HOME/odom_to_path.py" &
  sleep 1
fi

echo
echo "Lidar SLAM stack is starting. In another terminal, verify:"
echo "  ros2 topic hz /scan"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic echo /odom_path --once"
echo
echo "Drive slowly. Press Ctrl+C here to stop lidar SLAM helpers."
wait
