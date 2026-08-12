#!/bin/bash
# Formal dual-robot environment: Laptop + TB3 + Jetson UGV on one ROS domain.
# Run fleet_hotspot_reconnect.py once after the phone hotspot is enabled, then
# source this file in every laptop terminal used for dual-robot operation.

source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && \
    source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/roboinspec_ws/ros_ws/install/setup.bash" ] && \
    source "$HOME/roboinspec_ws/ros_ws/install/setup.bash"

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_fleet.xml"
export TURTLEBOT3_MODEL=burger
unset CYCLONEDDS_URI

[ -f "$HOME/.ros/fleet_hotspot.env" ] && \
    source "$HOME/.ros/fleet_hotspot.env"

if [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "[env_fleet] WARNING: missing $FASTRTPS_DEFAULT_PROFILES_FILE" >&2
    echo "[env_fleet] run fleet_hotspot_reconnect.py first" >&2
fi

echo "[env_fleet] domain=$ROS_DOMAIN_ID profile=$FASTRTPS_DEFAULT_PROFILES_FILE"
if [ -n "${TB3_ROBOT_IP:-}${JETSON_IP:-}" ]; then
    echo "[env_fleet] TB3=${TB3_ROBOT_IP:-unknown} Jetson=${JETSON_IP:-unknown}"
fi
