#!/bin/bash
# ROS 2 environment for the Waveshare UGV hosted by the Jetson.
# Source this file on either the laptop or Jetson:
#   source ~/roboinspec_ws/ros_ws/src/real/scripts/env_jetson.sh  # laptop
#   source ~/env_jetson.sh                                       # Jetson

source /opt/ros/humble/setup.bash

# Source whichever machine-local overlay exists.
[ -f "$HOME/ros2_ws/install/setup.bash" ] && \
    source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/roboinspec_ws/ros_ws/install/setup.bash" ] && \
    source "$HOME/roboinspec_ws/ros_ws/install/setup.bash"

export ROS_DOMAIN_ID=74
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/.ros/cyclone_jetson_hotspot.xml"
unset FASTRTPS_DEFAULT_PROFILES_FILE

if [ ! -f "$HOME/.ros/cyclone_jetson_hotspot.xml" ]; then
    echo "[env_jetson] WARNING: missing $HOME/.ros/cyclone_jetson_hotspot.xml" >&2
fi

echo "[env_jetson] domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION profile=$CYCLONEDDS_URI"
