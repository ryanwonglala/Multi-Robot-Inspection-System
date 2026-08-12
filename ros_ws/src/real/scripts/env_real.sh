#!/bin/bash
# Laptop-side environment for talking to the physical TB3 through the phone
# hotspot.  Run tb3_hotspot_reconnect.py after every hotspot/subnet change
# before sourcing this file and starting new ROS participants.
# Usage:  source ~/roboinspec_ws/ros_ws/src/real/scripts/env_real.sh
#
# Every terminal that needs to see the robot MUST load the FastDDS unicast
# initial-peers profile.  Hotspot IPs are dynamic: never copy an old address
# from this comment or a log.  The reconnect script discovers the TB3 by its
# stable MAC and regenerates both laptop- and robot-side peer profiles.

source /opt/ros/humble/setup.bash
[ -f "$HOME/roboinspec_ws/ros_ws/install/setup.bash" ] && \
    source "$HOME/roboinspec_ws/ros_ws/install/setup.bash"

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_laptop.xml"
export TURTLEBOT3_MODEL=burger

echo "[env_real] domain=$ROS_DOMAIN_ID model=$TURTLEBOT3_MODEL profile=$FASTRTPS_DEFAULT_PROFILES_FILE"
