#!/bin/bash
# Robot-side one-shot start: bringup + camera. Runs ON the TB3 (LattePanda).
# Deploy:  scp this file to tb@<robot>:~/tb3_robot_start.sh
# Usage:   bash ~/tb3_robot_start.sh          # bringup + camera
#          bash ~/tb3_robot_start.sh nocam    # bringup only
#
# Notes:
# - usb_port uses the stable by-id path: OpenCR re-enumerates (ACM1->ACM2...)
#   every time its power switch is toggled; the default ttyACM1 then breaks.
# - Bracketed pkill patterns avoid this script's own cmdline self-matching.

source /opt/ros/humble/setup.bash
[ -f "$HOME/tb_ws/install/setup.bash" ] && source "$HOME/tb_ws/install/setup.bash"

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_robot.xml"

OPENCR=/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00
if [ ! -e "$OPENCR" ]; then
    echo "ERROR: OpenCR not found at $OPENCR — check its power switch / USB." >&2
    exit 1
fi

# ld08_driver and robot_state_publisher are launch *children*: when the launch
# parent is killed they get reparented to init and keep publishing. On
# 2026-07-27 that left a second ld08_driver fighting the previous one over the
# LDS serial port (/scan halved to 7 Hz with corrupted returns) and a second
# robot_state_publisher spamming TF_REPEATED_DATA. Kill them by name too.
pkill -f '[r]obot.launch' 2>/dev/null
pkill -f '[t]urtlebot3_ros' 2>/dev/null
pkill -f '[v]4l2_camera' 2>/dev/null
pkill -f '[l]d08_driver' 2>/dev/null
pkill -f '[r]obot_state_publisher' 2>/dev/null
sleep 2

# Verify nothing survived, otherwise the new bringup inherits the same mess.
# A killed launch child can briefly remain as a zombie owned by init. Zombies
# hold no serial port and publish nothing, so they must not block a clean
# restart; only surviving, non-zombie processes are unsafe here.
LEFTOVER=$(
    pgrep -af '[l]d08_driver|[t]urtlebot3_ros|[r]obot_state_publisher|[v]4l2_camera' |
    while read -r pid command; do
        state=$(ps -o stat= -p "$pid" 2>/dev/null)
        case "$state" in
            Z*) ;;
            *) printf '%s %s\n' "$pid" "$command" ;;
        esac
    done
)
if [ -n "$LEFTOVER" ]; then
    echo "ERROR: old ROS nodes survived the cleanup, refusing to start a" >&2
    echo "       second copy. Kill these by PID and rerun:" >&2
    echo "$LEFTOVER" >&2
    exit 1
fi

nohup ros2 launch turtlebot3_bringup robot.launch.py usb_port:="$OPENCR" \
    > "$HOME/bringup.log" 2>&1 &
echo "bringup started (log: ~/bringup.log)"

if [ "$1" != "nocam" ]; then
    CAM=$(ls /dev/v4l/by-id/*CAMERA*-video-index0 2>/dev/null | head -1)
    [ -z "$CAM" ] && CAM=/dev/video14
    CAMERA_INFO="$HOME/.ros/camera_info/tb3_usb_camera_640x480.yaml"
    CAMERA_INFO_ARGS=()
    if [ -f "$CAMERA_INFO" ]; then
        CAMERA_INFO_ARGS=(-p "camera_info_url:=file://$CAMERA_INFO")
    else
        echo "WARNING: camera calibration missing at $CAMERA_INFO" >&2
    fi
    nohup ros2 run v4l2_camera v4l2_camera_node --ros-args \
        -p video_device:="$CAM" -p image_size:="[640,480]" \
        "${CAMERA_INFO_ARGS[@]}" \
        > "$HOME/camera.log" 2>&1 &
    echo "camera started on $CAM (log: ~/camera.log)"
fi
