"""View the physical TB3 in RViz over WiFi.

Assumes the robot-side bringup (turtlebot3_bringup robot.launch.py) is already
running on the TB3, so /scan, /odom, /tf and /robot_description are available
on the DDS network. This launch only starts laptop-side visualization.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('real'), 'rviz', 'real_view.rviz')

    return LaunchDescription([
        # Camera arrives over WiFi as /image_raw/compressed; decompress
        # locally so RViz's Image display gets a raw stream without pulling
        # ~27 MB/s of uncompressed video across the network.
        Node(
            package='image_transport',
            executable='republish',
            name='camera_republish',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', '/image_raw/compressed'),
                ('out', '/image_raw_local'),
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_real',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
