"""Cartographer SLAM against the physical TB3 (wall-clock time).

Robot-side bringup must already be running (use real/scripts/tb3_robot_start.sh).
Drive with teleop while watching the map grow in the Cartographer RViz window,
then save with:

    ros2 run nav2_map_server map_saver_cli -f \
        ~/roboinspec_ws/ros_ws/src/real/maps/lab_arena

Mirrors sim/launch/mapping.launch.py minus Gazebo, with use_sim_time=false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cartographer = get_package_share_directory('turtlebot3_cartographer')
    pkg_real = get_package_share_directory('real')

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': 'true',
        }.items(),
    )

    # Live camera view while mapping (same republish as view_real.launch.py)
    camera_republish = Node(
        package='image_transport',
        executable='republish',
        name='camera_republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/image_raw_local'),
        ],
        output='screen',
    )

    return LaunchDescription([
        cartographer,
        camera_republish,
    ])
