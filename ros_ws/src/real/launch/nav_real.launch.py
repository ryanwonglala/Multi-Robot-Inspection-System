"""Nav2 point-to-point navigation on the physical TB3.

Laptop-side only: map_server + AMCL + Nav2 stack + RViz. Robot-side bringup
must already be running. Set the initial pose in RViz (2D Pose Estimate at
the taped start cross), then send goals with 2D Goal Pose.

    ros2 launch real nav_real.launch.py
    ros2 launch real nav_real.launch.py map:=/path/to/other_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_real = get_package_share_directory('real')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg_real, 'maps', 'lab_arena.yaml')
    default_params = os.path.join(pkg_real, 'config', 'nav2_real.yaml')
    default_world_model = os.path.join(
        pkg_real, 'config', 'world_model_real.yaml')

    args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to the real-site map YAML'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 params tuned for the physical robot'),
        DeclareLaunchArgument(
            'world_model',
            default_value=default_world_model,
            description='World model used for RViz viewpoint markers'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Off by default: the republish node holds a CONTINUOUS
        # /image_raw/compressed subscription (~1-2 MB/s over the campus
        # WiFi uplink) that competes with /scan and with the inspection
        # runner's momentary frame grabs. Enable only for teleop sessions
        # where a live camera view in RViz is actually needed.
        DeclareLaunchArgument('use_camera_view', default_value='false'),
    ]

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav_real',
        arguments=['-d', os.path.join(
            pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        output='screen',
    )

    viewpoint_markers = Node(
        package='real',
        executable='viewpoint_markers.py',
        name='viewpoint_markers',
        parameters=[{
            'world_model_path': LaunchConfiguration('world_model'),
            # nav2_default_view.rviz already has an enabled MarkerArray display
            # on /waypoints, so the points appear without manual RViz setup.
            'topic': '/waypoints',
            'area': 'arena',
            'goal_tolerance_m': 0.05,
        }],
        output='screen',
    )

    camera_republish = Node(
        package='image_transport',
        executable='republish',
        name='camera_republish',
        condition=IfCondition(LaunchConfiguration('use_camera_view')),
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/image_raw_local'),
        ],
        output='screen',
    )

    return LaunchDescription(
        args + [nav2, viewpoint_markers, rviz, camera_republish])
