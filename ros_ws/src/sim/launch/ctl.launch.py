import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel',
        description='Velocity command topic')

    odom_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic')

    # ctl.py is installed to lib/sim/ by CMakeLists install(PROGRAMS ...)
    ctl_node = Node(
        package='sim',
        executable='ctl.py',
        name='manual_controller',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        cmd_vel_arg,
        odom_arg,
        ctl_node,
    ])
