from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGS = [
    DeclareLaunchArgument('cmd', default_value='/cmd_vel'),
    DeclareLaunchArgument('odom', default_value='/odom'),
]


def generate_launch_description():
    return LaunchDescription([
        *ARGS,
        Node(
            package='sim',
            executable='ctl.py',
            name='tb4_ctl',
            output='screen',
            parameters=[{
                'cmd': LaunchConfiguration('cmd'),
                'odom': LaunchConfiguration('odom'),
            }],
        ),
    ])
