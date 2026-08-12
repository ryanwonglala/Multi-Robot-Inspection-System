"""Launch the namespaced RoboInspect UGV base driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ugv_base_driver')
    default_params = os.path.join(package_share, 'config', 'ugv_base.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ugv'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'),
        Node(
            package='ugv_base_driver',
            executable='ugv_base_node',
            name='ugv_base_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                LaunchConfiguration('params_file'),
                {'port': LaunchConfiguration('port')},
            ],
            output='screen',
        ),
    ])
