import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_cartographer = get_package_share_directory('turtlebot3_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    # ── 1. Gazebo + robot spawn (map.launch.py) ───────────────────────────
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'map.launch.py')
        ),
    )

    # ── 2. Cartographer SLAM (delayed 5 s — Gazebo needs time to start) ───
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'true',
        }.items(),
    )

    cartographer_delayed = TimerAction(period=5.0, actions=[cartographer])

    # ── 3. Mapping GUI controller (delayed 8 s — topics must exist first) ─
    mapping_ctl_node = Node(
        package='sim',
        executable='mapping_ctl.py',
        name='mapping_ctl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mapping_ctl_delayed = TimerAction(period=8.0, actions=[mapping_ctl_node])

    return LaunchDescription([
        declare_use_sim_time,
        gazebo_sim,
        cartographer_delayed,
        mapping_ctl_delayed,
    ])
