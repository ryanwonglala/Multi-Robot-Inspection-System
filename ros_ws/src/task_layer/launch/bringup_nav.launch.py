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
    pkg_task_layer = get_package_share_directory('task_layer')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Map ships inside the package so any clone/machine resolves it the same way.
    default_map = os.path.join(pkg_task_layer, 'maps', 'tb3_map.yaml')
    params_file = os.path.join(pkg_task_layer, 'config', 'nav2_inspection.yaml')
    rviz_config = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to SLAM map YAML file')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=params_file,
        description='Full path to Nav2 params YAML file')

    # ── 1. Gazebo + robot spawn ─────────────────────────────────────────────
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'map.launch.py')
        ),
    )

    # ── 2. Nav2 full stack via bringup_launch.py (T+8 s) ───────────────────
    # bringup_launch.py starts: map_server, amcl, bt_navigator,
    # controller_server, planner_server, smoother, behaviors, waypoint_follower,
    # velocity_smoother, collision_monitor, lifecycle_manager
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
        }.items(),
    )

    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    # ── 3. RViz with Nav2 default config (T+8 s, same as Nav2) ────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz_delayed = TimerAction(period=8.0, actions=[rviz])

    # ── 4. Initial pose publisher (T+15 s — AMCL must be up first) ─────────
    # Publishes /initialpose 10 × at 0.5 s intervals then shuts down.
    # delay_sec=0 because TimerAction already provides the 15 s delay.
    initial_pose = Node(
        package='task_layer',
        executable='set_initial_pose_node.py',
        name='set_initial_pose',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'x': -4.8,
            'y': -3.825,
            'z': 0.0325,
            'yaw': -1.5708,
            'frame_id': 'map',
            'delay_sec': 0.0,
            'repeat_count': 10,
            'repeat_period_sec': 0.5,
        }],
    )

    initial_pose_delayed = TimerAction(period=15.0, actions=[initial_pose])

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        gazebo_sim,
        nav2_delayed,
        rviz_delayed,
        initial_pose_delayed,
    ])
