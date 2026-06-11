import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_task_layer = get_package_share_directory('task_layer')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Map ships inside the package so any clone/machine resolves it the same way.
    default_map = os.path.join(pkg_task_layer, 'maps', 'tb3_map.yaml')
    default_params = os.path.join(pkg_task_layer, 'config', 'nav2_inspection.yaml')

    args = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Robot namespace, e.g. tb3 / arm. Empty = legacy single-robot'),
        DeclareLaunchArgument(
            'use_namespace', default_value='false',
            description='Must be true whenever namespace is non-empty'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Full path to SLAM map YAML file'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to Nav2 params YAML file'),
        DeclareLaunchArgument(
            'start_gazebo', default_value='true',
            description='false when an outer launch already started Gazebo (multi-robot)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('init_x', default_value='-4.8'),
        DeclareLaunchArgument('init_y', default_value='-3.825'),
        DeclareLaunchArgument('init_yaw', default_value='-1.5708'),
    ]

    # ── 1. Gazebo + single-robot spawn ──────────────────────────────────────
    # Multi-robot mode starts Gazebo (and namespaced spawns) via
    # sim/multi_sim.launch.py instead, with start_gazebo:=false here.
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'map.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_gazebo')),
    )

    # ── 2. Nav2 full stack (T+8 s) ──────────────────────────────────────────
    # nav2_bringup natively supports namespacing on Humble: with
    # use_namespace:=true every node lands in <ns> and /tf is remapped to
    # <ns>/tf (independent TF tree per robot, frame names unchanged).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
        }.items(),
    )
    nav2_delayed = TimerAction(period=8.0, actions=[nav2])

    # ── 3. RViz — namespace-aware variant from nav2_bringup (T+8 s) ────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': LaunchConfiguration('use_namespace'),
        }.items(),
    )
    rviz_delayed = TimerAction(period=8.0, actions=[rviz])

    # ── 4. Initial pose publisher (T+15 s — AMCL must be up first) ─────────
    # Publishes <ns>/initialpose 10 × at 0.5 s intervals then shuts down.
    # LaunchConfiguration values arrive as strings; ParameterValue coerces
    # them back to the double type the node declares.
    initial_pose = Node(
        package='task_layer',
        executable='set_initial_pose_node.py',
        name='set_initial_pose',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'x': ParameterValue(LaunchConfiguration('init_x'), value_type=float),
            'y': ParameterValue(LaunchConfiguration('init_y'), value_type=float),
            'z': 0.0325,
            'yaw': ParameterValue(LaunchConfiguration('init_yaw'), value_type=float),
            'frame_id': 'map',
            'delay_sec': 0.0,
            'repeat_count': 10,
            'repeat_period_sec': 0.5,
        }],
    )
    initial_pose_delayed = TimerAction(period=15.0, actions=[initial_pose])

    return LaunchDescription(args + [
        gazebo_sim,
        nav2_delayed,
        rviz_delayed,
        initial_pose_delayed,
    ])
