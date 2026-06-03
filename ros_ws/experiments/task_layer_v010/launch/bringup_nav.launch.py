from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_MAP = '/home/ryan/tb4_ws/maps/base_aligned_20260602.yaml'

ARGS = [
    DeclareLaunchArgument('world', default_value='map'),
    DeclareLaunchArgument('model', default_value='standard'),
    DeclareLaunchArgument('rviz', default_value='true'),
    DeclareLaunchArgument('map', default_value=DEFAULT_MAP),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
    DeclareLaunchArgument('x', default_value='-4.8'),
    DeclareLaunchArgument('y', default_value='-3.5'),
    DeclareLaunchArgument('z', default_value='0.0'),
    DeclareLaunchArgument('yaw', default_value='-1.5708'),
    DeclareLaunchArgument('set_initial_pose', default_value='true'),
    DeclareLaunchArgument('initial_pose_delay_sec', default_value='8.0'),
]


def generate_launch_description():
    sim_launch = PathJoinSubstitution([
        FindPackageShare('sim'),
        'launch',
        'map.launch.py',
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot4_navigation'),
        'launch',
        'localization.launch.py',
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot4_navigation'),
        'launch',
        'nav2.launch.py',
    ])

    gazebo_and_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'slam': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    initial_pose = Node(
        condition=IfCondition(LaunchConfiguration('set_initial_pose')),
        package='task_layer_v010',
        executable='set_initial_pose',
        name='set_initial_pose',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'delay_sec': LaunchConfiguration('initial_pose_delay_sec'),
        }],
    )

    return LaunchDescription([
        *ARGS,
        gazebo_and_robot,
        localization,
        nav2,
        initial_pose,
    ])
