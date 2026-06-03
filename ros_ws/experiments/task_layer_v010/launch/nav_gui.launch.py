from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_WORLD_MODEL = PathJoinSubstitution([
    FindPackageShare('task_layer_v010'),
    'config',
    'world_model.yaml',
])

ARGS = [
    DeclareLaunchArgument('world_model_path', default_value=DEFAULT_WORLD_MODEL),
    DeclareLaunchArgument('yaw', default_value='0.0'),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
]


def generate_launch_description():
    return LaunchDescription([
        *ARGS,
        Node(
            package='task_layer_v010',
            executable='nav_gui',
            name='nav_gui',
            output='screen',
            parameters=[{
                'world_model_path': LaunchConfiguration('world_model_path'),
                'yaw': LaunchConfiguration('yaw'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
