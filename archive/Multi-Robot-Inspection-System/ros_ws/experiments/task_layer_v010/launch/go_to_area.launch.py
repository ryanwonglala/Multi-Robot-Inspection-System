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
    DeclareLaunchArgument('target', default_value='main_corridor'),
    DeclareLaunchArgument('world_model_path', default_value=DEFAULT_WORLD_MODEL),
    DeclareLaunchArgument('yaw', default_value='0.0'),
    DeclareLaunchArgument('dry_run', default_value='false'),
    DeclareLaunchArgument('use_sim_time', default_value='true'),
]


def generate_launch_description():
    return LaunchDescription([
        *ARGS,
        Node(
            package='task_layer_v010',
            executable='go_to_area',
            name='go_to_area',
            output='screen',
            parameters=[{
                'target': LaunchConfiguration('target'),
                'world_model_path': LaunchConfiguration('world_model_path'),
                'yaw': LaunchConfiguration('yaw'),
                'dry_run': LaunchConfiguration('dry_run'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
