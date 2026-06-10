from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_WORLD_MODEL = PathJoinSubstitution([
    FindPackageShare('task_layer_v020'),
    'config',
    'world_model.yaml',
])

ARGS = [
    DeclareLaunchArgument('world', default_value='map'),
    DeclareLaunchArgument('world_model_path', default_value=DEFAULT_WORLD_MODEL),
]


def generate_launch_description():
    return LaunchDescription([
        *ARGS,
        Node(
            package='task_layer_v020',
            executable='scene_builder_gui',
            name='scene_builder_gui',
            output='screen',
            parameters=[{
                'world': LaunchConfiguration('world'),
                'world_model_path': LaunchConfiguration('world_model_path'),
            }],
        ),
    ])
