import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_MODEL = os.path.join(
    get_package_share_directory('task_layer'),
    'config',
    'world_model.yaml',
)

ARGS = [
    DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'),
    DeclareLaunchArgument(
        'world_model_path', default_value=WORLD_MODEL,
        description='Path to world_model.yaml'),
    DeclareLaunchArgument(
        'world', default_value='map',
        description='Gazebo world name (for model spawning)'),
    DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Default yaw for single-shot Navigate tab goals'),
    DeclareLaunchArgument(
        'robots', default_value="['tb3','arm']",
        description="Robot namespaces the GUI commands. Legacy single robot: \"['']\""),
]


def generate_launch_description():
    return LaunchDescription([
        *ARGS,
        Node(
            package='task_layer',
            executable='task_gui_node.py',
            name='task_gui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world_model_path': LaunchConfiguration('world_model_path'),
                'world': LaunchConfiguration('world'),
                'yaw': LaunchConfiguration('yaw'),
                # value_type=None -> the string is yaml-parsed into a list
                'robots': ParameterValue(
                    LaunchConfiguration('robots'), value_type=None),
            }],
        ),
    ])
