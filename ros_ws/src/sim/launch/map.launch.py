import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGS = [
    DeclareLaunchArgument('world', default_value='map', description='World name in sim/worlds'),
    DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0.0'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_gui = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_tb4_desc = get_package_share_directory('turtlebot4_description')
    pkg_create_desc = get_package_share_directory('irobot_create_description')
    pkg_create_gz = get_package_share_directory('irobot_create_gz_bringup')
    pkg_create_plugins = get_package_share_directory('irobot_create_gz_plugins')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_sim, 'worlds'),
            os.path.join(pkg_tb4_gz, 'worlds'),
            os.path.join(pkg_create_gz, 'worlds'),
            str(Path(pkg_tb4_desc).parent.resolve()),
            str(Path(pkg_create_desc).parent.resolve()),
        ]),
    )

    gui_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_tb4_gui, 'lib'),
            os.path.join(pkg_create_plugins, 'lib'),
        ]),
    )

    gz_launch = PathJoinSubstitution([pkg_ros_gz, 'launch', 'gz_sim.launch.py'])
    spawn_launch = PathJoinSubstitution([pkg_tb4_gz, 'launch', 'turtlebot4_spawn.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'), '.sdf',
                ' -r -v 4 --gui-config ',
                PathJoinSubstitution([
                    pkg_tb4_gz,
                    'gui',
                    LaunchConfiguration('model'),
                    'gui.config',
                ]),
            ]),
        ],
    )

    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_launch]),
        launch_arguments=[
            ('model', LaunchConfiguration('model')),
            ('rviz', LaunchConfiguration('rviz')),
            ('slam', LaunchConfiguration('slam')),
            ('world', LaunchConfiguration('world')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ],
    )

    ld = LaunchDescription(ARGS)
    ld.add_action(gz_path)
    ld.add_action(gui_path)
    ld.add_action(gazebo)
    ld.add_action(clock)
    ld.add_action(spawn)
    return ld
