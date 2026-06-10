import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gz_dir = get_package_share_directory('turtlebot3_gazebo')

    # Paths
    default_world = os.path.join(pkg_sim, 'worlds', 'map.world')
    # URDF for robot_state_publisher (TF chain only — no plugins)
    urdf_path = os.path.join(tb3_gz_dir, 'urdf', 'turtlebot3_burger_cam.urdf')
    # model.sdf for Gazebo spawn — contains all sensor/drive plugins
    model_sdf_path = os.path.join(
        tb3_gz_dir, 'models', 'turtlebot3_burger_cam', 'model.sdf')
    model_path = os.path.join(tb3_gz_dir, 'models')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2')

    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='false',
        description='Launch SLAM Toolbox (not used in nav mode)')

    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Full path to the Gazebo world file')

    # Environment variables
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='burger')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')])

    # Gazebo Classic server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # robot_state_publisher — publishes /robot_description and TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn turtlebot3_burger_cam from model.sdf so Gazebo loads all plugins
    # (diff_drive, ray_sensor, camera, joint_state_publisher are in model.sdf
    #  but NOT in the burger_cam URDF — spawning via -topic would lose them)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_burger_cam',
            '-file', model_sdf_path,
            '-x', '-4.8',
            '-y', '-3.825',
            '-z', '0.035',
            '-Y', '-1.5708',
        ],
    )

    # RViz2 (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        set_tb3_model,
        set_gazebo_model_path,
        use_rviz_arg,
        use_slam_arg,
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz,
    ])
