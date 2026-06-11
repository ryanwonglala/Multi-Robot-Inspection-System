import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Mid-term: Robot A's sim stand-in is also a burger_cam; the Final swaps in
# the real mobile manipulator's model under the same 'arm' namespace.
ROBOTS = [
    {'ns': 'tb3', 'x': '-4.8', 'y': '-3.825', 'yaw': '-1.5708'},  # charging_station
    {'ns': 'arm', 'x': '-2.0', 'y': '-3.3',   'yaw': '0.0'},      # east of mother_base
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gz_dir = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(pkg_sim, 'worlds', 'map.world')
    urdf_path = os.path.join(tb3_gz_dir, 'urdf', 'turtlebot3_burger_cam.urdf')
    # Namespaced SDF variant: identical to turtlebot3_gazebo's burger_cam
    # except the diff_drive plugin remaps /tf:=tf so its odom TF stays inside
    # the namespace injected by -robot_namespace (stock plugin publishes the
    # TF on absolute /tf, which would mix both robots' odometry).
    model_sdf_path = os.path.join(
        pkg_sim, 'models', 'turtlebot3_burger_cam_ns', 'model.sdf')
    model_path = os.path.join(tb3_gz_dir, 'models')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    actions = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world}.items()),
    ]

    for robot in ROBOTS:
        ns = robot['ns']
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ))
        # -robot_namespace pushes every plugin in model.sdf into <ns>; the
        # diff_drive/sensor plugins use relative topic names on Humble, so
        # odom/scan/camera and the odom->base_footprint TF land in <ns>/...
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{ns}',
            output='screen',
            arguments=[
                '-entity', ns,
                '-file', model_sdf_path,
                '-robot_namespace', ns,
                '-x', robot['x'], '-y', robot['y'], '-z', '0.035',
                '-Y', robot['yaw'],
            ],
        ))

    return LaunchDescription(actions)
