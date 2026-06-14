import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Per-robot sim models match the real camera hardware:
#   tb3 (Robot B) -> ASUS Webcam C3       : mono RGB        (burger_cam_ns)
#   arm (Robot A) -> RealSense D436 depth : RGB-D + points  (burger_d436_ns)
# The arm sim stand-in is still a burger chassis; the Final swaps in the real
# mobile manipulator under the same 'arm' namespace, keeping the D436 model.
ROBOTS = [
    # Docked face-to-wall on opposite sides of the mother_base corridor:
    # tb3 at the south-wall charging station, arm mirrored on the north wall.
    {'ns': 'tb3', 'model': 'turtlebot3_burger_cam_ns',
     'x': '-4.8', 'y': '-3.825', 'yaw': '-1.5708'},
    {'ns': 'arm', 'model': 'turtlebot3_burger_d436_ns',
     'x': '-4.8', 'y': '-2.95',  'yaw': '1.5708'},
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gz_dir = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(pkg_sim, 'worlds', 'map.world')
    urdf_path = os.path.join(tb3_gz_dir, 'urdf', 'turtlebot3_burger_cam.urdf')
    # Each robot spawns from its own namespaced SDF (see ROBOTS['model']); both
    # variants remap the diff_drive /tf:=tf so the odom TF stays inside the
    # namespace injected by -robot_namespace. The two models differ only in the
    # camera sensor block (C3 RGB vs D436 depth). The shared burger_cam URDF
    # below provides the camera_link / camera_rgb_optical_frame for both.
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
                '-file', os.path.join(
                    pkg_sim, 'models', robot['model'], 'model.sdf'),
                '-robot_namespace', ns,
                '-x', robot['x'], '-y', robot['y'], '-z', '0.035',
                '-Y', robot['yaw'],
            ],
        ))

    return LaunchDescription(actions)
