import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# One Nav2 stack per robot. Initial poses must match the spawn poses in
# sim/launch/multi_sim.launch.py.
#
# NOTE: this launch deliberately does NOT include bringup_nav.launch.py per
# robot. Launch configurations are global across includes, and bringup_nav's
# internal TimerActions read LaunchConfiguration('namespace') lazily at fire
# time — with two robots the second include overwrites the global value
# before the first robot's timers fire, so both Nav2 stacks would land in the
# last namespace. Here every per-robot value is a literal, and each deferred
# include evaluates atomically when its timer fires.
ROBOTS = [
    # Must match the spawn poses in sim/launch/multi_sim.launch.py:
    # both robots docked on opposite walls of the mother_base corridor.
    {'ns': 'tb3', 'init_x': -4.8, 'init_y': -3.825, 'init_yaw': -1.5708},
    {'ns': 'arm', 'init_x': -4.8, 'init_y': -2.78,  'init_yaw': 1.5708},
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('sim')
    pkg_task_layer = get_package_share_directory('task_layer')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    map_yaml = os.path.join(pkg_task_layer, 'maps', 'tb3_map.yaml')
    params_file = os.path.join(pkg_task_layer, 'config', 'nav2_inspection.yaml')
    # Dual view: full tb3 stack view + the arm's amcl_pose/plan overlaid in
    # orange. Both map frames are physically aligned, so the arm overlays
    # (map-frame topics) need no TF from the arm's tree.
    rviz_config = os.path.join(pkg_task_layer, 'rviz', 'dual_view.rviz')

    # 1. Gazebo + both namespaced robots
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sim, 'launch', 'multi_sim.launch.py'))),
    ]

    for index, robot in enumerate(ROBOTS):
        ns = robot['ns']

        # 2. Nav2 stack for this robot (staggered 4 s apart so two lifecycle
        #    bringups do not fight for CPU during activation)
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'true',
                'map': map_yaml,
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        )
        actions.append(TimerAction(period=10.0 + 4.0 * index, actions=[nav2]))

        # 3. One RViz, looking at the first robot (namespaced view config)
        if index == 0:
            rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')),
                launch_arguments={
                    'namespace': ns,
                    'use_namespace': 'true',
                    'rviz_config': rviz_config,
                }.items(),
            )
            actions.append(TimerAction(period=12.0, actions=[rviz]))

        # 4. Initial pose for this robot (after its AMCL is up)
        initial_pose = Node(
            package='task_layer',
            executable='set_initial_pose_node.py',
            name='set_initial_pose',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'x': robot['init_x'],
                'y': robot['init_y'],
                'z': 0.0325,
                'yaw': robot['init_yaw'],
                'frame_id': 'map',
                'delay_sec': 0.0,
                'repeat_count': 10,
                'repeat_period_sec': 0.5,
            }],
        )
        actions.append(TimerAction(period=25.0 + 4.0 * index, actions=[initial_pose]))

    return LaunchDescription(actions)
