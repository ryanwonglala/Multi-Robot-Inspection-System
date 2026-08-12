"""Regression checks for the physical TB3's short-path Nav2 tuning.

These checks are deliberately pure YAML/geometry tests.  They protect the
specific start -> viewpoint_1 case that exposed bare MPPI's short lateral-goal
failure without requiring a robot, ROS graph, or running controller server.
"""

import math
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def load_yaml(relative_path):
    with (PACKAGE / relative_path).open(encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def controller_config():
    params = load_yaml('config/nav2_real.yaml')
    return params['controller_server']['ros__parameters']


def test_amcl_updates_within_short_viewpoint_approaches():
    params = load_yaml('config/nav2_real.yaml')
    amcl = params['amcl']['ros__parameters']

    assert 0.0 < amcl['update_min_d'] <= 0.05
    assert 0.0 < amcl['update_min_a'] <= 0.05


def test_amcl_bootstraps_from_v5_home_pose():
    params = load_yaml('config/nav2_real.yaml')
    world = load_yaml('config/world_model_real_v5.yaml')
    amcl = params['amcl']['ros__parameters']
    home = world['robot_start']['pose']
    assert amcl['set_initial_pose'] is True
    for key in ('x', 'y', 'yaw'):
        assert math.isclose(amcl['initial_pose'][key], home[key], abs_tol=1e-9)


def test_short_lateral_goal_uses_rotation_shim_around_mppi():
    follow = controller_config()['FollowPath']

    assert (
        follow['plugin']
        == 'nav2_rotation_shim_controller::RotationShimController'
    )
    assert (
        follow['primary_controller']
        == 'nav2_mppi_controller::MPPIController'
    )
    assert follow['rotate_to_goal_heading'] is False
    assert follow['closed_loop'] is True


def test_rotation_shim_can_sample_the_start_to_vp1_path():
    world = load_yaml('config/world_model_real.yaml')
    start = world['robot_start']['pose']
    vp1 = world['areas']['arena']['viewpoints'][0]
    path_distance = math.hypot(
        vp1['x'] - start['x'], vp1['y'] - start['y'])

    follow = controller_config()['FollowPath']
    sample_distance = follow['forward_sampling_distance']

    # RotationShim falls back to the primary controller if the configured
    # look-ahead point does not exist.  Its old 0.5 m default exceeded this
    # entire 0.36 m path and silently recreated the original failure.
    assert 0.0 < sample_distance < path_distance
    assert (
        0.0
        < follow['angular_disengage_threshold']
        < follow['angular_dist_threshold']
        < math.pi
    )


def test_spin_collision_checker_uses_local_costmap_frame():
    params = load_yaml('config/nav2_real.yaml')
    behavior = params['behavior_server']['ros__parameters']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    assert behavior['costmap_topic'] == 'local_costmap/costmap_raw'
    assert behavior['footprint_topic'] == 'local_costmap/published_footprint'
    assert behavior['global_frame'] == local['global_frame'] == 'odom'


def test_viewpoint_arrival_is_position_only_with_five_cm_tolerance():
    controller = controller_config()
    follow = controller['FollowPath']
    checker = controller['viewpoint_position_checker']

    assert controller['goal_checker_plugins'] == [
        'viewpoint_position_checker']
    assert checker['plugin'] == 'nav2_controller::PositionGoalChecker'
    assert checker['stateful'] is False
    assert checker['xy_goal_tolerance'] == 0.05
    assert follow['rotate_to_goal_heading'] is False
    assert follow['GoalAngleCritic']['enabled'] is False


def test_progress_checker_allows_five_cm_final_approach():
    controller = controller_config()
    progress = controller['progress_checker']
    checker = controller['viewpoint_position_checker']

    assert progress['required_movement_radius'] < checker['xy_goal_tolerance']
    assert progress['required_movement_radius'] == 0.02
    assert progress['movement_time_allowance'] == 15.0


def test_v5_world_model_has_only_three_routine_patrol_points():
    world = load_yaml('config/world_model_real_v5.yaml')
    arena = world['areas']['arena']
    home = world['robot_start']

    assert [point['id'] for point in arena['viewpoints']] == [
        'vp1', 'vp2', 'vp3']
    assert all(
        point['arrival_orientation_required'] is False
        for point in arena['viewpoints']
    )
    anomaly = arena['anomaly_handling']['former_vp2_region']
    vp4 = anomaly['stopping_pose']
    assert vp4['id'] == 'vp4'
    assert vp4['arrival_orientation_required'] is True
    assert vp4['approach_mode'] == 'reverse_parking'
    assert vp4['arrival_trigger'] == 'mechanical_arm_unload_ready'
    assert vp4['xy_tolerance_m'] == 0.03
    assert vp4['yaw_tolerance_rad'] == 0.05
    approach = anomaly['approach']
    assert approach['mode'] == (
        'nav2_global_to_coarse_A_then_apriltag_terminal')
    assert approach['direct_reverse_from_vp3_forbidden'] is True
    assert approach['final_behavior'] == '/backup'
    assert approach['final_leg_collision_checked'] is True
    assert approach['final_reverse_speed_mps'] <= 0.03
    assert anomaly['capture_status'] == (
        'site_recorded_pending_local_docking_solution')
    validation = anomaly['autonomous_validation']
    assert validation['amcl_numeric_tolerance_passed'] is True
    assert validation['physical_operator_passed'] is False
    assert validation['tolerance_passed'] is False
    assert validation['arm_trigger_emitted'] is False
    assert validation['applies_to_current_pose'] is False
    assert validation['status'] == 'failed_physical_operator_validation'
    assert validation['observed_xy_error_m'] <= vp4['xy_tolerance_m']
    assert validation['observed_yaw_error_rad'] <= vp4['yaw_tolerance_rad']
    assert anomaly['included_in_routine_patrol'] is False
    assert home['arrival_orientation_required'] is True
    assert home['xy_tolerance_m'] == 0.05
    assert home['yaw_tolerance_rad'] == 0.08


def test_hybrid_dock_uses_operator_requested_left_shift_and_safe_gates():
    world = load_yaml('config/world_model_real_v5.yaml')
    anomaly = world['areas']['arena']['anomaly_handling']['former_vp2_region']
    reference = anomaly['approach']['hybrid_docking_reference']
    coarse = reference['coarse_A']
    stop = reference['perfect_stop']

    assert reference['status'] == 'operator_validated'
    assert reference['lateral_adjustment_map_x_m'] == -0.015
    assert math.isclose(coarse['x'], 1.465, abs_tol=1e-9)
    assert math.isclose(stop['x'], 1.473, abs_tol=1e-9)
    assert math.isclose(coarse['y'], -0.429, abs_tol=1e-9)
    assert math.isclose(stop['y'], -0.330, abs_tol=1e-9)
    assert stop['operator_confirmed'] is True
    assert stop['validation_status'] == 'operator_confirmed_stable'
    assert reference['policy']['apriltag_required_for_terminal_correction']
    assert reference['policy']['odom_and_rear_lidar_required_for_final_reverse']
    assert reference['policy']['map_pose_alone_must_not_trigger_unloading']
