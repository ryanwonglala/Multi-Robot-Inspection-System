from task_layer.inspection_runner import inside_clearance_radius


def test_clearance_guard_is_circular_not_square():
    goal_x, goal_y = -0.003, 0.017

    # This is the real HOME false-positive observed on 2026-07-31: inside the
    # old square iteration bounds but 27.2 cm from a 22 cm circular guard.
    assert not inside_clearance_radius(
        goal_x, goal_y, 0.173, 0.225, 0.22)

    # A cell genuinely inside the configured radius must remain protected.
    assert inside_clearance_radius(
        goal_x, goal_y, goal_x + 0.20, goal_y, 0.22)


def test_home_specific_guard_excludes_wall_edge_but_keeps_close_obstacle():
    goal_x, goal_y = -0.003, 0.017

    # Current v5 HOME observation: the two lethal cells are about 20.9 cm
    # away. They block a generic 22 cm viewpoint guard but not the validated
    # 15 cm dock guard.
    wall_x, wall_y = -0.027, 0.225
    assert inside_clearance_radius(goal_x, goal_y, wall_x, wall_y, 0.22)
    assert not inside_clearance_radius(goal_x, goal_y, wall_x, wall_y, 0.15)

    # A genuinely close dynamic obstacle must still block HOME.
    assert inside_clearance_radius(
        goal_x, goal_y, goal_x, goal_y + 0.12, 0.15)
