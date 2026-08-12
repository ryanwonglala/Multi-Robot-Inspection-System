import time

import rclpy

from task_layer.set_initial_pose_node import SetInitialPoseNode


def test_default_ten_publications_finish_without_shutting_rclpy():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'repeat_period_sec:=0.001',
    ])
    node = SetInitialPoseNode()
    try:
        deadline = time.monotonic() + 2.0
        while not node.finished and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        assert node.finished
        assert node._sent == 10
        # Completion is signalled to main rather than shutting the ROS context
        # down from inside the timer callback.
        assert rclpy.ok()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
