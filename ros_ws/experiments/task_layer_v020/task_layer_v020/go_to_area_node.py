#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


def default_world_model_path() -> str:
    share_dir = get_package_share_directory('task_layer_v020')
    return str(Path(share_dir) / 'config' / 'world_model.yaml')


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


class GoToAreaNode(Node):
    def __init__(self):
        super().__init__('go_to_area')
        self.declare_parameter('world_model_path', default_world_model_path())
        self.declare_parameter('target', '')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('server_timeout_sec', 10.0)
        self.declare_parameter('dry_run', False)

        action_name = self.get_parameter('action_name').value
        self._client = ActionClient(self, NavigateToPose, action_name)

    def load_world_model(self) -> dict:
        path = Path(self.get_parameter('world_model_path').value).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'world_model_path does not exist: {path}')
        with path.open('r', encoding='utf-8') as file:
            return yaml.safe_load(file) or {}

    def build_goal(self, world_model: dict, target: str) -> NavigateToPose.Goal:
        areas = world_model.get('areas', {})
        if target not in areas:
            known = ', '.join(sorted(areas))
            raise ValueError(f"Unknown target area '{target}'. Known areas: {known}")

        area = areas[target]
        center = area.get('center')
        if not center or len(center) < 2:
            raise ValueError(f"Area '{target}' is missing center: [x, y]")

        yaw = float(self.get_parameter('yaw').value)
        quat = yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('goal_frame').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(center[0])
        pose.pose.position.y = float(center[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat['x']
        pose.pose.orientation.y = quat['y']
        pose.pose.orientation.z = quat['z']
        pose.pose.orientation.w = quat['w']

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def run_once(self) -> int:
        target = self.get_parameter('target').value
        if not target:
            world_model = self.load_world_model()
            known = ', '.join(sorted(world_model.get('areas', {})))
            self.get_logger().error(f"Parameter 'target' is required. Known areas: {known}")
            return 2

        world_model = self.load_world_model()
        goal = self.build_goal(world_model, target)
        area = world_model['areas'][target]
        display_name = area.get('display_name', target)
        x = goal.pose.pose.position.x
        y = goal.pose.pose.position.y

        self.get_logger().info(f"Task: go_to_area target={target} ({display_name})")
        self.get_logger().info(f"Resolved goal: frame={goal.pose.header.frame_id} x={x:.3f} y={y:.3f}")

        if bool(self.get_parameter('dry_run').value):
            self.get_logger().info('dry_run=true, not sending Nav2 goal')
            return 0

        timeout = float(self.get_parameter('server_timeout_sec').value)
        self.get_logger().info('Waiting for Nav2 NavigateToPose action server...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f'NavigateToPose action server not available after {timeout:.1f}s')
            return 3

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return 4

        self.get_logger().info('Nav2 accepted the goal')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status_text = STATUS_TEXT.get(result.status, str(result.status))

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded: {target}')
            return 0

        self.get_logger().error(f'Goal failed with status={status_text}')
        return 5


def main(args=None):
    rclpy.init(args=args)
    node = GoToAreaNode()
    try:
        code = node.run_once()
    except Exception as exc:  # noqa: BLE001 - keep prototype errors visible in ROS logs.
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
