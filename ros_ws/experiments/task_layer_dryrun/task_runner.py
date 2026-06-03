#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def find_config_dir() -> Path:
    return Path(__file__).resolve().parent


def load_yaml(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
    return data or {}


def pose_text(location: dict) -> str:
    x = float(location['x'])
    y = float(location['y'])
    yaw = float(location['yaw'])
    return f"map x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"


def make_plan(task: dict, world_model: dict) -> list[dict]:
    task_type = task.get('type')
    target = task.get('target')

    if task_type != 'inspect_area':
        raise ValueError(f"Unsupported task type: {task_type}")
    if not target:
        raise ValueError('Task is missing target')

    areas = world_model['areas']
    locations = world_model['locations']

    if target in areas:
        area = areas[target]
        profile = area.get('profile', 'basic_360_scan')
        steps = []

        entry_pose = area.get('entry_pose')
        if entry_pose:
            steps.append({
                'skill': 'NavigateTo',
                'target': entry_pose,
                'area': target,
                'needs': ['navigate'],
            })

        for pose_name in area.get('inspect_poses', []):
            if pose_name != entry_pose:
                steps.append({
                    'skill': 'NavigateTo',
                    'target': pose_name,
                    'area': target,
                    'needs': ['navigate'],
                })

            steps.append({
                'skill': 'InspectArea',
                'target': target,
                'pose': pose_name,
                'profile': profile,
                'needs': ['inspect_area'],
            })

            if task.get('require_image', False):
                steps.append({
                    'skill': 'CaptureImage',
                    'target': target,
                    'pose': pose_name,
                    'needs': ['capture_image'],
                })

        steps.append({'skill': 'ReturnHome', 'target': 'home', 'needs': ['navigate']})
        return steps

    if target in locations:
        steps = [
            {'skill': 'NavigateTo', 'target': target, 'needs': ['navigate']},
            {'skill': 'InspectArea', 'target': target, 'pose': target, 'profile': 'basic_360_scan', 'needs': ['inspect_area']},
        ]
        if task.get('require_image', False):
            steps.append({'skill': 'CaptureImage', 'target': target, 'pose': target, 'needs': ['capture_image']})
        steps.append({'skill': 'ReturnHome', 'target': 'home', 'needs': ['navigate']})
        return steps

    raise ValueError(f"Target '{target}' is not defined as an area or location in world_model.yaml")


def required_capabilities(plan: list[dict]) -> set[str]:
    needed = set()
    for step in plan:
        needed.update(step['needs'])
    return needed


def choose_robot(robots: dict, needed: set[str]) -> tuple[str, dict]:
    for robot_name, robot in robots.items():
        capabilities = set(robot.get('capabilities', []))
        if robot.get('status') == 'available' and needed.issubset(capabilities):
            return robot_name, robot

    needed_text = ', '.join(sorted(needed))
    raise RuntimeError(f'No available robot has all required capabilities: {needed_text}')


def check_plan_targets(plan: list[dict], world_model: dict) -> None:
    locations = world_model['locations']
    areas = world_model['areas']
    profiles = world_model['inspection_profiles']

    for step in plan:
        if step['skill'] in {'NavigateTo', 'ReturnHome'} and step['target'] not in locations:
            raise ValueError(f"Location '{step['target']}' is not defined in world_model.yaml")
        if step['skill'] in {'InspectArea', 'CaptureImage'}:
            if step['target'] not in areas and step['target'] not in locations:
                raise ValueError(f"Inspection target '{step['target']}' is not defined in world_model.yaml")
            if step.get('pose') and step['pose'] not in locations:
                raise ValueError(f"Inspection pose '{step['pose']}' is not defined in world_model.yaml")
        if step.get('profile') and step['profile'] not in profiles:
            raise ValueError(f"Profile '{step['profile']}' is not defined in world_model.yaml")


def step_location(step: dict, world_model: dict) -> dict | None:
    locations = world_model['locations']
    pose_name = step.get('pose') or step.get('target')
    return locations.get(pose_name)


def print_plan(task: dict, plan: list[dict], robot_name: str, world_model: dict) -> None:
    target = task['target']
    area = world_model['areas'].get(target)
    target_text = area.get('display_name', target) if area else target

    print('Task-driven inspection dry run')
    print('--------------------------------')
    print(f"Task type: {task['type']}")
    print(f"Target: {target_text} ({target})")
    print(f"Require image: {bool(task.get('require_image', False))}")
    print(f"Selected robot: {robot_name}")
    print('')
    print('Generated skill sequence:')

    for index, step in enumerate(plan, start=1):
        location = step_location(step, world_model)
        pose = f" at {step.get('pose', step['target'])}" if location else ''
        profile = f" profile={step['profile']}" if step.get('profile') else ''
        pose_detail = f" -> {pose_text(location)}" if location else ''
        print(f"{index}. {step['skill']}({step['target']}){pose}{profile}{pose_detail}")

    print('')
    print('Execution mode: dry-run only. No robot command was sent.')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Run a structured inspection task.')
    config_dir = find_config_dir()
    parser.add_argument('--task', default=str(config_dir / 'tasks' / 'inspect_lab_room.yaml'))
    parser.add_argument('--robots', default=str(config_dir / 'robots.yaml'))
    parser.add_argument('--world-model', default=str(config_dir / 'world_model.yaml'))
    parser.add_argument('--dry-run', action='store_true', default=True)
    args, _unknown_ros_args = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()

    task = load_yaml(Path(args.task))['task']
    robots = load_yaml(Path(args.robots))['robots']
    world_model = load_yaml(Path(args.world_model))

    plan = make_plan(task, world_model)
    check_plan_targets(plan, world_model)
    needed = required_capabilities(plan)
    robot_name, _robot = choose_robot(robots, needed)

    print_plan(task, plan, robot_name, world_model)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
