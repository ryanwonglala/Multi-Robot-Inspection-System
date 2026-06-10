# Task Layer Dry Run Experiment

This folder is a small experiment area for understanding the task-driven inspection layer before it becomes a ROS2 node.

It does not command Gazebo, Nav2, or TurtleBot4. It reads structured YAML files, expands semantic inspection areas into concrete navigation/inspection poses, selects a robot by capability, and prints the plan.

## Files

```text
task_runner.py                       dry-run task layer script
world_model.yaml                     semantic map from the current map.sdf
robots.yaml                          robot capability/status model
tasks/inspect_lab_room.yaml          example lab inspection task
tasks/inspect_server_room.yaml       example server-room task
tasks/inspect_storage_area.yaml      example storage task
tasks/inspect_utility_area.yaml      example utility task
tasks/inspect_restricted_zone.yaml   example restricted-zone task
```

The previous narrow-passage task was removed because the current `map.sdf` has that former narrow/service passage sealed. It is kept only as `blocked_zones.sealed_service_passage` in `world_model.yaml` so the project history is visible without treating it as a valid inspection target.

## Run

```bash
cd /home/ryan/tb4_ws
python3 experiments/task_layer_dryrun/task_runner.py --task experiments/task_layer_dryrun/tasks/inspect_lab_room.yaml
```

Try another target:

```bash
python3 experiments/task_layer_dryrun/task_runner.py --task experiments/task_layer_dryrun/tasks/inspect_storage_area.yaml
```

Expected idea:

```text
semantic task -> area model -> concrete poses -> skill sequence -> robot selection
```

## Why This Is Separate

The `src/sim` package should stay focused on ROS2/Gazebo simulation glue. This folder is a learning and design sandbox. Once the task logic is clear, the stable parts can move into a real ROS2 task-layer package or node.
