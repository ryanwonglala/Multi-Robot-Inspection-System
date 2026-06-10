# Task Layer ROS2 Experiment v0.1 (task_layer_v010)

Version `v0.1` validates the first task-layer loop for RoboInspect:

```text
single robot -> semantic area name/number -> fixed map coordinate -> Nav2 NavigateToPose -> result status
```

This package intentionally lives under `experiments/` while the task-layer design is still fluid. Mature ROS2 packages can move to `src/` later.

## Scope

Included in v0.1:

- one-command bringup for Gazebo, TurtleBot4, localization, Nav2, and RViz
- aligned map support using `maps/base_aligned_20260602.yaml`
- automatic AMCL initial pose publishing from the same `x/y/z/yaw` used for Gazebo spawn
- semantic world model in `config/world_model.yaml`
- GUI target selection by area number, area key, or display name
- CLI target command via `go_to_area`
- single-robot fixed-coordinate navigation

Not included yet:

- dynamic task decomposition
- multi-robot namespaces or assignment
- inspection behavior after arrival
- feedback-driven replanning

## Build

```bash
cd /home/ryan/tb4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select task_layer_v010
source install/setup.bash
```

## Bringup

Start Gazebo, localization, Nav2, RViz, and automatic initial pose publishing:

```bash
ros2 launch task_layer_v010 bringup_nav.launch.py
```

Defaults:

```text
map: /home/ryan/tb4_ws/maps/base_aligned_20260602.yaml
start: x=-4.8, y=-3.5, yaw=-1.5708
use_sim_time: true
```

The same `x/y/z/yaw` values are used for both Gazebo spawn and AMCL `/initialpose`, because the aligned map was rebuilt from the same coordinate basis.

## GUI Navigation

After `bringup_nav.launch.py` is running, open the task navigation GUI:

```bash
ros2 launch task_layer_v010 nav_gui.launch.py
```

The GUI loads `config/world_model.yaml`, lists all semantic areas, and accepts either:

```text
area number, e.g. 2
area key, e.g. main_corridor
area display name, e.g. Main Corridor
```

Double-click an area or press `Go` to send a Nav2 `NavigateToPose` goal. Use `Cancel` to cancel the current goal.

## CLI Navigation

Send a target directly:

```bash
ros2 run task_layer_v010 go_to_area --ros-args \
  -p use_sim_time:=true \
  -p target:=main_corridor
```

Try another area:

```bash
ros2 run task_layer_v010 go_to_area --ros-args \
  -p use_sim_time:=true \
  -p target:=central_hall
```

Dry-run lookup only:

```bash
ros2 run task_layer_v010 go_to_area --ros-args \
  -p target:=lab_room \
  -p dry_run:=true
```

## Current Targets

Targets come from `config/world_model.yaml` and include:

```text
charging_station
mother_base
main_corridor
central_hall
lab_room
prep_room
north_hall
east_hall
server_room
restricted_zone
storage_area
utility_area
narrow_passage
```

## Notes

If `config/world_model.yaml` changes, rebuild the package before using installed launch/run commands:

```bash
colcon build --packages-select task_layer_v010
source install/setup.bash
```
