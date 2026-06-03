# Task Layer ROS2 Experiment v0.2 Scaffold (task_layer_v020)

Version `v0.2` starts from the validated v0.1 task-layer loop for RoboInspect:

```text
single robot -> semantic area name/number -> fixed map coordinate -> Nav2 NavigateToPose -> result status
```

This package intentionally lives under `experiments/` while the task-layer design is still fluid. Mature ROS2 packages can move to `src/` later.

## Scope

Carried over from v0.1:

- one-command bringup for Gazebo, TurtleBot4, localization, Nav2, and RViz
- aligned map support using `maps/base_aligned_20260602.yaml`
- automatic AMCL initial pose publishing from the same `x/y/z/yaw` used for Gazebo spawn
- semantic world model in `config/world_model.yaml`
- GUI target selection by area number, area key, or display name
- CLI target command via `go_to_area`
- single-robot fixed-coordinate navigation

Planned for v0.2:

- runtime spawning of custom Gazebo models for repeatable test scenes
- custom inspection routes by semantic area name
- fixed inspection behavior after arrival: four-direction scan and camera evidence
- continue-on-fail custom route execution with return-home behavior
- simple structured inspection reports for checked/unchecked/Nav fail/return-home status
- GUI/CLI flow for navigation plus inspection

Not included yet:

- dynamic task decomposition
- multi-robot namespaces or assignment
- visual detection, such as YOLO
- LiDAR-based anomaly judgment
- feedback-driven replanning

## Build

```bash
cd /home/ryan/tb4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select task_layer_v020
source install/setup.bash
```

## Package Layout

```text
launch/
  bringup_nav.launch.py      # Gazebo + localization + Nav2 + RViz
  task_gui.launch.py         # unified Navigate / Inspect / Scene GUI
  scene_builder.launch.py    # legacy model placement GUI entry

task_layer_v020/
  go_to_area_node.py         # CLI Nav2 target sender
  inspection_runner.py       # CLI area inspection flow
  task_gui_node.py           # unified task GUI
  set_initial_pose_node.py   # automatic AMCL initial pose publisher
  scan_analyzer.py           # LaserScan summary helpers
  report_writer.py           # YAML inspection report writer
  model_spawner.py           # reusable model spawning backend and CLI
  scene_builder_gui_node.py  # legacy GUI for model/area placement
```

## Bringup

Start Gazebo, localization, Nav2, RViz, and automatic initial pose publishing:

```bash
ros2 launch task_layer_v020 bringup_nav.launch.py
```

Defaults:

```text
map: /home/ryan/tb4_ws/maps/base_aligned_20260602.yaml
start: x=-4.8, y=-3.5, yaw=-1.5708
use_sim_time: true
nav2 params: config/nav2_inspection.yaml
```

The same `x/y/z/yaw` values are used for both Gazebo spawn and AMCL `/initialpose`, because the aligned map was rebuilt from the same coordinate basis. v0.2 uses `config/nav2_inspection.yaml` by default, which keeps TurtleBot4 Nav2 behavior but tightens `yaw_goal_tolerance` for more consistent four-direction inspection photos.

## Task GUI

After `bringup_nav.launch.py` is running, open the unified task GUI:

```bash
ros2 launch task_layer_v020 task_gui.launch.py
```

The GUI loads `config/world_model.yaml` and provides three tabs:

```text
Navigate: select/type an area and send a Nav2 NavigateToPose goal
Inspect: build a semantic-area route and start inspect_area backend
Scene: spawn built-in models by area center/random/manual coordinates
```

The Navigate tab accepts area number, area key, or display name. Double-click an area
or press `Go` to send a Nav2 goal. Use `Cancel` to cancel the current goal.

The Inspect tab builds a comma-separated route and launches the existing `inspect_area`
backend, so CLI and GUI inspection behavior stay aligned.

The Scene tab reads `models/*.sdf` and `config/world_model.yaml`, supports area center,
random-in-area, and manual placement, and automatically assigns fresh entity names for
repeated spawns. `scene_builder.launch.py` remains available as a legacy focused scene
builder entry while the unified GUI matures.

## Spawn Test Models

CLI spawning is still available for repeatable scripted tests:

CLI spawning is still available for repeatable scripted tests:

```bash
ros2 run task_layer_v020 spawn_model --ros-args \
  -p model:=small_box_obstacle \
  -p x:=-3.0 \
  -p y:=-2.0 \
  -p z:=0.25
```

If `name` is omitted, `spawn_model` generates a unique entity name. Pass `name:=...`
only when you intentionally want a fixed Gazebo entity name.

To use an external SDF file:

```bash
ros2 run task_layer_v020 spawn_model --ros-args \
  -p file:=/absolute/path/to/model.sdf \
  -p name:=custom_obstacle \
  -p x:=-3.0 \
  -p y:=-2.0
```

Built-in models are installed from `models/*.sdf`. The first test model is `small_box_obstacle`, a static red box with collision geometry so LiDAR and Nav2 can see it.

You can also place by semantic area without typing coordinates:

```bash
ros2 run task_layer_v020 spawn_model --ros-args \
  -p model:=small_box_obstacle \
  -p area:=mother_base \
  -p placement:=random \
  -p random_margin:=0.2
```

## CLI Navigation

Send a target directly:

```bash
ros2 run task_layer_v020 go_to_area --ros-args \
  -p use_sim_time:=true \
  -p target:=main_corridor
```

Try another area:

```bash
ros2 run task_layer_v020 go_to_area --ros-args \
  -p use_sim_time:=true \
  -p target:=central_hall
```

Dry-run lookup only:

```bash
ros2 run task_layer_v020 go_to_area --ros-args \
  -p target:=lab_room \
  -p dry_run:=true
```

## CLI Inspection

Run a minimal area inspection task:

```bash
ros2 run task_layer_v020 inspect_area --ros-args \
  -p use_sim_time:=true \
  -p target:=mother_base
```

Run a custom route:

```bash
ros2 run task_layer_v020 inspect_area --ros-args \
  -p use_sim_time:=true \
  -p route:=mother_base,lab_room,server_room
```

The runner:

```text
1. resolves each semantic area in the requested route
2. generates candidate inspection poses inside area bounds
3. tries Nav2 goals until one candidate succeeds
4. rotates through fixed scan yaws at the selected pose
5. samples `/scan` after each yaw for sensor-online evidence only
6. saves one camera image after each yaw under reports/images/
7. writes a YAML report under reports/
```

This v0.2 inspection flow does not judge whether an area is abnormal. It only records
whether the area was checked, unchecked, or failed during Nav2 navigation, plus the
photo/scan evidence collected at each scan direction.

Dry-run candidate generation and report writing:

```bash
ros2 run task_layer_v020 inspect_area --ros-args \
  -p route:=mother_base,lab_room \
  -p dry_run:=true
```

Useful parameters:

```text
target:=mother_base
route:=mother_base,lab_room,server_room
candidate_offset:=0.5
candidate_spread_ratio:=0.35
bounds_margin:=0.25
max_candidate_attempts_per_area:=2
capture_nav_fail_evidence:=true
scan_yaws:=[0.0,1.5708,3.1416,-1.5708]
scan_settle_sec:=1.0
image_topic:=/oakd/rgb/preview/image_raw
camera_settle_sec:=1.0
return_home:=true
home_area:=charging_station
return_home_standoff_distance:=0.6
report_dir:=/home/ryan/tb4_ws/experiments/task_layer_v020/reports
```

Each run creates a folder like this:

```text
reports/
  inspection_20260603T081530Z_mother_base_lab_room/
    report.yaml      # concise human-facing report
    details.yaml     # full technical details for debugging
    01_mother_base/
      scan_01_yaw_0.0000.ppm
      scan_02_yaw_1.5708.ppm
    02_lab_room/
      scan_01_yaw_0.0000.ppm
```

`report.yaml` includes route status, per-area status, return-home status, and evidence paths:

```yaml
status: completed
summary:
  requested_count: 3
  checked_count: 3
  failed_count: 0
  return_home_status: succeeded
return_home:
  attempted: true
  target: charging_station
  status: succeeded
areas:
  - area: mother_base
    display_name: Mother Base Bay
    status: checked
    evidence_dir: /home/ryan/tb4_ws/experiments/task_layer_v020/reports/.../01_mother_base
    captured_image_count: 4
    image_paths:
      - /home/ryan/tb4_ws/experiments/task_layer_v020/reports/.../01_mother_base/scan_01_yaw_0.0000.ppm
  - area: lab_room
    status: nav_failed
    reason: candidate_attempt_limit_reached
    nav_fail_image_paths:
      - /home/ryan/tb4_ws/experiments/task_layer_v020/reports/.../02_lab_room/nav_fail_attempt_01.ppm
details_file: /home/ryan/tb4_ws/experiments/task_layer_v020/reports/.../details.yaml
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
colcon build --packages-select task_layer_v020
source install/setup.bash
```
