# ROS Workspace

ROS 2 Jazzy workspace for RoboInspect simulation and task-layer development.

This workspace contains the current ROS/Gazebo/Nav2 prototype for the multi-robot inspection project. At this stage, the implemented system focuses on a single TurtleBot4 in simulation, semantic-area navigation, route-based inspection, evidence capture, report generation, and a unified task GUI.

The repository root README describes the overall project. This README focuses only on the ROS workspace under `ros_ws/`.

## Current Status

Current working baseline:

```text
single TurtleBot4 + Gazebo map + Nav2 + semantic areas + inspection route + report + task GUI
```

Latest prototype package:

```text
experiments/task_layer_v020/
```

The v0.2 prototype is considered sufficient as the current single-robot simulation baseline.

## Workspace Layout

```text
src/sim/                         Gazebo map world and simulation launch package
maps/                            saved occupancy maps for localization/Nav2
doc/                             dated project logs, debug notes, plans, and commands
experiments/task_layer_v010/      v0.1 fixed-coordinate semantic navigation archive
experiments/task_layer_v020/      v0.2 route inspection, reports, scene spawning, task GUI
experiments/task_layer_dryrun/    early dry-run task-layer prototype
```

Generated build/runtime directories are not part of the source tree:

```text
build/
install/
log/
```

## Main v0.2 Features

`experiments/task_layer_v020` includes:

- one-command bringup for Gazebo, localization, Nav2, RViz, and automatic initial pose
- semantic world model in `config/world_model.yaml`
- aligned occupancy map support using `maps/base_aligned_20260602.yaml`
- Nav2-based semantic area navigation
- custom inspection routes by semantic area name
- adaptive inspection candidate poses based on area bounds
- four-direction camera evidence capture after reaching an inspection pose
- Nav failure evidence capture
- return-home behavior to a safe charging-station standoff pose
- concise `report.yaml` plus detailed `details.yaml`
- Gazebo model spawning for test obstacles and scene setup
- unified `task_gui` with `Navigate`, `Inspect`, and `Scene` tabs

Not included in v0.2:

- multi-robot assignment
- YOLO or visual object recognition
- true anomaly classification
- precision docking
- real hardware integration
- robotic arm control

## Build

From the repository root:

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select task_layer_v020
source install/setup.bash
```

For a full workspace build:

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Launch v0.2 Simulation

Start Gazebo, TurtleBot4, localization, Nav2, RViz, and automatic initial pose:

```bash
cd ros_ws
source install/setup.bash
ros2 launch task_layer_v020 bringup_nav.launch.py
```

Open the unified task GUI in another terminal:

```bash
cd ros_ws
source install/setup.bash
ros2 launch task_layer_v020 task_gui.launch.py
```

The GUI has three tabs:

```text
Navigate    send Nav2 goals to semantic areas
Inspect     build and execute inspection routes
Scene       spawn test models/obstacles into Gazebo
```

## CLI Examples

Navigate to a semantic area:

```bash
ros2 run task_layer_v020 go_to_area --ros-args \
  -p use_sim_time:=true \
  -p target:=north_hall
```

Run an inspection route:

```bash
ros2 run task_layer_v020 inspect_area --ros-args \
  -p use_sim_time:=true \
  -p route:=prep_room,lab_room,north_hall \
  -p max_candidate_attempts_per_area:=2 \
  -p candidate_spread_ratio:=0.35 \
  -p return_home:=true
```

Dry-run inspection without moving the robot:

```bash
ros2 run task_layer_v020 inspect_area --ros-args \
  -p target:=north_hall \
  -p dry_run:=true
```

Spawn a test obstacle by semantic area:

```bash
ros2 run task_layer_v020 spawn_model --ros-args \
  -p model:=small_box_obstacle \
  -p area:=north_hall \
  -p placement:=random \
  -p random_margin:=0.2
```

## Reports

Inspection reports are generated under:

```text
experiments/task_layer_v020/reports/
```

Each inspection run creates a folder containing:

```text
report.yaml       concise human-facing status/evidence report
details.yaml      full technical details for debugging
NN_area_name/     captured camera evidence images
```

Generated report contents are ignored by git. Only `reports/.gitignore` is tracked.

## Important Notes

- The map and Gazebo world are currently aligned around the v0.2 start pose and saved map workflow.
- `task_layer_v020` uses `config/nav2_inspection.yaml`, which keeps TurtleBot4 Nav2 behavior while tightening final yaw tolerance for inspection photos.
- Return-home is a Nav2 goal to a safe standoff point near the charger, not precision docking.
- LiDAR-based anomaly detection was tested and removed from v0.2 because results were unstable. Reports currently record execution status and evidence only.

## Development Direction

The next major step should be a cleaner v0.3 task layer rather than adding more ad-hoc logic to v0.2.

Suggested v0.3 direction:

- explicit skill interfaces such as `NavigateToArea`, `InspectAreaRoute`, `CaptureEvidence`, and `ReturnHome`
- structured task input files
- task execution state machine
- GUI support for route presets and report browsing
- later multi-robot namespace/assignment support
- later visual recognition module if needed

Before adding a feature, ask:

- Does it support task-driven inspection?
- Does it avoid turning the system into a fixed route script?
- Can it be validated quickly in Gazebo?
- Can it become part of the final demo?
