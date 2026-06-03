# TB4 RoboInspect

ROS 2 Jazzy workspace for task-driven TurtleBot4 inspection simulation.

This workspace is the clean project line for RoboInspect. It is isolated from `~/grad_project_ws` and no longer uses the old custom robot stack as the main mapping/control platform.

## Direction

The project goal is to build a task-driven multi-robot inspection framework:

> Predefined robot skills are dynamically composed, assigned, and adjusted based on task goals, robot capabilities, and execution feedback.

In short:

- robot skills are predefined
- full inspection routines are not hard-coded routes
- tasks are decomposed into skill sequences or task graphs
- robots are selected by capability and status
- execution feedback can trigger fallback or reassignment

The detailed development guide is [doc/DESCRIPTION.md](doc/DESCRIPTION.md).

## Current Base

The current base is a working TurtleBot4 simulation stack:

- official TurtleBot4 Gazebo simulation installed under `/opt/ros/jazzy`
- custom inspection facility world in `src/sim/worlds/map.sdf`
- local launch glue in `src/sim/launch/map.launch.py`
- minimal control GUI in `src/sim/scripts/ctl.py`
- baseline SLAM map saved in `maps/base_20260527.yaml` and `maps/base_20260527.pgm`

Local code should wrap or configure the mature TurtleBot4 stack instead of replacing stable upstream behavior.

## Architecture Target

The next software layer should be organized around five levels:

```text
Task Input Layer
Task Planner Layer
Skill Layer
Robot Assignment Layer
ROS2 Execution Layer
```

Initial task input should be structured, not natural language. Example:

```yaml
task:
  type: inspect_area
  target: room_A
  require_image: true
  priority: normal
```

## Near-Term Priorities

1. Single TurtleBot4 accepts a structured task command and executes a basic inspection sequence.
2. Two TurtleBot4 robots run with namespaces, independent navigation, and simple task assignment.
3. Capability models describe what each robot can do.
4. Feedback and fallback handle unavailable robots or failed navigation.
5. Mother-child/scout robot behavior remains an exploration branch, validated first as task logic in simulation.

Avoid spending the next phase on LLM interfaces, complex object detection, reinforcement learning, real manipulator control, full docking, or multi-robot SLAM unless they directly serve the task-driven inspection demo.

## Demo Targets

The minimal demo path should aim for:

- single-robot task decomposition
- two-robot parallel inspection
- reassignment when a robot is unavailable
- optional scout/narrow-area task flow

## Workspace Layout

Short names are intentional.

```text
src/sim        simulation package, launch files, worlds, GUI script
maps           saved SLAM maps
doc            dated project records and direction docs
```

Documentation layout:

```text
doc/YYYYMMDD/log
doc/YYYYMMDD/cmd
doc/YYYYMMDD/dbg
doc/YYYYMMDD/plan
```

## Common Commands

Build:

```bash
cd /home/ryan/tb4_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Launch simulation and SLAM:

```bash
ros2 launch sim map.launch.py
```

Launch control GUI:

```bash
ros2 launch sim ctl.launch.py
```

Save map:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/base_YYYYMMDD
```

After editing `src/sim/worlds/map.sdf` in Gazebo GUI, rebuild before using `ros2 launch`:

```bash
colcon build
source install/setup.bash
```

## Direction Check

Before adding a feature, ask:

- Does it support task-driven inspection?
- Does it prove the system is not a fixed route script?
- Does it improve robot coordination?
- Can it be validated quickly in Gazebo?
- Can it become part of the final demo?
