# RoboInspect ROS 2 Workspace

Core ROS 2 Humble workspace for the RoboInspect multi-robot inspection system.
It contains the simulation stack, shared inspection/task layer, physical
TurtleBot3 interface, and namespaced Waveshare UGV base driver.

> **Status:** the simulation workflow is validated end to end. Physical-robot
> packages are field-tested prototypes and require operator supervision.

## Workspace layout

```text
ros_ws/
├── doc/                  # Public commands and troubleshooting references
└── src/
    ├── sim/              # Gazebo worlds, robot models, and simulation launch
    ├── task_layer/       # Allocation, navigation, inspection, detection,
    │                     # reporting, GUI, demo audio, and RViz markers
    ├── real/             # Physical TurtleBot3 deployment and guarded workflows
    └── ugv_base_driver/  # Namespaced Waveshare UGV serial/odometry driver
```

Generated `build/`, `install/`, `log/`, reports, local maps, baselines,
and Python caches are excluded from version control.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot3, Nav2, Gazebo Classic, RViz, and Cartographer packages required by
  the selected workflow
- Python dependencies declared by each ROS package
- A TurtleBot3 overlay workspace when `turtlebot3_gazebo` is not installed in
  the system ROS environment

Physical deployments have additional hardware and network requirements. Read
`src/real/README.md` and `src/ugv_base_driver/README.md` before connecting
hardware.

## Build

From the repository root:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash

# Optional TurtleBot3 overlay:
source ~/tb3_ws/install/setup.bash

colcon build --base-paths src --symlink-install
source install/setup.bash
```

Build only the main project packages when iterating:

```bash
colcon build --base-paths src   --packages-select sim task_layer real ugv_base_driver   --symlink-install
```

Every new terminal must source ROS Humble, any required TurtleBot3 overlay, and
this workspace's `install/setup.bash`.

## Run the simulation

Start the complete two-robot simulation, Nav2 stacks, RViz, and initial poses:

```bash
ros2 launch task_layer multi_nav.launch.py
```

Start only the two-robot Gazebo simulation:

```bash
ros2 launch sim multi_sim.launch.py
```

Start the operator GUI after the navigation stack is ready:

```bash
ros2 launch task_layer task_gui.launch.py
```

For the supported single-robot and manual inspection commands, see
`doc/COMMANDS.md`.

## Run on physical hardware

Physical workflows do not use the simulation launch files. Start with the
module documentation and a read-only preflight:

```bash
source src/real/scripts/env_real.sh
ros2 run real tb3_healthcheck.py --phase env
ros2 run real tb3_healthcheck.py --phase base
```

Site maps, semantic world models, camera baselines, calibration captures, and
private DDS/network profiles are intentionally not shipped in the public
repository. Supply explicit local paths as described in
`src/real/README.md`.

The UGV driver is a commissioning component with an unresolved skid-steer
rotation asymmetry. Do not enable autonomous UGV navigation until the
mechanical and odometry checks in `src/ugv_base_driver/README.md` pass.

## Validate

Run the project test suites from `ros_ws/` after building:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m pytest -q   src/task_layer/test   src/real/test   src/ugv_base_driver/test
```

The final reviewed snapshot passes 141 tests. Hardware-free tests do not prove
that a robot, camera, serial device, network profile, or site calibration is
ready for motion.

## Data and publication policy

Keep these local:

- generated build/install/log directories;
- inspection reports and camera baselines;
- maps and site-specific world models;
- rosbags and calibration captures;
- credentials, device identities, private IP addresses, and personal images.

Commit reusable configuration, source code, tests, and consolidated
troubleshooting knowledge. Historical experiments remain in Git history rather
than duplicated archive directories.

## Documentation

- `doc/COMMANDS.md` — common build and operation commands.
- `doc/TROUBLESHOOTING.md` — failure modes and recovery procedures.
- `src/real/README.md` — physical TurtleBot3 workflow and safety boundary.
- `src/task_layer/assets/audio/README.md` — optional demo audio sidecar.
- `src/ugv_base_driver/README.md` — UGV driver contract and commissioning.
