# Physical TurtleBot3 Interface (`real`)

ROS 2 Humble package for laptop-side operation of the physical TurtleBot3
Burger. It provides visualization, mapping, Nav2 configuration, camera and
AprilTag utilities, guarded inspection workflows, network recovery helpers,
and read-only health checks.

> **Status:** field-tested prototype. Physical runs remain operator-supervised;
> validate battery, localization, clearances, and the emergency-stop procedure
> before enabling motion.

## Package boundary

`real` supplies the physical-robot interface while reusing the inspection
logic in `task_layer`. It does not start the robot-side TurtleBot3 drivers and
does not contain public site maps, semantic world models, camera baselines, or
private network profiles.

## Tracked contents

- `launch/` — laptop-side RViz, Cartographer mapping, and Nav2 launch files.
- `config/nav2_real.yaml` — reusable physical-robot Nav2 tuning.
- `config/99-realsense-d436.rules` — optional RealSense udev rules.
- `maps/.gitkeep` — destination for local site maps.
- `assets/` — printable camera-calibration checkerboard.
- `rviz/` — physical-robot visualization layouts.
- `scripts/` — health checks, calibration, recovery, inspection, docking, and
  hardware probes.
- `test/` — offline tests for configuration and pure workflow logic.
- `tools/d436_imu_probe.cpp` — low-level RealSense IMU diagnostic source.

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble.
- TurtleBot3 Burger packages, Nav2, Cartographer, RViz, and the dependencies
  declared in `package.xml`.
- Robot-side TurtleBot3 bringup already running for physical visualization,
  mapping, navigation, or inspection.
- A local Fast DDS profile when multicast discovery is unavailable.

## Build and validate

From the repository root:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select task_layer real --symlink-install
source install/setup.bash
colcon test --packages-select real
colcon test-result --verbose
```

The tests do not require a connected robot. They cover configuration parsing,
health-check verdicts, reconnect logic, UGV probes, and guarded workflow state
transitions.

## Local deployment assets

The public repository intentionally omits the following site-specific files:

```text
ros_ws/src/real/maps/<site>.pgm
ros_ws/src/real/maps/<site>.yaml
ros_ws/src/real/config/world_model_<site>.yaml
camera baselines, calibration captures, and network peer profiles
```

Create or restore them locally. Pass explicit paths to `nav_real.launch.py`;
its historical defaults are only convenient when local `lab_arena` assets
already exist:

```bash
ros2 launch real nav_real.launch.py \
  map:=/absolute/path/to/site_map.yaml \
  world_model:=/absolute/path/to/world_model.yaml
```

Never publish credentials, device identities, private IP addresses, personal
images, or site evidence with these assets.

## Environment and connectivity

Every laptop terminal that communicates with the robot must load the same ROS
domain and DDS configuration:

```bash
source ros_ws/src/real/scripts/env_real.sh
```

The supplied script uses ROS domain `2` and expects
`~/fastdds_laptop.xml`. Its repository-location fallback assumes the checkout
is at `~/roboinspec_ws`; if cloned elsewhere, source the workspace setup file
yourself after sourcing ROS Humble.

Phone-hotspot addresses can change between sessions. Preview discovery and
profile changes before applying them:

```bash
python3 ros_ws/src/real/scripts/tb3_hotspot_reconnect.py --dry-run
python3 ros_ws/src/real/scripts/tb3_hotspot_reconnect.py --restart-robot
```

For a TB3 and Jetson on one ROS graph, use
`scripts/fleet_hotspot_reconnect.py` and `scripts/env_fleet.sh`. Keep robot
topics and TF frames namespaced; do not run a root-topic UGV stack alongside
the root-topic TB3 stack.

## Read-only health check

`tb3_healthcheck.py` never publishes `cmd_vel` or sends a Nav2 goal.

```bash
source ros_ws/src/real/scripts/env_real.sh

ros2 run real tb3_healthcheck.py --phase env
ros2 run real tb3_healthcheck.py --phase base
ros2 run real tb3_healthcheck.py --phase nav
```

- `env` checks the host environment and Fast DDS XML.
- `base` additionally checks reachability, sensor topics, rates, TF, battery,
  and camera health before Nav2 starts.
- `nav` additionally checks localization, the map, and Nav2 action-server
  availability without sending a goal.

Exit codes are `0` for ready/skipped, `1` for warnings, `2` for blockers,
`3` for a tool error, and `130` when interrupted. Use `--json` for
machine-readable output.

## Common operating flows

View robot topics in RViz:

```bash
ros2 launch real view_real.launch.py
```

Create a local map:

```bash
ros2 launch real mapping_real.launch.py
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/ros_ws/src/real/maps/<site>"
```

Validate the final workflow configuration without moving the robot:

```bash
ros2 run real run_full_workflow.py --check-only
```

Run the guarded workflow only after the health check and operator safety
review:

```bash
ros2 run real run_full_workflow.py \
  --enable-motion --start-at-home --use-rviz
```

The workflow performs Home → VP1/VP2/VP3 inspection → report → operator load
confirmation → AprilTag-assisted VP4 docking. It waits indefinitely at the
handoff gate; press Enter in the same terminal or explicitly call:

```bash
ros2 service call /inspection_workflow/continue_to_dock \
  std_srvs/srv/Trigger '{}'
```

The request is rejected unless inspection is complete and battery, odometry,
and handoff-position checks pass.

## Safety boundaries

- Run `--check-only` and the appropriate health-check phase before motion.
- Use a freshly charged battery; guarded docking can abort on stale odometry
  or low power.
- Recalibrate camera and map assets after changing the site, camera, or robot
  geometry.
- Restart all ROS participants after changing DDS peer profiles.
- Treat AprilTag docking and the final unloading approach as supervised
  prototype behavior, not unattended production autonomy.

See `../../doc/COMMANDS.md` for the broader command reference and
`../../doc/TROUBLESHOOTING.md` for known failure modes and recovery steps.
