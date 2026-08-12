# real — physical TB3 interface layer

Laptop-side bringup, RViz views, Nav2/Cartographer configs, and diagnostics
for the physical TurtleBot3 Burger. `task_layer` is untouched by this
package: `real` only swaps in the real map / world_model / nav2 params in
place of their simulated counterparts.

## Layout

- `scripts/env_real.sh` — source this in every terminal that talks to the
  robot. Sets `ROS_DOMAIN_ID`, the FastDDS unicast profile, etc.
- `scripts/tb3_robot_start.sh` — deployed to the robot (`scp` + `bash`),
  starts `turtlebot3_bringup` + the camera node.
- `scripts/tb3_healthcheck.py` — read-only preflight check (see below).
- `launch/view_real.launch.py`, `mapping_real.launch.py`, `nav_real.launch.py`
  — RViz-only view, Cartographer SLAM, and Nav2 point-to-point respectively.
- `config/nav2_real.yaml`, `config/world_model_real.yaml` — real-site tuning.
- `maps/lab_arena.{pgm,yaml}` — current map (`maps/old_*` are dated backups).

## Complete inspection-to-unloading workflow

`run_full_workflow.py` runs the real demonstration as one guarded state
machine: Home → VP1/VP2/VP3 six-direction inspection → consolidated anomaly
report → indefinite operator loading wait at VP3 → hybrid AprilTag-assisted
VP4 docking. It never crosses the VP3 gate on a timer and never triggers the
mechanical arm. The operator must explicitly acknowledge that the cubes are
on the rear tray and everyone has left the arena.

```bash
source ros_ws/src/real/scripts/env_real.sh

# Configuration/report validation only; never moves the robot.
ros2 run real run_full_workflow.py --check-only

# Real run, with the custom anomaly-marker RViz view.
ros2 run real run_full_workflow.py \
  --enable-motion --start-at-home --use-rviz
```

After the VP1–VP3 report is printed, the program remains alive and publishes
its latched JSON state on `/inspection_workflow/state`. In a normal interactive
terminal, load the tray, leave the arena, then press **Enter in that same
terminal**. No second command is required.

For remote or automated operation, the same gate can instead be acknowledged
from another correctly sourced terminal:

```bash
ros2 service call /inspection_workflow/continue_to_dock \
  std_srvs/srv/Trigger '{}'
```

The same gate also accepts a one-shot `std_msgs/msg/Empty` message on
`/inspection_workflow/continue`, but the service is preferred because it
returns an explicit accepted/rejected reason. A request is rejected unless
the inspection is complete, battery and odometry are fresh, and TB3 remains
within the configured VP3 handoff radius. Reports are linked under
`reports/full_workflow/workflow_YYYYMMDD_HHMMSS/`; the original detailed
inspection evidence remains under `reports/red_cube_stress/`.

## Preflight healthcheck

`tb3_healthcheck.py` is a **read-only** diagnostic: it never publishes
`cmd_vel`, never sends a Nav2 goal (the action-server check only calls
`wait_for_server()`), and never writes anything to the robot.

### Phases

Checks are grouped into three cumulative phases, because the things Nav2
*provides* cannot be preconditions for launching Nav2:

```bash
source ros_ws/src/real/scripts/env_real.sh

ros2 run real tb3_healthcheck.py                 # --phase base (default)
ros2 run real tb3_healthcheck.py --phase nav     # after nav_real.launch.py is up
ros2 run real tb3_healthcheck.py --phase env     # offline: env + XML only
```

| Phase | Check | What it proves |
|---|---|---|
| env | C0 host environment | `ROS_DOMAIN_ID` matches the robot's (2), `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`, `FASTRTPS_DEFAULT_PROFILES_FILE`, `TURTLEBOT3_MODEL`, `ros2` on PATH |
| env | C1 FastDDS profile | profile XML parses (namespaced or not), has a `is_default_profile="true"` participant (otherwise FastDDS loads it and never applies it), `maxInitialPeersRange` ≥ 32, `useBuiltinTransports=false`, and the target robot IP is in the initial-peers list |
| base | C2 robot reachability | ICMP ping (3 packets, averaged RTT) + connect-only TCP probe of port 22 — no `ssh` binary, no auth attempt |
| base | C3 base topics + rates | `/scan` `/odom` `/battery_state` present and publishing (`/imu` optional), measured Hz vs nominal, publisher QoS listed, duplicate publishers flagged, and `/scan` content checked for an all-`inf` return field |
| base | C4 odom TF | `odom -> base_footprint` **and** `base_footprint -> base_scan` (without the static one Nav2 cannot place a healthy `/scan` in the costmap) |
| base | C5 battery | voltage: warn < 11.0 V, block < 10.5 V; NaN/0 readings reported as "no reading" instead of passing the guard |
| base | C6 camera | `/image_raw/compressed` (or `/image_raw`) publishing, `/camera_info` `K` non-degenerate — matches the `fx/fy<=0` fallback in `task_layer`'s runner |
| nav | C7 map TF | `map -> odom` and `map -> base_footprint` (AMCL is up and localized) |
| nav | C8 `/map` | a latched `OccupancyGrid` is actually received, with its size/resolution/origin |
| nav | C9 Nav2 action | `navigate_to_pose` reachable via `wait_for_server` only — no goal sent |

**`base` is the right phase before launching Nav2.** It covers exactly what
the robot-side bringup must provide. Checks that are not run in the selected
phase are reported `SKIPPED`, never silently omitted.

### QoS (why a Reliable subscriber lies)

Every monitored sensor topic is subscribed with **BEST_EFFORT/VOLATILE**
(`qos_profile_sensor_data`). This is not cosmetic: `ld08_driver` publishes
`/scan` as `BEST_EFFORT`, and a default (`RELIABLE`) subscriber is
QoS-*incompatible* with it — it connects, receives nothing, and the tool
would report a perfectly healthy lidar as `0 Hz / BLOCKED`. A best-effort
reader matches both best-effort and reliable writers, so it can never
manufacture that false negative. `/map` is the one exception: it is latched,
so C8 subscribes `RELIABLE/TRANSIENT_LOCAL` — a volatile subscriber would
wait forever for a message that was published before it joined.

C3/C6 print each publisher's actual reliability/durability, which is how you
tell a dead topic from a QoS mismatch at a glance.

### Exit codes and flags

| Code | Meaning |
|---|---|
| 0 | all `READY`/`SKIPPED` |
| 1 | at least one `WARN` (use `--exit-zero-on-warn` to ignore) |
| 2 | at least one `BLOCKED` |
| 3 | tool error — bad env, `rclpy` missing, unexpected exception |
| 130 | interrupted |

Code `3` exists so a crashed tool cannot be mistaken for a `WARN` — the first
version let an `ImportError` traceback exit `1`, i.e. "READY with warnings".

```bash
ros2 run real tb3_healthcheck.py --json                  # machine-readable
ros2 run real tb3_healthcheck.py --robot-ip 10.32.55.167 # or $TB3_ROBOT_IP
ros2 run real tb3_healthcheck.py --exit-zero-on-warn     # only blockers matter
ros2 run real tb3_healthcheck.py --rate-window 5 --tf-timeout 10
```

All timeouts are separate flags (`--discovery-wait`, `--rate-window`,
`--battery-timeout`, `--tf-timeout`, `--map-timeout`, `--nav2-timeout`) and
every sampling loop is bounded by the **monotonic clock**, not by an
iteration count — `spin_once()` returns as soon as one callback fires, so
counting iterations both under-measures rates and cuts TF waits short.
Defaults are sized for this ~70–200 ms RTT WiFi link, not for a LAN.

A typical `--phase base` run takes ~20 s.

Robot reachability being `BLOCKED` does not necessarily mean the robot is
off: this campus network never puts the robot in the laptop's ARP table
even when it answers pings, so don't substitute `arp-scan` for the ping
check the tool already does.

### Duplicate publishers

C3/C4/C6 warn when a topic has more than one publisher. This caught a real
fault on 2026-07-27: `tb3_robot_start.sh`'s old `pkill` patterns matched
`robot.launch`/`turtlebot3_ros`/`v4l2_camera` but not `ld08_driver` or
`robot_state_publisher`, so those two launch children were reparented to
`init` and survived. A second `ld08_driver` then fought the first over the
LDS serial port — `/scan` at 7 Hz instead of 9, with corrupted returns — and
a second `robot_state_publisher` spammed `TF_REPEATED_DATA`. Both scripts are
fixed: `tb3_robot_start.sh` now kills those names too and *refuses to start*
if anything survives the cleanup.

### Tests

`colcon test --packages-select real` runs `test/test_tb3_healthcheck.py`,
which covers the pure logic (phase selection, exit codes, FastDDS XML parsing
with and without a namespace, battery NaN handling, rate/scan verdicts, ping
parsing). No `rclpy`, no network, no robot needed.

If you build with `--symlink-install` for iterative script edits, note that
`tb3_healthcheck.py`'s source file in `scripts/` is not itself marked
executable in git (matching the other scripts in this directory, which are
invoked via `bash`/`source` rather than directly) — a plain `colcon build`
(no `--symlink-install`) installs an executable copy, which is what
`ros2 run` needs. If you symlink-install and want to run the script
directly instead of through `ros2 run`, `chmod +x` the source file first.

## Known network facts (see `temp goal.md` for the full runbook)

- Campus WiFi blocks multicast DDS discovery; unicast initial-peers via
  FastDDS XML is mandatory, on both robot and laptop.
- Robot and laptop are not on the same L2 segment: ping succeeds but the
  robot never appears in `arp -a` / `arp-scan`. This is expected.
- After editing either FastDDS profile XML, **restart every ROS process and
  the daemon** on both sides — stale participants stay blind to new peers.

## Reconnecting through a phone hotspot

Phone hotspots may choose a different subnet every time they are enabled.
After the laptop and TB3 both appear in the hotspot client list, run:

```bash
python3 ros_ws/src/real/scripts/tb3_hotspot_reconnect.py --dry-run
python3 ros_ws/src/real/scripts/tb3_hotspot_reconnect.py --restart-robot
```

The script derives the laptop address from its default route, finds the TB3
by its WiFi MAC address, verifies that MAC again over SSH, and updates the
non-loopback peer in both `~/fastdds_laptop.xml` and
`~/fastdds_robot.xml`. Changed files receive timestamped backups. Without
`--restart-robot`, the script only updates the profiles and stops the laptop
ROS daemon; existing robot processes are deliberately left alone.

## Formal TB3 + Jetson operation on a phone hotspot

Formal dual-robot operation uses one ROS graph so a single task-layer process
can see both robots:

- ROS domain `2` on Laptop, TB3, and Jetson.
- Fast DDS unicast on all three machines.
- Laptop initial peers: both TB3 and Jetson.
- TB3 and Jetson initial peer: Laptop.
- Topic names must remain distinct. The namespaced `ugv_base_driver` uses
  `/ugv/...` and can share this graph with the TB3 for communication testing.
  The teammate-validated production UGV stack (`~/ugv02_opensource_node.py`
  and `~/start_ugv02_navigation.sh`) still uses root `/cmd_vel`, `/odom`,
  `/scan`, and `odom -> base_link`; it must not run beside the root-topic TB3
  stack until its topics and TF frames are namespaced.

The isolated Jetson environment (`env_jetson.sh`, domain `74`, CycloneDDS) is
only for stand-alone UGV testing. Do not use it for formal dual-robot work.

Every time the phone hotspot is turned on:

1. Connect Laptop, TB3, and Jetson to the hotspot. Confirm all three appear in
   the phone's client list.
2. On Laptop, discover both robots by their stable WiFi MAC addresses and
   preview the changes:

   ```bash
   python3 ros_ws/src/real/scripts/fleet_hotspot_reconnect.py --dry-run
   ```

3. Apply the new addresses and restart TB3 so every TB3 DDS participant loads
   the new profile:

   ```bash
   python3 ros_ws/src/real/scripts/fleet_hotspot_reconnect.py --restart-tb3
   ```

4. Start the namespaced Jetson communication-test driver only after step 3.
   On Jetson:

   ```bash
   ssh nvidia@<JETSON_IP>
   source ~/env_fleet.sh
   ros2 launch ugv_base_driver ugv_base.launch.py
   ```

   This is not the teammate-validated production Nav stack. Do not substitute
   `~/start_ugv02_navigation.sh` while TB3 is present on the same ROS domain;
   its root topics and TF frames currently collide with TB3.

5. In every formal-operation Laptop terminal:

   ```bash
   source ~/roboinspec_ws/ros_ws/src/real/scripts/env_fleet.sh
   ros2 topic list
   ros2 run real tb3_healthcheck.py --phase base --robot-ip "$TB3_ROBOT_IP"
   ```

   `env_fleet.sh` loads the current `TB3_ROBOT_IP` and `JETSON_IP`
   automatically from `~/.ros/fleet_hotspot.env`, which is generated in
   step 3.

Do not reuse ROS processes from before an IP/profile change. Fast DDS reads the
profile when each participant starts; changing the XML cannot repair an
already-running participant. The fleet reconnect script stops the Laptop ROS
daemon, `--restart-tb3` restarts TB3 bringup/camera, and Jetson drivers are
started afterwards in step 4.

The script verifies identities before writing anything:

- TB3 WiFi MAC: `d4:54:8b:2e:c2:2d`, SSH user `tb`.
- Jetson WiFi MAC: `48:8f:4c:d4:4d:82`, SSH user `nvidia`.

It writes timestamped backups when an existing profile changes. If automatic
discovery is unavailable, pass both addresses explicitly with `--tb3-ip` and
`--jetson-ip`.
