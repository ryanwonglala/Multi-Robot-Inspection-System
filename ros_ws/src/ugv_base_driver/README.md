# Namespaced UGV Base Driver (`ugv_base_driver`)

ROS 2 Humble serial driver for the tracked Waveshare UGV used as the
RoboInspect RGB-D verifier/carrier.

> **Status:** commissioning driver. Translation feedback has received an
> initial calibration, but skid-steer rotation remains asymmetric. Do not use
> this package for autonomous Nav2 motion until the chassis is revalidated.

## Package contents

- `ugv_base_driver/ugv_base_node.py` — serial/ROS bridge with command timeout,
  diagnostics, battery state, odometry, and TF.
- `ugv_base_driver/odometry.py` — testable differential-odometry integration.
- `config/ugv_base.yaml` — conservative limits and first-pass calibration.
- `launch/ugv_base.launch.py` — namespaced launch entry point.
- `test/test_odometry.py` — offline odometry tests.

## ROS contract

With the default `namespace:=ugv`, the node uses:

| Interface | Name |
| --- | --- |
| Velocity command | `/ugv/cmd_vel` |
| Odometry | `/ugv/odom` |
| Battery | `/ugv/battery_state` |
| Diagnostics | `/ugv/diagnostics` |
| TF | `ugv/odom -> ugv/base_link` |

The driver deliberately avoids root `/cmd_vel`, root `/odom`, and unprefixed
TF frames so it can share a ROS domain with TurtleBot3 without name collisions.

## Hardware and defaults

- Serial device: `/dev/ttyTHS1`
- Baud rate: `115200`
- Maximum linear command: `0.15 m/s`
- Maximum angular command: `0.80 rad/s`
- Command deadman timeout: `0.40 s`
- Feedback timeout: `2.0 s`
- First-pass odometry: `meters_per_tick=0.01`, `track_width=0.41`

Override the serial device at launch time when necessary:

```bash
ros2 launch ugv_base_driver ugv_base.launch.py port:=/dev/<device>
```

## Build and test

From the repository root:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_base_driver --symlink-install
source install/setup.bash
colcon test --packages-select ugv_base_driver
colcon test-result --verbose
```

The odometry tests are offline and do not open the serial port.

## Zero-command commissioning check

Raise the chassis on a stable support or otherwise prevent unintended motion.
Start the base without publishing a velocity command:

```bash
ROS_DOMAIN_ID=74 ros2 launch ugv_base_driver ugv_base.launch.py
```

Then confirm that diagnostics arrive and the commanded output remains zero:

```bash
ros2 topic echo --once /ugv/diagnostics
ros2 topic echo --once /ugv/battery_state
```

Do not publish `/ugv/cmd_vel` until the operator has completed the mechanical
inspection and established a clear test area.

## Calibration evidence and open blocker

The superseded driver treated controller distance counters as millimetres and
applied a 90-degree quaternion-only yaw offset. LiDAR registration indicated
that the counters are approximately centimetres, and the yaw offset made
translation and orientation internally inconsistent. The current first-pass
values remove that offset.

Short straight-line observations were:

| Motion | LiDAR | Odometry |
| --- | ---: | ---: |
| Forward pulse | 4.66 cm | 5.00 cm |
| Reverse pulse | 5.97 cm | 6.50 cm |

These are commissioning results, not metrology-grade calibration. Longer runs
are required before tightening covariance.

Tracked skid-steer rotation is strongly surface- and direction-dependent. In
the recorded session, `+1.0 rad/s` produced visible rotation while the
corresponding negative command produced little motion. Investigate track
tension, rubbing, load balance, motor output, and lower-controller mixing
before enabling navigation. The former `0.45 rad/s` Nav2 limit was inside the
observed unreliable region.

## Driver safety behavior

- Starts at zero velocity and continues sending zero until a fresh command is
  received.
- Clamps linear and angular commands to configured limits.
- Returns to zero after `deadman_timeout`.
- Sends zero twice during shutdown before releasing the serial port.
- Reports feedback age, counters, voltage, parse failures, and serial errors.
- Must not run beside another process that opens the same serial device.

Before any motion session, inspect both tracks, compare left/right response in
both directions, record raw counters, and revalidate distance and heading
against an external measurement.

See `../../doc/TROUBLESHOOTING.md` for broader system recovery guidance.
