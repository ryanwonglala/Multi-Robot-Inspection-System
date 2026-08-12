# UGV base driver

Namespaced ROS 2 driver for the tracked Waveshare UGV used as the
RoboInspect RGB-D verifier/carrier.

## Topic and frame contract

With `ugv_base.launch.py namespace:=ugv`, the driver uses:

- command: `/ugv/cmd_vel`
- odometry: `/ugv/odom`
- battery: `/ugv/battery_state`
- diagnostics: `/ugv/diagnostics`
- TF: `ugv/odom -> ugv/base_link`

It deliberately does not publish the root `/odom`, subscribe to root
`/cmd_vel`, or publish an unprefixed `odom -> base_link` transform. Those names
would collide with the TB3 when both robots share one ROS domain.

## First-pass calibration (2026-07-27)

The original Jetson-side `wave_rover_driver` used:

```yaml
meters_per_tick: 0.001
track_width: 0.26
odom_yaw_offset_deg: 90.0
```

LiDAR scan registration showed that the distance counter is approximately
centimetres, not millimetres. The first-pass replacement values are:

```yaml
meters_per_tick: 0.01
track_width: 0.41
```

The 90-degree odometry offset was removed. Applying an orientation offset only
to the published quaternion while integrating translation with the unshifted
yaw made the pose internally inconsistent. Sensor mounting differences must be
represented by static transforms instead.

Short straight validation with this driver:

| Motion | LiDAR | Odometry |
|---|---:|---:|
| forward pulse | 4.66 cm | 5.00 cm |
| reverse pulse | 5.97 cm | 6.50 cm |

These results are adequate for initial commissioning, not a final metrology
calibration. The controller counters are coarse (about 1 cm), so longer runs
will be needed before tightening covariance.

## Known blocker: tracked skid-steer rotation

This chassis is a two-track skid-steer platform, not an ordinary two-wheel
differential robot. An in-place turn must scrub both long rubber contact
patches sideways, producing a large and surface-dependent breakaway threshold.

Observed on the current floor:

- `+0.6 rad/s` for 1 s: little or inconsistent rotation.
- `+1.0 rad/s` for 0.6 s: LiDAR measured about 16.7 degrees and odometry about
  14.0 degrees.
- `-1.0 rad/s` for 0.6 s: almost no physical rotation.

The strong direction asymmetry is not explained by track scrub alone. Possible
causes include unequal track tension/resistance, direction-dependent motor
output, load distribution, or asymmetric mixing in the lower controller.
Do not enable autonomous Nav2 motion until this is isolated. In particular,
the inherited Nav2 limit `max_vel_theta: 0.45` lies inside the observed
unreliable region.

The chassis AP also stopped accepting clients during this session even after a
chassis-only power cycle. The OLED continued to show `AP: UGV`; UART feedback
remained healthy at about 20 Hz and 11.71 V. Treat the web UI as unavailable,
but keep the AP fault in the lower-controller investigation.

## Safety

- The driver starts with zero velocity and repeats zero until a command is
  received.
- Commands are clamped and expire after `deadman_timeout` (0.4 s by default).
- Shutdown sends zero twice before releasing the serial port.
- `diagnostics` reports feedback age, raw counters, voltage and publish counts.
- Never run this driver together with the legacy `wave_rover_driver`; both
  open `/dev/ttyTHS1`.

## Build and zero-command check

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_base_driver
source install/setup.bash

# Starts the base only. Do not publish /ugv/cmd_vel during this check.
ROS_DOMAIN_ID=74 ros2 launch ugv_base_driver ugv_base.launch.py
```

Before the next motion session:

1. Inspect both tracks and side plates for rubbing.
2. Compare left/right track motion off the floor on a stable support, or use a
   lower-controller per-track diagnostic.
3. Record raw left/right counters for positive and negative angular commands.
4. Only after symmetry is restored, determine a usable angular deadband and
   revalidate `track_width` with LiDAR scan registration.

## Provisional physical start marker

On 2026-07-27 the UGV was manually driven from the TB3 map origin to a separate
parking location, then its centre and heading were marked on the floor. The
driver was stopped immediately afterward and `/dev/ttyTHS1` was released.

The final wheel-odometry reading, relative to the shared origin, was:

```yaml
x: 1.607
y: -0.009
yaw: -1.37
```

This is only an AMCL search seed. The vehicle visibly wandered during the
manual drive, and the estimate lies implausibly close to the map's east bound,
so it must not be copied into the semantic world model as `ugv_start`.

At the next session, keep the UGV on the floor marker, start the base at zero
speed plus LiDAR and localization, seed AMCL near the provisional pose, and
average the converged `map -> ugv/base_link` transform. That converged pose,
followed by a cold reinitialization test, becomes the formal UGV start.
