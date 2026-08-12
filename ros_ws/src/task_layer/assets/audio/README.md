# Demo Audio Cues

Packaged sounds for `task_layer/demo_audio_node.py`. The node is an optional,
read-only sidecar: it observes existing ROS topics and generated reports but
does not publish motion commands, call services, or change inspection logic.

> **Status:** live-demo enhancement. Omitting or stopping this node does not
> affect navigation, inspection, anomaly detection, or reporting.

## Cue mapping

| File | Trigger |
| --- | --- |
| `01_ready.wav` | Every configured robot has a Nav2 action server and an AMCL pose. |
| `02_task_received.wav` | The first configured robot starts moving after readiness. |
| `03_anomaly.wav` | A new event appears on `/anomaly_events`. |
| `04_complete.wav` | A new top-level `mission_report.md` appears below `report_dir`. |

Per-robot `report.md` files do not trigger the completion cue.

## Build and run

From the ROS workspace:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select task_layer --symlink-install
source install/setup.bash
ros2 run task_layer demo_audio_node.py
```

The default player is `paplay`, which supports WAV, OGG, and FLAC. MP3 requires
another player, for example:

```bash
ros2 run task_layer demo_audio_node.py \
  --ros-args -p player_cmd:='mpg123 -q'
```

Useful parameters:

```bash
ros2 run task_layer demo_audio_node.py --ros-args \
  -p robots:="['tb3','arm']" \
  -p task_requires_both:=true \
  -p anomaly_debounce_sec:=4.0 \
  -p report_dir:="$HOME/roboinspec_ws/reports"
```

The audio directory and individual filenames can also be overridden with
`sounds_dir`, `ready_file`, `task_file`, `anomaly_file`, and `complete_file`.

## Packaging

`task_layer/CMakeLists.txt` installs this directory under:

```text
share/task_layer/assets/audio/
```

The node resolves that package-share path at runtime, so it does not depend on
the source-tree location.
