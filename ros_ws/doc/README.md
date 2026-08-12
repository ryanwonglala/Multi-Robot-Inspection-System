# Project Documentation

Public, reusable documentation for the RoboInspect ROS 2 workspace.

> **Scope:** generated logs, rosbags, calibration captures, private network
> details, and site-specific evidence remain local. Reusable conclusions are
> consolidated here.

## Documents

- `COMMANDS.md` — build, simulation, navigation, inspection, and validation
  command reference.
- `TROUBLESHOOTING.md` — consolidated symptoms, causes, fixes, limitations,
  and recovery practices from the project lifecycle.

Module-specific operating instructions live beside their code:

- `../src/real/README.md` — physical TurtleBot3 deployment and safety scope.
- `../src/ugv_base_driver/README.md` — namespaced Waveshare UGV base driver.
- `../../jetson_realsense_gate/README.md` — Jetson/RealSense handoff gate.
- `../../so-arm101/README.md` — standalone SO-ARM101 sorting subsystem.

## Local field-record convention

When keeping non-public records, use date-based directories outside the final
tracked tree:

```text
doc/YYYYMMDD/log/
doc/YYYYMMDD/cmd/
doc/YYYYMMDD/dbg/
doc/YYYYMMDD/plan/
```

Use timestamped names such as `YYYYMMDD_HHMMSS.md`. Do not commit credentials,
device identities, private IP addresses, personal images, raw rosbags, or
machine-specific calibration captures.

## Maintenance rules

1. Update `COMMANDS.md` when a supported command or package entry point
   changes.
2. Move durable debugging knowledge into `TROUBLESHOOTING.md`; do not add a
   second troubleshooting archive.
3. Keep paths relative to the repository or ROS package share directory.
4. Mark hardware motion commands clearly and keep diagnostic-only commands
   separate.
5. Check all links and paths before publishing the final Main branch.
