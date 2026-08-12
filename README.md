# RoboInspect — Multi-Robot Indoor Inspection System

A simulation-to-real multi-robot system for autonomous indoor inspection,
built around ROS 2 Humble. The system tasks robots to patrol an indoor
environment, detect environmental changes from camera imagery, localize the
anomalies on the map, and produce inspection reports. The repository also
contains the physical TurtleBot3 workflow, a Jetson/RealSense safety gate, an
archived custom UGV stack, and the standalone SO-ARM response station.

> **Status (final project snapshot, August 2026):** the simulation inspection
> workflow and standalone SO-ARM sorting pipeline are validated end-to-end.
> The physical TurtleBot3 stack has completed field trials for localization,
> Nav2 transit, AprilTag terminal alignment, inspection, and return/docking.
> Cross-platform handoff remains an operator-supervised integration rather
> than a fully autonomous production deployment.

## Objective

- Autonomous indoor inspection of structured spaces (labs, corridors, storage)
- Vision-based environmental change detection
- Map-level anomaly localization
- Inspection report generation
- (Future) robotic response using a custom mobile robot and SO-ARM

## Workstreams

The project is organized into three parallel workstreams.

### 1 · Simulation & Inspection System  — *core workflow validated in simulation*

Implemented in this repository under `ros_ws/`. Current capabilities:

- **Multi-robot task allocation** — an inspection route is split across the
  fleet, with one inspection process per robot (per-namespace).
- **Autonomous viewpoint navigation** — Nav2 `NavigateToPose` drives each robot
  to per-area inspection viewpoints.
- **Backup viewpoint switching** — viewpoints are costmap-prescreened; when a
  stop is blocked, generated ring candidates serve as fallbacks.
- **Image & LiDAR acquisition** — camera frames and laser scans captured at
  each viewpoint.
- **Vision-based change detection** — each photo is compared against a
  known-clean baseline from the same viewpoint; changed regions become anomaly
  candidates.
- **Map-level anomaly localization** — anomaly ground-contact pixels are
  back-projected through the camera model onto the floor plane to yield map
  coordinates, published as RViz markers.
- **Dashboard & mission reporting** — a Tk GUI (Scene + Inspect tabs) for scene
  setup and live monitoring, a fleet-wide anomaly bus, and generated YAML +
  bilingual (中文/English) Markdown reports with collected evidence.

The final demo layer also includes audio state cues and textured RViz anomaly
markers. Generated baselines and reports stay local because they are tied to a
particular camera, map, and inspection site.

### 2 · Physical Robot Deployment  — *field-tested prototype*

Implemented under `ros_ws/src/real/`, `ros_ws/src/ugv_base_driver/`, and
`jetson_realsense_gate/`:

- TurtleBot3 mapping/localization, camera calibration, Nav2 navigation, health
  checks, inspection workflow, AprilTag visual servoing, and hybrid return.
- Hotspot/Fast DDS recovery helpers for repeatable field setup.
- Namespaced custom-UGV base driver and a Jetson RealSense safety/load gate.
- Machine-readable run reports and guarded motion limits for physical trials.

The final field evidence includes repeated runs that passed all automatic
navigation and AprilTag gates. One low-battery trial stopped during the final
reverse step (`odom_stop`), so docking still requires an operator check and a
healthy-battery preflight. Site maps, camera images, device calibration, and
network runbooks are intentionally kept outside this public repository. The
earlier Wave Rover experiments are preserved in `ugv02_archive/` and are not
the primary production path.

### 3 · SO-ARM Vision Sorting Module  — *autonomous sorting pipeline validated (standalone)*

Implemented in this repository under `so-arm101/` (pure Python + lerobot on
macOS, fully independent from `ros_ws/`). A fixed SO-ARM101 detects
abnormal-colored objects in its work zone, grasps them, and drops them into a
recycling bin — the physical-response station that the TurtleBot3 tray will
feed.

*Completed:* mechanical assembly & servo bring-up · camera-server architecture
(macOS TCC) · reference-frame-diff detection + rule-based classification ·
pixel→joint mapping from a human-taught grid (plane fit) · hover visual
servoing · vertical-posture constraint · contact-stop grasping with load
verification · transport & drop choreography · stress-test tooling with
logged success-rate data · full worklogs and a new-site deployment guide
(`so-arm101/docs/`).

The module remains intentionally independent of ROS. The TurtleBot3-to-arm
handoff and unattended multi-platform operation are documented extension
points, not claims of the final prototype.

## Repository Structure

```text
roboinspec_ws/
├── ros_ws/              # Active simulation & inspection system (ROS 2 Humble)
│   └── src/
│       ├── task_layer/  # Task allocation, inspection runner, change detection,
│       │                #   anomaly localization, reporting, operator GUI
│       ├── sim/         # Gazebo worlds + TurtleBot3 models and launch files
│       ├── real/        # Physical TB3 bringup, navigation and field workflows
│       └── ugv_base_driver/ # Namespaced custom-UGV commissioning driver
├── jetson_realsense_gate/ # Jetson RGB-D safety and load-detection gate
├── so-arm101/           # SO-ARM101 vision sorting subsystem (Python + lerobot,
│                        #   no ROS; see so-arm101/README.md)
├── ugv02_archive/       # Preserved Wave Rover commissioning implementation
├── archive/             # Earlier project versions
├── markers/             # RViz anomaly-marker runtime assets
└── sounds/              # Optional final-demo audio cues
```

## Current Project Status

| Workstream                     | Status                          |
| ------------------------------ | ------------------------------- |
| Simulation & Inspection System | End-to-end validated            |
| Physical TurtleBot3            | Field-tested prototype          |
| Custom UGV / Jetson gate       | Commissioned / archived path    |
| SO-ARM Vision Sorting          | End-to-end validated standalone |
| Full System Integration        | Operator-supervised prototype   |

## Roadmap

**Completed**
- End-to-end inspection workflow in simulation (allocate → navigate → capture →
  detect → localize → report)
- Physical TB3 localization, Nav2 transit, inspection, AprilTag alignment, and
  guarded hybrid return workflow
- Real-robot health checks, hotspot recovery, map/calibration assets, and
  machine-readable run evidence
- Jetson RealSense safety/load gate and archived custom-UGV commissioning stack
- SO-ARM autonomous sorting pipeline: detection → classification → taught-grid
  localization → visual servoing → verified grasp → transport & drop
  (stress-tested standalone; see `so-arm101/`)

**Known limitations / extension points**
- Final physical docking requires operator validation; low battery can trigger
  the guarded odometry-stop abort.
- Camera baselines must be recorded again when the device, map, pose, or site
  changes; they are deliberately excluded from version control.
- TB3 tray docking to SO-ARM and unattended multi-platform orchestration remain
  future integration work.

---

*Final academic-project snapshot · ROS 2 Humble · Nav2 · Gazebo · RViz · AprilTag · RealSense.*
