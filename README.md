# RoboInspect — Multi-Robot Indoor Inspection System

A **simulation-first** multi-robot system for autonomous indoor inspection,
built on ROS 2 Humble. The system tasks a fleet of robots to patrol an indoor
environment, detect environmental changes from camera imagery, localize the
anomalies on the map, and produce an inspection report — with a future path
toward physical robotic response using a custom mobile robot and an SO-ARM
manipulator.

> **Status:** Mid-Term milestone. The simulation inspection workflow is
> validated end-to-end; the two hardware platforms are under active
> development and not yet integrated.

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

*Remaining work:* anomaly response logic, dashboard refinement, sim-to-real
deployment.

### 2 · Custom Mobile Robot Platform  — *hardware integration in progress*

*Completed:* chassis assembled · Jetson Nano installed · LiDAR installed ·
depth camera installed · ROS 2 Humble configured · preliminary sensor testing.

*In progress:* ROS 2 communication · SLAM · localization · autonomous
navigation · integration with the inspection workflow.

### 3 · SO-ARM Manipulation Module  — *assembly & calibration in progress*

*Completed:* mechanical assembly · servo installation · controller connection ·
leader–follower teleoperation demonstrated.

*Remaining:* calibration · manipulation accuracy · predefined manipulation
motions · integration with the mobile robot.

## Repository Structure

```text
roboinspec_ws/
├── ros_ws/              # Active simulation & inspection system (ROS 2 Humble)
│   └── src/
│       ├── task_layer/  # Task allocation, inspection runner, change detection,
│       │                #   anomaly localization, reporting, operator GUI
│       └── sim/         # Gazebo worlds + TurtleBot3 robot models, launch files
├── archive/             # Previous archived development versions
├── doc/                 # Project trace records / development logs
└── reports/             # Generated inspection reports
```

## Current Project Status

| Workstream                     | Status                          |
| ------------------------------ | ------------------------------- |
| Simulation & Inspection System | Core workflow validated         |
| Custom Mobile Robot            | Hardware integration in progress|
| SO-ARM                         | Assembly & calibration in progress |
| Full System Integration        | Planned                         |

## Roadmap

**Completed**
- End-to-end inspection workflow in simulation (allocate → navigate → capture →
  detect → localize → report)
- Mobile robot hardware assembly and preliminary sensor testing
- SO-ARM assembly and leader–follower teleoperation

**In Progress**
- Mobile robot ROS 2 communication, SLAM, localization, autonomous navigation
- SO-ARM calibration and manipulation accuracy
- Dashboard refinement and anomaly response logic

**Future Work**
- Sim-to-real deployment of the inspection workflow
- Integration of the SO-ARM with the mobile robot
- Full multi-robot system integration (inspection + physical response)

---

*Simulation built on ROS 2 Humble · Nav2 · Gazebo · RViz. Mid-term milestone — not a final release.*
