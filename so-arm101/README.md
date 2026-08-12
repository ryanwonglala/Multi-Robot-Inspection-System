# SO-ARM101 Vision Sorting Subsystem

Standalone vision-guided sorting module for the RoboInspect project. A fixed
SO-ARM101 with a wrist camera detects an abnormal-colour object in a known work
plane, approaches it, verifies the grasp, and drops it into a recycling area.

> **Status:** the standalone sorting pipeline was validated end to end. The
> TurtleBot3-to-arm handoff remains an operator-supervised integration. This
> module does not depend on ROS 2.

## Design

- Monocular wrist camera and a known-height work plane; no depth camera is
  required for grasp localization.
- Reference-frame differencing or non-white segmentation for tray objects,
  followed by solidity and Lab-colour classification.
- Pixel-to-joint mapping fitted from a human-taught grid and refined by hover
  visual servoing.
- Vertical grasp constraint, contact-stop closing, load verification, and
  guarded transport/drop choreography.
- Classical vision by design; failed learned-depth and zero-shot experiments
  are retained in `docs/worklog-2026-07-30.md` as project evidence.

## Contents

- `src/soarm/` — arm, vision, mapping, and camera-client modules.
- `scripts/` — numbered commissioning, calibration, grasping, recovery, and
  analysis tools.
- `config/` — poses, ROI, class rules, offsets, and visual-servo parameters.
- `calibration/` — taught grids, reference images, and stress-test records.
- `calibration/lerobot/` — device-specific LeRobot servo calibration.
- `docs/deployment-new-site.md` — relocation and recalibration procedure.
- `docs/worklog-2026-07-30.md` — design decisions and experiment history.
- `auto_clear.py` — non-interactive entry point used by the Jetson gate.

## Supported environment

The validated development environment used macOS with:

- Python 3.12 for the arm process;
- `lerobot[feetech]==0.6.0`;
- OpenCV, SciPy, and Matplotlib; and
- a separate camera-only Python environment when macOS camera permissions
  required the camera server to own the device.

Jetson/Linux deployment uses the same Python modules but must be recalibrated
for its serial device, camera, work surface, and lighting.

## Create the environments

From the repository root:

```bash
cd so-arm101

python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "lerobot[feetech]==0.6.0" \
  opencv-python scipy matplotlib
export PYTHONPATH="$PWD/src"
```

On macOS, create the camera environment with a Python installation that has
camera permission:

```bash
python3.13 -m venv .venv-cam
.venv-cam/bin/python -m pip install opencv-python
```

On Linux, add the operator to the serial-device group when required and set
the actual port explicitly:

```bash
export SOARM_PORT=/dev/ttyACM0
```

## Hardware calibration

The tracked `calibration/lerobot/main_arm.json` belongs to the original arm.
Install it only for that same physical unit:

```bash
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so_follower
cp -i calibration/lerobot/main_arm.json \
  ~/.cache/huggingface/lerobot/calibration/robots/so_follower/main_arm.json
```

Never copy this file to a different arm. Recalibrate replacement hardware
instead. See `calibration/lerobot/README.md` for details.

## Preflight

Keep the arm unpowered while checking configuration and camera access. Then
run the numbered tools in order, with the work area clear:

```bash
PYTHONPATH=src .venv/bin/python scripts/01_test_arm.py
.venv-cam/bin/python scripts/camera_server.py --index 0
PYTHONPATH=src .venv/bin/python scripts/05_verify_observe.py
```

Interactive teaching tools such as `04_record_pose.py`, `10_servo_calib.py`,
`15_teach_class.py`, `16_measure_grip.py`, `18_teach_grid.py`, and
`19_manual_grasp.py` require an operator at the hardware.

## Run the sorting workflow

Use two terminals from `so-arm101/`.

Terminal 1 — camera server:

```bash
.venv-cam/bin/python scripts/camera_server.py --index 0
```

Terminal 2 — supervised sorting loop:

```bash
export PYTHONPATH="$PWD/src"
export SOARM_PORT=/dev/ttyACM0
.venv/bin/python scripts/09_grasp.py --loop --step
```

The Jetson handoff gate invokes the non-interactive entry point only after its
ROI dwell condition succeeds:

```bash
PYTHONPATH=src SOARM_PORT=/dev/ttyACM0 \
  .venv/bin/python auto_clear.py
```

## Relocation rules

Moving the arm, camera, tray, lighting, or drop container invalidates some or
all site calibration. Follow `docs/deployment-new-site.md`; at minimum, verify
poses, redraw the ROI, capture a clean reference, reteach the grid, and
recalibrate the visual-servo Jacobian before a grasp attempt.

The committed calibration images and JSON files document the validated setup.
They are examples and historical evidence, not universal deployment values.

## Safety

- Keep an emergency-stop or power-disconnect action within immediate reach.
- Clear the workspace before enabling torque or running any motion script.
- Start with low-speed, step-by-step commands after every configuration change.
- Do not mix raw servo units with normalized gripper units.
- Inspect `config/poses.json` after recording poses; teaching scripts can
  replace stored values.
- Do not run two processes against the same serial port or camera.
- Treat perception success as insufficient proof of physical clearance.

For the full design rationale and known failure modes, read
`docs/worklog-2026-07-30.md` and `docs/deployment-new-site.md` before changing
the control or calibration strategy.
