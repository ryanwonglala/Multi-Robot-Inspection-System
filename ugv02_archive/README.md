# UGV02 Archive

This folder stores the UGV02 / Wave Rover code separately from the main
multi-robot workflow.

The UGV02 platform is not part of the active integrated project because of the
ESP32 hardware fault, but the driver, navigation, mapping, teleoperation, and
test code are kept here for reference and future recovery.

## Contents

- `ros2_ws/src/wave_rover_driver/`
  - ROS 2 Python package for the Wave Rover / UGV02 driver.
- `scripts/`
  - Jetson-side launch, teleoperation, odometry, feedback, and test scripts.
- `config/`
  - Nav2, SLAM, and RViz configuration files used during UGV02 testing.
- `maps/`
  - Saved UGV02 map files.
- `firmware_tools/`
  - Small ESP32/Arduino diagnostic sketches kept as source only.

## Notes

- This archive is intentionally not wired into the active TurtleBot3 +
  RealSense + SO-ARM101 workflow.
- Logs, Python caches, firmware dumps, compressed factory packages, generated
  Arduino build outputs, and old backup variants were not included.
- If UGV02 hardware is repaired later, start from
  `ros2_ws/src/wave_rover_driver/` and the launch scripts in `scripts/`.
