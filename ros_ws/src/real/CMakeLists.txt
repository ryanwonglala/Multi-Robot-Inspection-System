cmake_minimum_required(VERSION 3.8)
project(real)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/analyze_apriltag_reference.py
  scripts/apriltag_alignment_monitor.py
  scripts/apriltag_visual_servo.py
  scripts/backup_odom_lidar.py
  scripts/drive_odom_short.py
  scripts/laser_sector_probe.py
  scripts/d436_depth_viewer.py
  scripts/capture_camera_frame.py
  scripts/calibrate_tb3_camera.py
  scripts/env_fleet.sh
  scripts/env_jetson.sh
  scripts/env_real.sh
  scripts/fleet_hotspot_reconnect.py
  scripts/jetson_d436_rsusb_start.sh
  scripts/rotate_odom_test.py
  scripts/tb3_hotspot_reconnect.py
  scripts/tb3_robot_start.sh
  scripts/ugv02_opensource_node.py
  scripts/ugv_motion_probe.py
  scripts/validate_vp_rotation.py
  DESTINATION share/${PROJECT_NAME}/scripts/)

# Installed to lib/ (not share/scripts/) so it is runnable as
# `ros2 run real tb3_healthcheck.py`, unlike env_real.sh (sourced) and
# tb3_robot_start.sh (scp'd to the robot and run there).
install(PROGRAMS
  scripts/tb3_healthcheck.py
  scripts/real_workflow_gui.py
  scripts/run_hybrid_docking.py
  scripts/run_full_workflow.py
  scripts/run_return_home.py
  scripts/viewpoint_markers.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Covers only the pure logic of tb3_healthcheck.py (phase selection, exit
  # codes, FastDDS XML parsing, battery/rate/scan verdicts) -- no rclpy, no
  # network, so it runs with no robot present.
  ament_add_pytest_test(test_tb3_healthcheck test/test_tb3_healthcheck.py)
  ament_add_pytest_test(
    test_fleet_hotspot_reconnect test/test_fleet_hotspot_reconnect.py)
  ament_add_pytest_test(
    test_ugv_motion_probe test/test_ugv_motion_probe.py)
  ament_add_pytest_test(
    test_ugv02_opensource_node test/test_ugv02_opensource_node.py)
  ament_add_pytest_test(
    test_run_hybrid_docking test/test_run_hybrid_docking.py)
  ament_add_pytest_test(
    test_run_full_workflow test/test_run_full_workflow.py)
  ament_add_pytest_test(
    test_run_return_home test/test_run_return_home.py)
  ament_add_pytest_test(
    test_real_workflow_gui test/test_real_workflow_gui.py)
  ament_add_pytest_test(
    test_nav2_real_config test/test_nav2_real_config.py)
endif()

ament_package()
