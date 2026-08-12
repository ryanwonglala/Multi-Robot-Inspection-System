cmake_minimum_required(VERSION 3.8)
project(task_layer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install task_layer/ as an importable Python package so nodes can do
# `from task_layer.report_writer import ...` etc.
ament_python_install_package(task_layer)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

install(PROGRAMS
  task_layer/task_allocator.py
  task_layer/set_initial_pose_node.py
  task_layer/go_to_area_node.py
  task_layer/inspection_runner.py
  task_layer/reset_to_dock.py
  task_layer/task_gui_node.py
  task_layer/model_spawner.py
  task_layer/scene_builder_gui_node.py
  task_layer/area_clear_acceptance.py
  task_layer/demo_audio_node.py
  task_layer/anomaly_texture_node.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_set_initial_pose_node test/test_set_initial_pose_node.py)
  ament_add_pytest_test(test_photo_baseline test/test_photo_baseline.py)
  ament_add_pytest_test(test_photo_diff_check test/test_photo_diff_check.py)
  ament_add_pytest_test(
    test_inspection_sector_mapping test/test_inspection_sector_mapping.py)
endif()

ament_package()
