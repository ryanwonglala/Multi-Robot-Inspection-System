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
  task_layer/scan_analyzer.py
  task_layer/report_writer.py
  task_layer/scene_builder_gui_node.py
  task_layer/area_clear_acceptance.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
