cmake_minimum_required(VERSION 3.8)
project(sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
  scripts/ctl.py
  scripts/mapping_ctl.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
