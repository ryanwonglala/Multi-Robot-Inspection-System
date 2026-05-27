# Task-Driven Collaborative Multi-Robot Inspection System for Indoor Environments

## Project Overview

This project is currently in the early concept and planning stage. Our current direction is to develop a ROS2-based indoor inspection robot system for structured indoor environments such as laboratories, corridors, storage rooms, equipment areas, and facility entrances.

The main idea is to build a mobile robot that can receive simple inspection tasks from users and perform basic autonomous inspection. For example, the user may ask the robot to check whether a doorway is blocked, inspect a selected area, or follow a preset patrol route. After receiving the task, the robot should be able to navigate to the target location, collect sensor data, detect simple abnormal conditions, and provide understandable feedback to the user.

At the current stage, we plan to use a simulation-first development approach. We will first build a virtual indoor environment in Gazebo and use ROS2 as the main software framework to test navigation, sensing, and inspection logic. RViz and Nav2 may be used for map visualization, path planning, localization, and autonomous navigation. After the basic functions are verified in simulation, we hope to test part of the system on real robot hardware if time and resources allow.

The devices and platforms we are considering include TurtleBot3, LiDAR, camera or depth camera, Jetson, a four-wheel mobile chassis with encoders, So-Arm101 robotic arm, and possibly small 3Pi+ robots for simple exploration experiments. The main focus at this stage is to first develop a working mobile inspection prototype with clear task execution, sensor feedback, and basic reporting.

The initial functions we hope to achieve include:

- Let the robot move to selected indoor locations
- Let the robot follow a simple preset inspection route
- Use LiDAR or camera data to check simple abnormal situations, such as blocked doorways or corridors
- Capture basic inspection evidence, such as images
- Provide simple real-time warning messages when an abnormal situation is detected
- Generate a simple inspection result or report after the task is completed

The goal of this project is not to build a fully commercial robot at the beginning, but to create a clear and testable prototype system. Through this project, we hope to demonstrate that a ROS2-based mobile robot can understand basic inspection tasks, navigate in an indoor environment, collect useful sensor information, and report inspection results in a way that is easy for users to understand.
