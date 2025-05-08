cmake_minimum_required(VERSION 3.5)
project(aerial_robot_simulation)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(spinal REQUIRED)

install(
  DIRECTORY launch xacro config gazebo_model
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  )

install(PROGRAMS scripts/gazebo_control_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()
