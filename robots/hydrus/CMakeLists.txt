cmake_minimum_required(VERSION 3.5)
project(hydrus)

find_package(aerial_robot_core REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_simulation REQUIRED)
find_package(spinal REQUIRED)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
