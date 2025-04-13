cmake_minimum_required(VERSION 3.5)
project(aerial_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## Declare ROS messages and services
set(msg_files
  "msg/DynamicReconfigureLevels.msg"
  "msg/FourAxisGain.msg"
  "msg/AerialRobotStatus.msg"
  "msg/FlightNav.msg"
  "msg/PoseControlPid.msg"
  "msg/Pid.msg"
  "msg/State.msg"
  "msg/States.msg"
  "msg/Acc.msg"
  "msg/WrenchAllocationMatrix.msg"
)

## Generate added messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
