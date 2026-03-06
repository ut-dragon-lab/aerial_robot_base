cmake_minimum_required(VERSION 3.5)
project(aerial_robot_estimation)

# --- C++17 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ament & ROS 2 packages ---
find_package(ament_cmake REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
#find_package(geodesy REQUIRED)
#find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)

# --- common dependent packages ---
set(ROS_DEPENDENCIES
  aerial_robot_model
  aerial_robot_msgs
  #geodesy
  #geographic_msgs
  geometry_msgs
  kalman_filter
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  std_srvs
  spinal
  tf2_eigen_kdl
)

# --- aerial_robot_estimation library ---
add_library(aerial_robot_estimation SHARED
  src/state_estimation.cpp
)
ament_target_dependencies(aerial_robot_estimation
  ${ROS_DEPENDENCIES}
)
target_include_directories(aerial_robot_estimation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

# --- sensor plugin library ---
add_library(sensor_pluginlib SHARED
  src/sensor/imu.cpp
  src/sensor/ground_truth.cpp
  src/sensor/mocap.cpp
  src/sensor/odom.cpp
)
ament_target_dependencies(sensor_pluginlib
  ${ROS_DEPENDENCIES}
)
target_include_directories(sensor_pluginlib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


ament_export_include_directories(include)
ament_export_dependencies(ROS_DEPENDENCIES
  aerial_robot_model
  aerial_robot_msgs
  #geodesy
  #geographic_msgs
  geometry_msgs
  kalman_filter
  nav_msgs
  pluginlib
  sensor_msgs
  std_srvs
  spinal
  tf2_eigen_kdl
)

ament_package()
