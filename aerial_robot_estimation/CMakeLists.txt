cmake_minimum_required(VERSION 3.5)
project(aerial_robot_estimation)

# --- C++17 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geodesy REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(spinal_msgs REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  pluginlib
  tf2_eigen_kdl
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  nav_msgs
  std_srvs
  geodesy
  kalman_filter
  aerial_robot_model
  aerial_robot_msgs
  spinal
  spinal_msgs
)

# --- Build aerial_robot_estimation library ---
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

# --- Build sensor_plugin library ---
add_library(sensor_pluginlib SHARED
  src/sensor/base_plugin.cpp
  src/sensor/imu.cpp
  src/sensor/imu_for_wrench_est.cpp
  src/sensor/ground_truth.cpp
  src/sensor/mocap.cpp
  src/sensor/odom.cpp
  src/sensor/gps.cpp
  src/sensor/altitude.cpp
)
ament_target_dependencies(sensor_pluginlib
  ${ROS_DEPENDENCIES}
)

target_include_directories(sensor_pluginlib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugins/sensor_plugins.xml
)

# --- Installation ---
install(
  TARGETS
    aerial_robot_estimation
    sensor_pluginlib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY plugins
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# --- Export ---
ament_export_include_directories(include)
ament_export_libraries(
  aerial_robot_estimation
  sensor_pluginlib
)
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
)

# --- Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
