cmake_minimum_required(VERSION 3.8)
project(aerial_robot_navigation)

# --- C++17 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(spinal REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  nav_msgs
  visualization_msgs
  geodesy
  aerial_robot_estimation
  aerial_robot_msgs
  spinal
)

# --- Build flight_navigation library ---
add_library(flight_navigation
  src/flight_navigation.cpp
  src/util/joy_parser.cpp
  src/trajectory/reference_base.cpp
  src/trajectory/trajectory_reference/polynomial.cpp
  src/trajectory/trajectory_reference/polynomial_trajectory.cpp
  src/trajectory/trajectory_reference/sampled_trajectory.cpp
  src/trajectory/base/parameter_base.cpp
  src/trajectory/math/math.cpp
  src/trajectory/types/command.cpp
  src/trajectory/types/quad_state.cpp
  src/trajectory/types/quadrotor.cpp
  src/trajectory/utils/logger.cpp
  src/trajectory/utils/timer.cpp
)

ament_target_dependencies(flight_navigation
  ${ROS_DEPENDENCIES}
)

target_include_directories(flight_navigation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Installation ---
install(
  TARGETS
    flight_navigation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# --- Export ---
ament_export_include_directories(include)
ament_export_libraries(
  flight_navigation
)
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
)

# --- Linters ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
