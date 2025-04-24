cmake_minimum_required(VERSION 3.5)
project(aerial_robot_model)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find ament and ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# Use vendor-provided tinyxml2
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# include paths
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# build the RobotModel library
add_library(aerial_robot_model_base SHARED
  src/model/base_model/robot_model.cpp
)

# link dependencies
ament_target_dependencies(aerial_robot_model_base
  rclcpp
  orocos_kdl
  kdl_parser
  Eigen3
  urdf
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  tinyxml2_vendor
  TinyXML2
)

# link additional system libraries
target_link_libraries(aerial_robot_model_base
  ${orocos_kdl_LIBRARIES}
)

# install rules
install(
  TARGETS aerial_robot_model_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddExtraModule.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()
