cmake_minimum_required(VERSION 3.5)
project(aerial_robot_model)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ament & ROS 2 packages ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
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
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddExtraModule.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(aerial_robot_model_base SHARED
  src/model/base_model/robot_model.cpp
  src/model/transformable_model/robot_model.cpp
  src/model/transformable_model/kinematics.cpp
  src/model/transformable_model/jacobians.cpp
  src/model/transformable_model/stability.cpp
  src/model/transformable_model/statics.cpp
)
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
target_link_libraries(aerial_robot_model_base
  ${orocos_kdl_LIBRARIES}
)

## pluginlib
add_library(robot_model_pluginlib SHARED
  src/model/plugin/multirotor_robot_model.cpp
  src/model/plugin/underactuated_tilted_robot_model.cpp
)
ament_target_dependencies(robot_model_pluginlib
  pluginlib
)
target_link_libraries(robot_model_pluginlib
  aerial_robot_model_base
)

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugins/robot_model_plugins.xml
)

install(
  TARGETS
    aerial_robot_model_base
    # robot_model_pluginlib
  ARCHIVE   DESTINATION lib
  LIBRARY   DESTINATION lib
  RUNTIME   DESTINATION bin
)

install(
  DIRECTORY
    include
    launch
    script
    plugins
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_libraries(aerial_robot_model_base # robot_model_pluginlib
  )
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  pluginlib
  orocos_kdl
  kdl_parser
  Eigen3
  urdfdom_headers
  urdf
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  tinyxml2_vendor
  TinyXML2
  rosidl_default_runtime
)

ament_package()
