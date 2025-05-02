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
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(interactive_markers REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddExtraModule.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
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
# aerial_robot_model_ros
add_library(aerial_robot_model_ros
  src/model/base_model/robot_model_ros.cpp)
ament_target_dependencies(aerial_robot_model_ros
  rclcpp
  pluginlib
  rosidl_default_runtime
)
target_link_libraries(aerial_robot_model_ros
  aerial_robot_model_base
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

add_library(servo_bridge src/servo_bridge/servo_bridge.cpp)
ament_target_dependencies(servo_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  controller_manager_msgs
  spinal
  urdf
  kalman_filter
  tf2_eigen
  )
# target_link_libraries(servo_bridge lpf_filter)
add_executable(servo_bridge_node src/servo_bridge/servo_bridge_node.cpp)
ament_target_dependencies(servo_bridge_node
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  controller_manager_msgs
  spinal
  )
target_link_libraries (servo_bridge_node servo_bridge)

add_executable(rotor_tf_publisher src/utils/rotor_tf_publisher.cpp)
ament_target_dependencies(rotor_tf_publisher
  rclcpp
  tf2_ros
  tf2_kdl
  kdl_parser
  urdf
  geometry_msgs
  )
target_link_libraries(rotor_tf_publisher ${orocos_kdl_LIBRARIES})


add_executable(interactive_marker_tf_broadcaster src/utils/interactive_marker_tf_broadcaster.cpp)
ament_target_dependencies(interactive_marker_tf_broadcaster
  rclcpp
  interactive_markers
  visualization_msgs
  geometry_msgs
  tf2_ros
)
target_link_libraries(interactive_marker_tf_broadcaster)

install(
  TARGETS
  aerial_robot_model_base
  aerial_robot_model_ros
  robot_model_pluginlib  
  ARCHIVE   DESTINATION lib
  LIBRARY   DESTINATION lib
  RUNTIME   DESTINATION bin
)

install(
  DIRECTORY
    include
    launch
    # script
    plugins
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  )

install(TARGETS rotor_tf_publisher interactive_marker_tf_broadcaster
  servo_bridge_node
  DESTINATION lib/${PROJECT_NAME}
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
  std_srvs
  tf2_ros
  interactive_markers
  visualization_msgs
  controller_manager_msgs
  spinal
  )

ament_package()
