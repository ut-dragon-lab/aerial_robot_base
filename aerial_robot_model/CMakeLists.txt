cmake_minimum_required(VERSION 3.5)
project(aerial_robot_model)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ament & ROS 2 packages ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(spinal_msgs REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(interactive_markers REQUIRED)

# -- auto‐generate ROS interfaces --------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddExtraModule.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# --- common dependent packages -----------------------------------------------
set(COMMON_DEPENDENCIES
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  spinal
  spinal_msgs
)

set(MODEL_LIB_DEPENDENCIES
  ${COMMON_DEPENDENCIES}
  pluginlib
  rosidl_default_runtime
)

include_directories(
  include
  test
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# --- aerial_robot_model_base library ---------------------------------------
add_library(aerial_robot_model_base SHARED
  src/model/base_model/robot_model.cpp
  src/model/transformable_model/robot_model.cpp
  src/model/transformable_model/kinematics.cpp
  src/model/transformable_model/jacobians.cpp
  src/model/transformable_model/stability.cpp
  src/model/transformable_model/statics.cpp
)
ament_target_dependencies(aerial_robot_model_base
  ${COMMON_DEPENDENCIES}
  urdf
  orocos_kdl
  kdl_parser
  tf2_kdl
  tf2_eigen
  tf2_geometry_msgs
  tinyxml2_vendor
  TinyXML2
)
target_link_libraries(aerial_robot_model_base
  ${orocos_kdl_LIBRARIES}
  )

# --- aerial_robot_model_ros wrapper ----------------------------------------
add_library(aerial_robot_model_ros
  src/model/base_model/robot_model_ros.cpp)
ament_target_dependencies(aerial_robot_model_ros
  ${MODEL_LIB_DEPENDENCIES}
)
target_link_libraries(aerial_robot_model_ros
  aerial_robot_model_base
  )
target_include_directories(aerial_robot_model_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
rosidl_target_interfaces(aerial_robot_model_ros
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)  

# --- robot_model plugin ----------------------------------------
add_library(robot_model_plugin SHARED
  src/model/plugin/multirotor_robot_model.cpp
  src/model/plugin/underactuated_tilted_robot_model.cpp
)
ament_target_dependencies(robot_model_plugin
  pluginlib
)
target_link_libraries(robot_model_plugin
  aerial_robot_model_base
)
pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugins/robot_model_plugins.xml
  )

# --- servo_bridge ----------------------------------------
add_library(servo_bridge src/servo_bridge/servo_bridge.cpp)
ament_target_dependencies(servo_bridge
  ${COMMON_DEPENDENCIES}
  controller_manager_msgs
  urdf
  kalman_filter
  tf2_eigen
  )

add_executable(servo_bridge_node src/servo_bridge/servo_bridge_node.cpp)
ament_target_dependencies(servo_bridge_node
  ${COMMON_DEPENDENCIES}
  controller_manager_msgs
  )
target_link_libraries (servo_bridge_node
  servo_bridge
  )

# --- rotor_tf_publisher--------------------------------------------
add_executable(rotor_tf_publisher
  src/utils/rotor_tf_publisher.cpp
  )
ament_target_dependencies(rotor_tf_publisher
  rclcpp
  tf2_ros
  tf2_kdl
  kdl_parser
  urdf
  geometry_msgs
  )
target_link_libraries(rotor_tf_publisher ${orocos_kdl_LIBRARIES})

# --- interactive_marker_tf_broadcaster------------------------------
add_executable(interactive_marker_tf_broadcaster src/utils/interactive_marker_tf_broadcaster.cpp)
ament_target_dependencies(interactive_marker_tf_broadcaster
  rclcpp
  interactive_markers
  visualization_msgs
  geometry_msgs
  tf2_ros
)
target_link_libraries(interactive_marker_tf_broadcaster)

# --- jacobian test------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(numerical_jacobian_test
    test/aerial_robot_model/numerical_jacobians.cpp
  )
  ament_target_dependencies(numerical_jacobian_test
    rclcpp
    sensor_msgs
    spinal
    spinal_msgs
    )
  target_link_libraries(numerical_jacobian_test
    aerial_robot_model_base
    )
endif()

# --- installation------------------------------
install(
  TARGETS
    aerial_robot_model_base
    aerial_robot_model_ros
    robot_model_plugin
    # EXPORT export_aerial_robot_model
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(
  TARGETS
    servo_bridge
    servo_bridge_node
    rotor_tf_publisher
    interactive_marker_tf_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY plugins launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(
  aerial_robot_model_base
  aerial_robot_model_ros
  )
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_targets(export_aerial_robot_model HAS_LIBRARY_TARGET)
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
  spinal_msgs
  )

ament_package()
