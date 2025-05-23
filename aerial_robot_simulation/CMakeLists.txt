cmake_minimum_required(VERSION 3.5)
project(aerial_robot_simulation)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(gazebo_ros2_control REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
# find_package(ros_gz_sim REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_manager REQUIRED)

find_package(ignition-plugin1 REQUIRED)

add_definitions(-DSIMULATION)

include_directories(
  include
  )

# rotor handle lib
add_library(rotor_handle_lib INTERFACE)
target_include_directories(rotor_handle_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
ament_target_dependencies(rotor_handle_lib INTERFACE
  rclcpp
  urdfdom_headers
  ignition-math6
  tinyxml2_vendor
  aerial_robot_model
)

# spinal interface
add_library(spinal_interface SHARED
  src/spinal_interface.cpp
)

target_include_directories(spinal_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

ament_target_dependencies(spinal_interface
  rclcpp
  geometry_msgs
  nav_msgs
  hardware_interface
  urdf
  spinal
  )
target_link_libraries(spinal_interface
  rotor_handle_lib
  )

add_library(aerial_robot_hw_sim SHARED
  src/aerial_robot_hw_sim.cpp)
ament_target_dependencies(aerial_robot_hw_sim
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  nav_msgs
  geometry_msgs
  ignition-math6
  gz_ros2_control
  ignition-gazebo6
  )
target_link_libraries(aerial_robot_hw_sim
  spinal_interface
  )
pluginlib_export_plugin_description_file(
    gz_ros2_control
    aerial_robot_hw_sim_plugins.xml
)

add_library(simulation_attitude_controller SHARED
  src/simulation_attitude_controller.cpp)
ament_target_dependencies(simulation_attitude_controller
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  nav_msgs
  geometry_msgs
  gz_ros2_control
  ignition-gazebo6
  controller_interface
  )
target_link_libraries(simulation_attitude_controller
  spinal_interface
  )

add_executable(sim_param_server src/sim_param_server.cpp)
ament_target_dependencies(sim_param_server
  rclcpp
  )


pluginlib_export_plugin_description_file(
  gz_ros2_control
  aerial_robot_hw_sim_plugins.xml
  )

pluginlib_export_plugin_description_file(
  controller_interface
  flight_controllers_plugins.xml
  )


install(
  TARGETS rotor_handle_lib spinal_interface aerial_robot_hw_sim simulation_attitude_controller sim_param_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  )

install(
  DIRECTORY launch xacro config gazebo_model
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  )

install(
  FILES
    aerial_robot_hw_sim_plugins.xml
  DESTINATION
    share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gazebo_control_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
