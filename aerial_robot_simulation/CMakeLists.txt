cmake_minimum_required(VERSION 3.5)
project(aerial_robot_simulation)

# --- C++17 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
  set(BUILD_GAZEBO_SIM OFF)
else()
  set(BUILD_GAZEBO_SIM ON)
endif()

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
find_package(gz-math7 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
# find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_manager REQUIRED)

if(BUILD_GAZEBO_SIM)
  find_package(gz-cmake3 REQUIRED)
  find_package(gz-sim8 REQUIRED)
  find_package(gz-plugin2 REQUIRED)
  find_package(gz-gui8 REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Core Qml Quick)
  find_package(gz_ros2_control REQUIRED)
else()
  message(STATUS "Building aerial_robot_simulation without Gazebo support on ${CMAKE_SYSTEM_PROCESSOR}")
endif()

add_definitions(-DSIMULATION -DGZ_HEADERS)

include_directories(
  include
)

# Rotor handle lib
add_library(rotor_handle_lib INTERFACE)
target_include_directories(rotor_handle_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(rotor_handle_lib INTERFACE
  rclcpp
  urdfdom_headers
  gz-math7
  tinyxml2_vendor
  aerial_robot_model
)

# Spinal interface
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

if(BUILD_GAZEBO_SIM)
  add_library(RenderRateLimit SHARED src/render_rate_limit.cpp src/render_rate_limit.qrc)
  set_target_properties(RenderRateLimit PROPERTIES AUTOMOC ON AUTORCC ON)
  target_link_libraries(RenderRateLimit gz-gui8::gz-gui8 gz-plugin2::register)

  add_library(aerial_robot_hw_sim SHARED
    src/aerial_robot_hw_sim.cpp)
  ament_target_dependencies(aerial_robot_hw_sim
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    nav_msgs
    geometry_msgs
    gz-math7
    gz_ros2_control
    gz-sim8
  )
  target_link_libraries(aerial_robot_hw_sim
    spinal_interface
  )

  add_library(simulation_attitude_controller SHARED
    src/simulation_attitude_controller.cpp
  )
  ament_target_dependencies(simulation_attitude_controller
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    nav_msgs
    geometry_msgs
    gz_ros2_control
    gz-sim8
    controller_interface
    spinal
  )
  target_link_libraries(simulation_attitude_controller
    spinal_interface
    spinal::spinal_flight_control
  )

  pluginlib_export_plugin_description_file(
    gz_ros2_control
    aerial_robot_hw_sim_plugins.xml
  )

  pluginlib_export_plugin_description_file(
    controller_interface
    flight_controllers_plugins.xml
  )
endif()

add_executable(sim_param_server src/sim_param_server.cpp)
ament_target_dependencies(sim_param_server
  rclcpp
)

set(PROJECT_TARGETS
  rotor_handle_lib
  spinal_interface
  sim_param_server
)

if(BUILD_GAZEBO_SIM)
  list(APPEND PROJECT_TARGETS
    RenderRateLimit
    aerial_robot_hw_sim
    simulation_attitude_controller
  )
endif()

install(
  TARGETS ${PROJECT_TARGETS}
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
)

if(BUILD_GAZEBO_SIM)
  install(
    FILES
      aerial_robot_hw_sim_plugins.xml
      flight_controllers_plugins.xml
    DESTINATION
      share/${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gz-math7)
if(BUILD_GAZEBO_SIM)
  ament_export_definitions(GZ_HEADERS)
  ament_export_dependencies(gz_ros2_control gz-sim8)
endif()

# Linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
