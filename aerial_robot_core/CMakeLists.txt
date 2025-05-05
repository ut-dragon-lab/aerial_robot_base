cmake_minimum_required(VERSION 3.5)
project(aerial_robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )


add_executable(aerial_robot_core_node
  src/aerial_robot_core_node.cpp
  src/aerial_robot_core.cpp
  )

ament_target_dependencies(aerial_robot_core_node
  rclcpp
  std_msgs
  std_srvs
  aerial_robot_model
  )

install(
  DIRECTORY include/
  DESTINATION include
  )

install(TARGETS
  aerial_robot_core_node
  DESTINATION lib/${PROJECT_NAME}
  )

# install python files
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/keyboard_command.py
  scripts/rms.py
  scripts/send_igmp_report.py
  scripts/simple_demo.py
  DESTINATION lib/${PROJECT_NAME}
  )

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
