cmake_minimum_required(VERSION 3.5)
project(aerial_robot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ヘッダーファイルが含まれるディレクトリをインクルードパスに追加
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(aerial_robot_base_node
  src/aerial_robot_base_node.cpp
  src/aerial_robot_base.cpp
)

ament_target_dependencies(aerial_robot_base_node
  rclcpp
  std_msgs
)

# インストール時にヘッダーファイルを含める（オプション）
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  aerial_robot_base_node
  DESTINATION lib/${PROJECT_NAME}
  )

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
