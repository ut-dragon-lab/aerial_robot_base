cmake_minimum_required(VERSION 3.5)
project(hydrus)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
