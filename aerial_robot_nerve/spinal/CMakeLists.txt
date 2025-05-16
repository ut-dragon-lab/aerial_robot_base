cmake_minimum_required(VERSION 3.5)
project(spinal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(SPINAL_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/mcu_project/lib/My_Lib)

include_directories(
  ${SPINAL_DIRS}
)


add_definitions(-DSIMULATION)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PMatrixPseudoInverseUnit.msg"
  "msg/PMatrixPseudoInverseWithInertia.msg"
  "msg/ServoInfo.msg"
  "msg/BoardInfo.msg"
  "msg/ServoState.msg"
  "msg/ServoStates.msg"
  "msg/ServoControlCmd.msg"
  "msg/ServoTorqueCmd.msg"
  "msg/ServoTorqueStates.msg"
  "msg/Gyro.msg"
  "msg/SimpleImu.msg"
  "msg/FourAxisCommand.msg"
  "msg/Imu.msg"
  "msg/Barometer.msg"
  "msg/Gps.msg"
  "msg/GpsFull.msg"
  "msg/RollPitchYawTerm.msg"
  "msg/RollPitchYawTerms.msg"
  "msg/MotorInfo.msg"
  "msg/PwmInfo.msg"
  "msg/Pwms.msg"
  "msg/PwmTest.msg"
  "msg/UavInfo.msg"
  "msg/DesireCoord.msg"
  "msg/FlightConfigCmd.msg"
  "msg/Vector3Int16.msg"
  "msg/TorqueAllocationMatrixInv.msg"
  "msg/ESCTelemetry.msg"
  "msg/ESCTelemetryArray.msg"
  "msg/JointProfile.msg"
  "msg/JointProfiles.msg"
  "srv/GetBoardInfo.srv"
  "srv/SetBoardConfig.srv"
  "srv/SetAttitudeGains.srv"
  "srv/ImuCalib.srv"
  "srv/MagDeclination.srv"
  "srv/SetDirectServoConfig.srv"
  DEPENDENCIES std_msgs geometry_msgs
  )

add_library(spinal_math
  ${SPINAL_DIRS}/math/AP_Math.cpp
  ${SPINAL_DIRS}/math/matrix3.cpp
  ${SPINAL_DIRS}/math/vector2.cpp
  ${SPINAL_DIRS}/math/vector3.cpp
  ${SPINAL_DIRS}/math/quaternion.cpp
  # ${SPINAL_DIRS}/math/location.cpp
  )


add_library(spinal_state_estimation INTERFACE)
target_include_directories(spinal_state_estimation
  INTERFACE
  $<BUILD_INTERFACE:${SPINAL_DIRS}>
  $<INSTALL_INTERFACE:include/My_Lib>
  )
target_link_libraries(spinal_state_estimation
  INTERFACE
  rclcpp::rclcpp
  spinal_math
  )


install(
  TARGETS spinal_state_estimation spinal_math
  # EXPORT export_aerial_robot_model
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )


install(
  DIRECTORY ${SPINAL_DIRS}/
  DESTINATION include/My_Lib
  FILES_MATCHING PATTERN "*.h"
  )


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/My_Lib)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
