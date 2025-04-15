cmake_minimum_required(VERSION 3.5)
project(spinal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
