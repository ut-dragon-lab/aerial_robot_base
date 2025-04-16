#include "aerial_robot_core/aerial_robot_core.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AerialRobotCore>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
