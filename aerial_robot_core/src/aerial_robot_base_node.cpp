#include "aerial_robot_base/aerial_robot_base.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AerialRobotBase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
