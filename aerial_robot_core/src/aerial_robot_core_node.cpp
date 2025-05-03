#include "aerial_robot_core/aerial_robot_core.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto core_node = std::make_shared<rclcpp::Node>("aerial_robot_core", options);

  auto core = std::make_shared<AerialRobotCore>(core_node);
  rclcpp::spin(core_node);
  rclcpp::shutdown();
  return 0;
}
