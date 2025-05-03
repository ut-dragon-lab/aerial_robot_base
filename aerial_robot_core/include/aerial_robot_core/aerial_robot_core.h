#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class AerialRobotCore {
 public:
  AerialRobotCore(rclcpp::Node::SharedPtr node);
  ~AerialRobotCore();

 private:
  void mainFunc();
  rclcpp::TimerBase::SharedPtr main_timer_;

  // node handle
  rclcpp::Node::SharedPtr node_;

  // for debug
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};
