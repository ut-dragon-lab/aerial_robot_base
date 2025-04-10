#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class AerialRobotBase : public rclcpp::Node
{
public:
  AerialRobotBase();
  ~AerialRobotBase();

private:
  void mainFunc();
  rclcpp::TimerBase::SharedPtr main_timer_;

  //for debug
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};
