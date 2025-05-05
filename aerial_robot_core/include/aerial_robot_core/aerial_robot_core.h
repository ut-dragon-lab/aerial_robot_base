#pragma once

#include <aerial_robot_model/model/aerial_robot_model_ros.h>

#include <memory>
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

  // robot_model_ros
  std::shared_ptr<aerial_robot_model::RobotModelRos> robot_model_ros_;

  // for debug
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};
