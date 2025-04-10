#include "aerial_robot_base/aerial_robot_base.h"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

AerialRobotBase::AerialRobotBase()
: Node("aerial_robot_base")
{
  //declare parameters
  this->declare_parameter<bool>("param_verbose", true);
  this->declare_parameter<double>("main_rate", 1.0);
  //get parameters
  bool param_verbose = this->get_parameter("param_verbose").as_bool();
  double main_rate = this->get_parameter("main_rate").as_double();

  if (param_verbose) {
    RCLCPP_INFO(this->get_logger(), "%s: main_rate is %f", this->get_namespace(), main_rate);
  }

  if (main_rate <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "main rate is negative or zero, cannot run the main timer");
  } else {
    //create timer
    auto period = std::chrono::duration<double>(1.0 / main_rate);
    main_timer_ = this->create_wall_timer(
      period,
      std::bind(&AerialRobotBase::mainFunc, this)
    );
  }

  //for debug
  debug_pub_ = this->create_publisher<std_msgs::msg::String>("debug_topic", 10);  
}

AerialRobotBase::~AerialRobotBase()
{
  //we don't need any stop process since they are sopped automatically
}

void AerialRobotBase::mainFunc()
{
  //navigator_->update();
  // controller_->update();

  //for debug
  std_msgs::msg::String msg;
  std::stringstream ss;
  ss << "Debug message at time " << this->now().seconds();
  msg.data = ss.str();
  debug_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
}
