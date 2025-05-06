// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "aerial_robot_core/aerial_robot_core.h"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

AerialRobotCore::AerialRobotCore(rclcpp::Node::SharedPtr node) : node_(node) {
  // declare parameters
  node_->declare_parameter<bool>("param_verbose", true);
  node_->declare_parameter<double>("main_rate", 1.0);
  // get parameters
  bool param_verbose = node_->get_parameter("param_verbose").as_bool();
  double main_rate = node_->get_parameter("main_rate").as_double();

  robot_model_ros_ = std::make_shared<aerial_robot_model::RobotModelRos>(node_);
  auto robot_model = robot_model_ros_->getRobotModel();

  if (param_verbose) {
    RCLCPP_INFO(node_->get_logger(), "%s: main_rate is %f", node_->get_namespace(), main_rate);
  }

  if (main_rate <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "main rate is negative or zero, cannot run the main timer");
  } else {
    // create timer
    auto period = std::chrono::duration<double>(1.0 / main_rate);
    main_timer_ = node_->create_wall_timer(period, std::bind(&AerialRobotCore::mainFunc, this));
  }

  // // for debug
  // debug_pub_ = node_->create_publisher<std_msgs::msg::String>("debug_topic", 10);
}

AerialRobotCore::~AerialRobotCore() {
  // we don't need any stop process since they are sopped automatically
}

void AerialRobotCore::mainFunc() {
  // navigator_->update();
  //  controller_->update();

  // for debug
  std_msgs::msg::String msg;
  std::stringstream ss;
  ss << "Debug message at time " << node_->now().seconds();
  msg.data = ss.str();
  // debug_pub_->publish(msg);
  // RCLCPP_INFO(node_->get_logger(), "Published: '%s'", msg.data.c_str());
}
