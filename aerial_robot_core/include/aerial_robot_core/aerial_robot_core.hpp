// -*- mode: c++ -*-
/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *   3. Neither the name of the DRAGON Laboratory nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

/* Standard library */
#include <memory>
#include <chrono>
#include <cstdint>
#include <functional>

/* ROS 2 */
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>

/* Aerial robot packages */
#include "aerial_robot_model/model/aerial_robot_model_ros.h"
#include "aerial_robot_estimation/state_estimation.h"
#include "aerial_robot_navigation/flight_navigation.hpp"
#include "aerial_robot_control/base/control_base.hpp"

class AerialRobotCore
{
public:
  AerialRobotCore(rclcpp::Node::SharedPtr node);
  ~AerialRobotCore();

private:
  bool param_verbose_;
  double main_rate_;
  bool warn_main_rate_;
  double main_rate_warn_tolerance_;
  int main_rate_warn_throttle_ms_;
  int main_rate_warn_warmup_count_;
  int main_rate_warn_count_{ 0 };
  rclcpp::TimerBase::SharedPtr main_timer_;

  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
  int64_t last_main_time_ns_{ 0 };

  // Node handle
  rclcpp::Node::SharedPtr node_;

  // Model
  std::shared_ptr<aerial_robot_model::RobotModelRos> robot_model_ros_;

  // Estimator
  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;

  // Navigator
  std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator_;

  // Controller
  pluginlib::ClassLoader<aerial_robot_control::ControlBase> controller_loader_;
  std::shared_ptr<aerial_robot_control::ControlBase> controller_;

  // For debug messages
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

  // Main loop
  void mainFunc();
};
