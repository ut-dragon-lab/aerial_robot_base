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
#include "aerial_robot_core/aerial_robot_core.hpp"

using namespace std::chrono_literals;


AerialRobotCore::AerialRobotCore(rclcpp::Node::SharedPtr node)
  : node_(node), controller_loader_("aerial_robot_control", "aerial_robot_control::ControlBase")
{
  // Get parameters from launch file
  node_->get_parameter_or("param_verbose", param_verbose_, false);
  node_->get_parameter_or("main_rate", main_rate_, 1.0);
  node_->get_parameter_or("warn_main_rate", warn_main_rate_, true);
  node_->get_parameter_or("main_rate_warn_tolerance", main_rate_warn_tolerance_, 0.20);
  node_->get_parameter_or("main_rate_warn_throttle_ms", main_rate_warn_throttle_ms_, 5000);
  node_->get_parameter_or("main_rate_warn_warmup_count", main_rate_warn_warmup_count_, 10);
  double main_dt = 1.0 / main_rate_;

  if (param_verbose_) RCLCPP_INFO(node_->get_logger(), "%s: main rate is %f Hz", node_->get_namespace(), main_rate_);

  if (main_rate_ <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Main rate is zero or negative!");
    return;
  }

  if (main_rate_warn_tolerance_ < 0.0)
  {
    RCLCPP_WARN(node_->get_logger(), "main_rate_warn_tolerance must be non-negative. Use 0.0 instead.");
    main_rate_warn_tolerance_ = 0.0;
  }
  if (main_rate_warn_throttle_ms_ <= 0)
  {
    RCLCPP_WARN(node_->get_logger(), "main_rate_warn_throttle_ms must be positive. Use 5000 ms instead.");
    main_rate_warn_throttle_ms_ = 5000;
  }
  if (main_rate_warn_warmup_count_ < 0)
  {
    RCLCPP_WARN(node_->get_logger(), "main_rate_warn_warmup_count must be non-negative. Use 0 instead.");
    main_rate_warn_warmup_count_ = 0;
  }

  /* Model */
  robot_model_ros_ = std::make_shared<aerial_robot_model::RobotModelRos>(node_);
  auto robot_model = robot_model_ros_->getRobotModel();

  /* Estimator */
  estimator_ = std::make_shared<aerial_robot_estimation::StateEstimator>();
  estimator_->initialize(node_, robot_model);

  /* Navigation */
  navigator_ = std::make_shared<aerial_robot_navigation::NavigationBase>();
  navigator_->initialize(node_, robot_model, estimator_, main_dt);

  /* Controller */
  try
  {
    std::string aerial_robot_control_name;
    node_->get_parameter_or("aerial_robot_control_name", aerial_robot_control_name,
                            std::string("aerial_robot_control/under_actuated_pid_controller"));
    controller_ = controller_loader_.createSharedInstance(aerial_robot_control_name);
    controller_->initialize(node_, robot_model, estimator_, navigator_, main_dt);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "The controller plugin failed to load for some reason. Error: %s", ex.what());
  }

  /* Timer */
  auto period = rclcpp::Duration::from_seconds(main_dt);
  main_timer_ = rclcpp::create_timer(node_, node_->get_clock(), period, std::bind(&AerialRobotCore::mainFunc, this));
}

AerialRobotCore::~AerialRobotCore()
{
  // We don't need any stop processes since they are stopped automatically
}

void AerialRobotCore::mainFunc()
{
  const int64_t now_ns = steady_clock_.now().nanoseconds();
  if (last_main_time_ns_ != 0)
  {
    const double dt_real = static_cast<double>(now_ns - last_main_time_ns_) * 1e-9;
    const double dt_desire = (1.0 + main_rate_warn_tolerance_) * 1.0 / main_rate_;
    if (main_rate_warn_count_ < main_rate_warn_warmup_count_)
    {
      ++main_rate_warn_count_;
    }
    else if (warn_main_rate_ && dt_real > dt_desire)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), steady_clock_, main_rate_warn_throttle_ms_,
                           "Main loop rate is too low: (ts_real) %f s > (ts_desire incl. %2f%% tol) %f s", dt_real,
                           main_rate_warn_tolerance_ * 100, dt_desire);
    }
  }
  last_main_time_ns_ = now_ns;  // In nanosecons

  // Main update of navigation and control
  navigator_->update();
  controller_->update();
}
