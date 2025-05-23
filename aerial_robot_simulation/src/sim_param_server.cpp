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

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sim_param_server");

  node->declare_parameter<std::string>("my_param", "hello_world");
  node->declare_parameter<int>("other_param", 42);

  node->declare_parameter<double>("ground_truth_pub_rate", 0.01);
  node->declare_parameter<double>("ground_truth_pos_noise", 0.0);
  node->declare_parameter<double>("ground_truth_vel_noise", 0.0);
  node->declare_parameter<double>("ground_truth_rot_noise", 0.0);
  node->declare_parameter<double>("ground_truth_angular_noise", 0.0);
  node->declare_parameter<double>("ground_truth_rot_drift", 0.0);
  node->declare_parameter<double>("ground_truth_vel_drift", 0.0);
  node->declare_parameter<double>("ground_truth_angular_drift", 0.0);
  node->declare_parameter<double>("ground_truth_rot_drift_frequency", 0.0);
  node->declare_parameter<double>("ground_truth_vel_drift_frequency", 0.0);
  node->declare_parameter<double>("ground_truth_angular_drift_frequency", 0.0);
  node->declare_parameter<double>("mocap_pub_rate", 0.01);
  node->declare_parameter<double>("mocap_pos_noise", 0.001);
  node->declare_parameter<double>("mocap_rot_noise", 0.001);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
