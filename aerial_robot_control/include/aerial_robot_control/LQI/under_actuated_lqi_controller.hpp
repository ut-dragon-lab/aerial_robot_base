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

/* Aerial robot packages */
#include "aerial_robot_control/PID/pose_pid_controller_base.hpp"
#include "aerial_robot_control/LQI/care.hpp"
#include "aerial_robot_msgs/msg/four_axis_gain.hpp"
#include "spinal_msgs/msg/four_axis_command.hpp"
#include "spinal_msgs/msg/roll_pitch_yaw_terms.hpp"
#include "spinal_msgs/msg/p_matrix_pseudo_inverse_with_inertia.hpp"

namespace aerial_robot_control
{

class UnderActuatedLQIController : public PosePIDControllerBase
{
public:
  UnderActuatedLQIController();
  virtual ~UnderActuatedLQIController();

  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator, double ctrl_loop_dt);

  void activate() override;

protected:
  rclcpp::Publisher<spinal_msgs::msg::FourAxisCommand>::SharedPtr flight_cmd_pub_;  // for spinal
  rclcpp::Publisher<spinal_msgs::msg::RollPitchYawTerms>::SharedPtr rpy_gain_pub_;  // for spinal
  rclcpp::Publisher<aerial_robot_msgs::msg::FourAxisGain>::SharedPtr four_axis_gain_pub_;
  rclcpp::Publisher<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>::SharedPtr p_matrix_pseudo_inverse_inertia_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::thread gain_generator_thread_;
  double gain_generate_rate_;
  bool realtime_update_;

  double target_roll_, target_pitch_;
  double candidate_yaw_term_;
  std::vector<float> target_base_thrust_;

  int lqi_mode_;
  bool clamp_gain_;
  Eigen::MatrixXd K_;
  bool has_optimal_gain_;

  Eigen::Vector3d lqi_roll_pitch_weight_, lqi_yaw_weight_, lqi_z_weight_;
  std::vector<double> r_;  // Matrix R
  std::vector<Eigen::Vector3d> pitch_gains_, roll_gains_, yaw_gains_, z_gains_;

  bool gyro_moment_compensation_;

  bool verbose_;
  double trans_constraint_weight_;
  double att_control_weight_;

  virtual void rosParamInit();
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  void gainGeneratorFunc();
  bool updateGain(bool publish_gain);

  virtual bool optimalGain();
  void resetGain() { K_ = Eigen::MatrixXd(); }
  virtual void clampGain();

  virtual void allocateZTerm();
  virtual void allocateYawTerm();

  virtual void controlCore() override;

  virtual void sendGain();
  virtual void sendCmd() override;
  virtual void sendFourAxisCommand();
  void sendRotationalInertiaComp();
};

}
