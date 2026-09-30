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
#include <gimbalrotor/control/gimbalrotor_controller.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pluginlib/class_list_macros.hpp>

namespace aerial_robot_control
{
GimbalrotorController::GimbalrotorController()
  : PosePIDControllerBase(),
    hovering_approximate_(false),
    candidate_yaw_term_(0.0),
    gimbal_dof_(1),
    rotor_coef_(2),
    gimbal_calc_in_fc_(true),
    underactuate_(false),
    target_roll_(0.0),
    target_pitch_(0.0)
{
}

void GimbalrotorController::initialize(rclcpp::Node::SharedPtr node,
                                       std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                                       double ctrl_loop_dt)
{
  PosePIDControllerBase::initialize(node, robot_model, estimator, navigator, ctrl_loop_dt);
  gimbalrotor_robot_model_ = std::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);
  if (!gimbalrotor_robot_model_)
  {
    RCLCPP_ERROR(node_->get_logger(), "[gimbalrotor] robot model plugin is not GimbalrotorRobotModel");
  }

  rosParamInit();

  if (gimbal_dof_ != 1 && gimbal_dof_ != 2)
  {
    RCLCPP_WARN(node_->get_logger(), "[gimbalrotor] unsupported gimbal_dof=%d; use 1", gimbal_dof_);
    gimbal_dof_ = 1;
  }
  rotor_coef_ = gimbal_dof_ + 1;

  target_base_thrust_.resize(motor_num_ * rotor_coef_, 0.0f);
  target_full_thrust_.resize(motor_num_, 0.0f);
  target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0.0);

  flight_cmd_pub_ = node_->create_publisher<spinal_msgs::msg::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("gimbals_ctrl", 1);
  gimbal_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  target_vectoring_force_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "debug/target_vectoring_force", 1);
  rpy_gain_pub_ = node_->create_publisher<spinal_msgs::msg::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ = node_->create_publisher<spinal_msgs::msg::TorqueAllocationMatrixInv>(
      "torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("gimbal_dof", 1);
}

void GimbalrotorController::rosParamInit()
{
  getParam<int>("controller.gimbal_dof", gimbal_dof_, 1);
  getParam<bool>("controller.gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  getParam<bool>("controller.hovering_approximate", hovering_approximate_, false);
  getParam<bool>("controller.underactuate", underactuate_, false);
}

bool GimbalrotorController::update()
{
  sendGimbalCommand();
  if (gimbal_calc_in_fc_)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = gimbal_dof_;
    gimbal_dof_pub_->publish(msg);
  }

  return PosePIDControllerBase::update();
}

void GimbalrotorController::reset()
{
  PosePIDControllerBase::reset();
  setAttitudeGains();
}

void GimbalrotorController::controlCore()
{
  PosePIDControllerBase::controlCore();
  if (!gimbalrotor_robot_model_) return;

  const KDL::Rotation uav_rot = estimator_->getCogOrientation(estimate_mode_);
  const KDL::Vector target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                                 pid_controllers_.at(Z).result());
  const KDL::Rotation yaw_rot = KDL::Rotation::RPY(0.0, 0.0, rpy_.z());
  const KDL::Vector target_acc_dash = yaw_rot.Inverse() * target_acc_w;
  const KDL::Vector target_acc_cog = uav_rot.Inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

  if (underactuate_)
  {
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
  }
  else
  {
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
  }

  const double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  const double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  const double target_ang_acc_z = pid_controllers_.at(YAW).result();
  const Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
  const Eigen::Vector3d omega(omega_.x(), omega_.y(), omega_.z());
  const Eigen::Vector3d gyro = omega.cross(inertia * omega);

  if (gimbal_calc_in_fc_)
  {
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  }
  else
  {
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + inertia.inverse() * gyro;
  }

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
  const double mass_inv = 1.0 / gimbalrotor_robot_model_->getMass();
  const Eigen::Matrix3d inertia_inv = inertia.inverse();
  const auto rotors_origin_from_cog = gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto &rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
  const double m_f_rate = gimbalrotor_robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;
  for (int i = 0; i < motor_num_; ++i)
  {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  const auto thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();
  std::vector<Eigen::MatrixXd> masked_rot;
  masked_rot.reserve(motor_num_);
  for (int i = 0; i < motor_num_; ++i)
  {
    const Eigen::Matrix3d conv_cog_from_thrust = aerial_robot_model::kdlToEigen(thrust_coords_rot.at(i));
    if (gimbal_dof_ == 1)
    {
      Eigen::MatrixXd mask(3, 2);
      mask << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
    else
    {
      masked_rot.push_back(conv_cog_from_thrust);
    }
  }

  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
  for (int i = 0; i < motor_num_; ++i)
  {
    integrated_rot.block(3 * i, rotor_coef_ * i, 3, rotor_coef_) = masked_rot[i];
  }
  Eigen::MatrixXd integrated_map = full_q_mat * integrated_rot;

  if (underactuate_)
  {
    target_wrench_acc_cog = target_wrench_acc_cog.tail(4);
    integrated_map = integrated_map.bottomRows(4);
  }

  const Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
  integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
  integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
  if (underactuate_)
  {
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
  }
  else
  {
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
  }
  target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);

  if (underactuate_)
  {
    if (hovering_approximate_)
    {
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
    }
    else
    {
      target_roll_ = atan2(-target_acc_dash.y(),
                           sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
    }
    navigator_->setTargetCogRoll(target_roll_);
    navigator_->setTargetCogPitch(target_pitch_);
  }

  double max_yaw_scale = 0.0;
  last_col = 0;
  for (int i = 0; i < motor_num_; ++i)
  {
    const Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    for (int j = 0; j < rotor_coef_; ++j)
    {
      target_base_thrust_.at(rotor_coef_ * i + j) = static_cast<float>(f_i[j]);
    }

    const int yaw_col = underactuate_ ? YAW - 2 : YAW;
    if (integrated_map_inv(i, yaw_col) > max_yaw_scale)
    {
      max_yaw_scale = integrated_map_inv(i, yaw_col);
    }
    last_col += rotor_coef_;
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  last_col = 0;
  for (int i = 0; i < motor_num_; ++i)
  {
    const Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) +
                                           target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    target_full_thrust_.at(i) = static_cast<float>(f_i_integrated.norm());
    if (gimbal_dof_ == 1)
    {
      target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
    }
    else
    {
      if (f_i_integrated.norm() > 1.0e-9)
      {
        const double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
        const double gimbal_pitch = atan2(f_i_integrated[0],
                                          -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2] * cos(gimbal_roll));
        target_gimbal_angles_.at(2 * i) = gimbal_roll;
        target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
      }
    }
    last_col += rotor_coef_;
  }
}

void GimbalrotorController::sendCmd()
{
  PosePIDControllerBase::sendCmd();
  sendFourAxisCommand();

  if (gimbal_calc_in_fc_)
  {
    sendTorqueAllocationMatrixInv();
  }
  else
  {
    sensor_msgs::msg::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = node_->now();
    for (int i = 0; i < motor_num_; ++i)
    {
      if (gimbal_dof_ == 1)
      {
        gimbal_control_msg.name.push_back("gimbal" + std::to_string(i + 1));
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
      }
      else
      {
        gimbal_control_msg.name.push_back("gimbal" + std::to_string(i + 1) + "_roll");
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i));
        gimbal_control_msg.name.push_back("gimbal" + std::to_string(i + 1) + "_pitch");
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      }
    }
    gimbal_control_pub_->publish(gimbal_control_msg);

    std_msgs::msg::Float32MultiArray target_vectoring_force_msg;
    target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
    for (int i = 0; i < target_vectoring_f_.size(); ++i)
    {
      target_vectoring_force_msg.data.push_back(static_cast<float>(target_vectoring_f_(i)));
    }
    target_vectoring_force_pub_->publish(target_vectoring_force_msg);
  }
}

void GimbalrotorController::sendFourAxisCommand()
{
  spinal_msgs::msg::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = static_cast<float>(target_roll_);
  flight_command_data.angles[1] = static_cast<float>(target_pitch_);

  if (gimbal_calc_in_fc_)
  {
    flight_command_data.base_thrust = target_base_thrust_;
    flight_command_data.angles[2] = static_cast<float>(candidate_yaw_term_);
  }
  else
  {
    flight_command_data.base_thrust = target_full_thrust_;
  }

  flight_cmd_pub_->publish(flight_command_data);
}

void GimbalrotorController::sendGimbalCommand()
{
  sensor_msgs::msg::JointState gimbal_state_msg;
  gimbal_state_msg.header.stamp = node_->now();
  for (int i = 0; i < motor_num_; ++i)
  {
    if (gimbal_dof_ == 1)
    {
      gimbal_state_msg.name.push_back("gimbal" + std::to_string(i + 1));
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
    }
    else
    {
      gimbal_state_msg.name.push_back("gimbal" + std::to_string(i + 1) + "_roll");
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i));
      gimbal_state_msg.name.push_back("gimbal" + std::to_string(i + 1) + "_pitch");
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
    }
  }
  (void)gimbal_state_msg;
}

void GimbalrotorController::sendTorqueAllocationMatrixInv()
{
  if (integrated_map_inv_rot_.rows() == 0) return;

  spinal_msgs::msg::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
  torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
  if (integrated_map_inv_rot_.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
  {
    RCLCPP_ERROR(node_->get_logger(), "[gimbalrotor] Torque Allocation Matrix overflow");
  }

  for (int i = 0; i < motor_num_ * rotor_coef_; ++i)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = static_cast<int16_t>(integrated_map_inv_rot_(i, 0) * 1000);
    torque_allocation_matrix_inv_msg.rows.at(i).y = static_cast<int16_t>(integrated_map_inv_rot_(i, 1) * 1000);
    torque_allocation_matrix_inv_msg.rows.at(i).z = static_cast<int16_t>(integrated_map_inv_rot_(i, 2) * 1000);
  }
  torque_allocation_matrix_inv_pub_->publish(torque_allocation_matrix_inv_msg);
}

void GimbalrotorController::setAttitudeGains()
{
  spinal_msgs::msg::RollPitchYawTerms rpy_gain_msg;
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = static_cast<int16_t>(pid_controllers_.at(ROLL).getPGain() * 1000);
  rpy_gain_msg.motors.at(0).roll_i = static_cast<int16_t>(pid_controllers_.at(ROLL).getIGain() * 1000);
  rpy_gain_msg.motors.at(0).roll_d = static_cast<int16_t>(pid_controllers_.at(ROLL).getDGain() * 1000);
  rpy_gain_msg.motors.at(0).pitch_p = static_cast<int16_t>(pid_controllers_.at(PITCH).getPGain() * 1000);
  rpy_gain_msg.motors.at(0).pitch_i = static_cast<int16_t>(pid_controllers_.at(PITCH).getIGain() * 1000);
  rpy_gain_msg.motors.at(0).pitch_d = static_cast<int16_t>(pid_controllers_.at(PITCH).getDGain() * 1000);
  rpy_gain_msg.motors.at(0).yaw_d = static_cast<int16_t>(pid_controllers_.at(YAW).getDGain() * 1000);
  rpy_gain_pub_->publish(rpy_gain_msg);
}
}  // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorController, aerial_robot_control::ControlBase)
