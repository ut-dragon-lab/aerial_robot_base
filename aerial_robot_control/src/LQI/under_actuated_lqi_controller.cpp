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
#include "aerial_robot_control/LQI/under_actuated_lqi_controller.hpp"

namespace aerial_robot_control
{

UnderActuatedLQIController::UnderActuatedLQIController()
  : gain_generate_rate_(15.0),
    realtime_update_(false),
    target_roll_(0),
    target_pitch_(0),
    candidate_yaw_term_(0),
    lqi_mode_(4),
    clamp_gain_(true),
    has_optimal_gain_(false),
    gyro_moment_compensation_(false),
    verbose_(false),
    trans_constraint_weight_(1.0),
    att_control_weight_(1.0)
{
  lqi_roll_pitch_weight_.setZero();
  lqi_yaw_weight_.setZero();
  lqi_z_weight_.setZero();
}

void UnderActuatedLQIController::initialize(rclcpp::Node::SharedPtr node,
                                            std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                            std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                            std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                                            double ctrl_loop_dt)
{
  PosePIDControllerBase::initialize(node, robot_model, estimator, navigator, ctrl_loop_dt);

  rosParamInit();

  rpy_gain_pub_ = node_->create_publisher<spinal_msgs::msg::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = node_->create_publisher<spinal_msgs::msg::FourAxisCommand>("four_axes/command", 1);
  four_axis_gain_pub_ = node_->create_publisher<aerial_robot_msgs::msg::FourAxisGain>("debug/four_axes/gain", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = node_->create_publisher<spinal_msgs::msg::PMatrixPseudoInverseWithInertia>(
      "p_matrix_pseudo_inverse_inertia", 1);

  // Parameter-change callback
  param_cb_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&UnderActuatedLQIController::parametersCallback, this, std::placeholders::_1));

  pitch_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));
  roll_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));
  z_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));
  yaw_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));

  target_base_thrust_.resize(motor_num_);
  pid_msg_.z.total.resize(motor_num_);
  pid_msg_.z.p_term.resize(motor_num_);
  pid_msg_.z.i_term.resize(motor_num_);
  pid_msg_.z.d_term.resize(motor_num_);
  pid_msg_.yaw.total.resize(motor_num_);
  pid_msg_.yaw.p_term.resize(motor_num_);
  pid_msg_.yaw.i_term.resize(motor_num_);
  pid_msg_.yaw.d_term.resize(motor_num_);

  if (!robot_model_->isModelFixed())
  {
    realtime_update_ = true;
  }

  if (realtime_update_)
  {
    gain_generator_thread_ = std::thread(std::bind(&UnderActuatedLQIController::gainGeneratorFunc, this));
  }
  else
  {
    (void)updateGain(false);
  }
}

UnderActuatedLQIController::~UnderActuatedLQIController()
{
  // Clean up multithreading
  if (gain_generator_thread_.joinable())
  {
    gain_generator_thread_.join();
  }
}

void UnderActuatedLQIController::rosParamInit()
{
  std::string lqi_ns = "controller.lqi";
  getParam<bool>(lqi_ns + ".clamp_gain", clamp_gain_, true);
  getParam<bool>(lqi_ns + ".realtime_update", realtime_update_, false);
  getParam<bool>(lqi_ns + ".gyro_moment_compensation", gyro_moment_compensation_, false);

  /* Propeller direction and LQI R */
  r_.resize(motor_num_);
  for (int i = 0; i < motor_num_; ++i)
  {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    getParam<double>(lqi_ns + ".r" + ss.str(), r_.at(i), 1.0);
  }

  getParam<double>(lqi_ns + ".roll_pitch_p", lqi_roll_pitch_weight_[0], 1.0);
  getParam<double>(lqi_ns + ".roll_pitch_i", lqi_roll_pitch_weight_[1], 1.0);
  getParam<double>(lqi_ns + ".roll_pitch_d", lqi_roll_pitch_weight_[2], 1.0);
  getParam<double>(lqi_ns + ".yaw_p", lqi_yaw_weight_[0], 1.0);
  getParam<double>(lqi_ns + ".yaw_i", lqi_yaw_weight_[1], 1.0);
  getParam<double>(lqi_ns + ".yaw_d", lqi_yaw_weight_[2], 1.0);
  getParam<double>(lqi_ns + ".z_p", lqi_z_weight_[0], 1.0);
  getParam<double>(lqi_ns + ".z_i", lqi_z_weight_[1], 1.0);
  getParam<double>(lqi_ns + ".z_d", lqi_z_weight_[2], 1.0);

  getParam<int>(lqi_ns + ".lqi_mode", lqi_mode_, 4);
  if (lqi_mode_ != 3 && lqi_mode_ != 4)
  {
    RCLCPP_ERROR(node_->get_logger(), "[LQI] LQI mode should be 3 or 4, %d is not allowed.", lqi_mode_);
  }
}

void UnderActuatedLQIController::gainGeneratorFunc()
{
  getParam<double>("controller.lqi.gain_generate_rate", gain_generate_rate_, 15.0);
  rclcpp::Rate loop_rate(gain_generate_rate_);

  while (rclcpp::ok())
  {
    (void)updateGain(true);
    loop_rate.sleep();
  }
}

void UnderActuatedLQIController::activate()
{
  ControlBase::activate();

  if (!has_optimal_gain_ && !realtime_update_)
  {
    (void)updateGain(false);
  }

  // Publish gains during activation for general multirotor
  if (has_optimal_gain_)
  {
    sendGain();
    RCLCPP_INFO(node_->get_logger(), "[LQI] Send LQI gains");
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "[LQI] Cannot solve hamilton matrix!");
  }
}

bool UnderActuatedLQIController::updateGain(bool publish_gain)
{
  if (!robot_model_->initialized())
  {
    RCLCPP_DEBUG(node_->get_logger(), "[LQI] Robot model is not initialized!");
    resetGain();
    has_optimal_gain_ = false;
    return false;
  }

  has_optimal_gain_ = optimalGain();
  if (!has_optimal_gain_)
  {
    RCLCPP_ERROR(node_->get_logger(), "[LQI] Cannot solve hamilton matrix!");
    return false;
  }

  clampGain();
  if (publish_gain)
  {
    sendGain();
  }
  return true;
}

bool UnderActuatedLQIController::optimalGain()
{
  // Reference:
  // M, Zhao, et.al, "Transformable multirotor with two-dimensional multilinks:
  // modeling, control, and whole-body aerial manipulation" Sec. 3.2

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(lqi_mode_, motor_num_);
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  P_dash.row(0) = P.row(2) / robot_model_->getMass();                                               // Z
  P_dash.bottomRows(lqi_mode_ - 1) = (inertia.inverse() * P.bottomRows(3)).topRows(lqi_mode_ - 1);  // Roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode_, lqi_mode_ * 3);
  for (int i = 0; i < lqi_mode_; i++)
  {
    A(2 * i, 2 * i + 1) = 1;
    B.row(2 * i + 1) = P_dash.row(i);
    C(i, 2 * i) = 1;
  }
  A.block(lqi_mode_ * 2, 0, lqi_mode_, lqi_mode_ * 3) = -C;

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[LQI] Gain generator: B: \n" << B);

  Eigen::VectorXd q_diagonals(lqi_mode_ * 3);
  if (lqi_mode_ == 3)
  {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2),
        lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1),
        lqi_roll_pitch_weight_(1);
  }
  else
  {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2),
        lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_z_weight_(1),
        lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  }
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(motor_num_, motor_num_);
  for (int i = 0; i < motor_num_; ++i) R(i, i) = r_.at(i);

  /* Solve continuous-time algebraic Ricatti equation */
  double t = node_->get_clock()->now().seconds();

  if (K_.cols() != lqi_mode_ * 3)
  {
    resetGain();  // Four axis -> three axis and vice versa
  }

  bool use_kleinman_method = true;
  if (K_.cols() == 0 || K_.rows() == 0)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "[LQI] Gain generator: Not using Kleinman method since "
                        "initial K is empty");
    use_kleinman_method = false;
  }
  if (!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[LQI] Gain generator: Error in solver of continuous-time "
                 "algebraic Riccati equation (CARE)");
    return false;
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "[LQI] Gain generator: CARE: " << node_->get_clock()->now().seconds() - t << " sec");
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[LQI] Gain generator: K \n" << K_);

  for (int i = 0; i < motor_num_; ++i)
  {
    roll_gains_.at(i) = Eigen::Vector3d(-K_(i, 2), K_(i, lqi_mode_ * 2 + 1), -K_(i, 3));
    pitch_gains_.at(i) = Eigen::Vector3d(-K_(i, 4), K_(i, lqi_mode_ * 2 + 2), -K_(i, 5));
    z_gains_.at(i) = Eigen::Vector3d(-K_(i, 0), K_(i, lqi_mode_ * 2), -K_(i, 1));
    if (lqi_mode_ == 4)
      yaw_gains_.at(i) = Eigen::Vector3d(-K_(i, 6), K_(i, lqi_mode_ * 2 + 3), -K_(i, 7));
    else
      yaw_gains_.at(i).setZero();
  }

  return true;
}

void UnderActuatedLQIController::clampGain()
{
  /* Avoid the violation of 16int_t range because of spinal::RollPitchYawTerms
   */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for (int i = 0; i < motor_num_; ++i)
  {
    if (max_roll_p_gain < fabs(roll_gains_.at(i)[0])) max_roll_p_gain = fabs(roll_gains_.at(i)[0]);
    if (max_roll_d_gain < fabs(roll_gains_.at(i)[2])) max_roll_d_gain = fabs(roll_gains_.at(i)[2]);
    if (max_pitch_p_gain < fabs(pitch_gains_.at(i)[0])) max_pitch_p_gain = fabs(pitch_gains_.at(i)[0]);
    if (max_pitch_d_gain < fabs(pitch_gains_.at(i)[2])) max_pitch_d_gain = fabs(pitch_gains_.at(i)[2]);
    if (max_yaw_d_gain < fabs(yaw_gains_.at(i)[2])) max_yaw_d_gain = fabs(yaw_gains_.at(i)[2]);
  }

  double roll_p_gain_scale = 1, roll_d_gain_scale = 1, pitch_p_gain_scale = 1, pitch_d_gain_scale = 1,
         yaw_d_gain_scale = 1;
  if (max_roll_p_gain > max_gain_thresh)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[LQI] Gain generator: the max roll p gain violate the range "
                "of int16_t: %f",
                max_roll_p_gain);
    roll_p_gain_scale = max_gain_thresh / max_roll_p_gain;
  }
  if (max_roll_d_gain > max_gain_thresh)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[LQI] Gain generator: the max roll d gain violate the range "
                "of int16_t: %f",
                max_roll_d_gain);
    roll_d_gain_scale = max_gain_thresh / max_roll_d_gain;
  }
  if (max_pitch_p_gain > max_gain_thresh)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[LQI] Gain generator: the max pitch p gain violate the range "
                "of int16_t: %f",
                max_pitch_p_gain);
    pitch_p_gain_scale = max_gain_thresh / max_pitch_p_gain;
  }
  if (max_pitch_d_gain > max_gain_thresh)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[LQI] Gain generator: the max pitch d gain violate the range "
                "of int16_t: %f",
                max_pitch_d_gain);
    pitch_d_gain_scale = max_gain_thresh / max_pitch_d_gain;
  }
  if (max_yaw_d_gain > max_gain_thresh)
  {
    RCLCPP_WARN(node_->get_logger(),
                "[LQI] Gain generator: the max yaw d gain violate the range of "
                "int16_t: %f",
                max_yaw_d_gain);
    yaw_d_gain_scale = max_gain_thresh / max_yaw_d_gain;
  }

  for (int i = 0; i < motor_num_; ++i)
  {
    roll_gains_.at(i)[0] *= roll_p_gain_scale;
    roll_gains_.at(i)[2] *= roll_d_gain_scale;

    pitch_gains_.at(i)[0] *= pitch_p_gain_scale;
    pitch_gains_.at(i)[2] *= pitch_d_gain_scale;

    yaw_gains_.at(i)[2] *= yaw_d_gain_scale;
  }
}

void UnderActuatedLQIController::controlCore()
{
  PosePIDControllerBase::controlCore();

  tf2::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                            pid_controllers_.at(Z).result());

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, rpy_.z());
  tf2::Matrix3x3 rotation_matrix(q);
  tf2::Vector3 target_acc_dash = rotation_matrix.inverse() * target_acc_w;

  target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
  target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;

  allocateZTerm();

  allocateYawTerm();
}

void UnderActuatedLQIController::allocateZTerm()
{
  Eigen::VectorXd target_thrust_z_term = Eigen::VectorXd::Zero(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    double p_term = z_gains_.at(i)[0] * pid_controllers_.at(Z).getErrP();
    double i_term = z_gains_.at(i)[1] * pid_controllers_.at(Z).getErrI();
    double d_term = z_gains_.at(i)[2] * pid_controllers_.at(Z).getErrD();
    target_thrust_z_term(i) = p_term + i_term + d_term;
    pid_msg_.z.p_term.at(i) = p_term;
    pid_msg_.z.i_term.at(i) = i_term;
    pid_msg_.z.d_term.at(i) = d_term;
  }

  // Feed-forward term for z
  Eigen::MatrixXd q_mat_inv = getQInv();
  double ff_acc_z = navigator_->getTargetCogAcc().z();
  Eigen::VectorXd ff_term = q_mat_inv.col(0) * ff_acc_z;
  target_thrust_z_term += ff_term;

  // Constraint z (also I-term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(Z).getLimitSum();
  if (residual > 0)
  {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
    target_thrust_z_term *= (1 - residual / max_term);
  }

  for (int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = target_thrust_z_term(i);
    pid_msg_.z.total.at(i) = target_thrust_z_term(i);
  }
}

void UnderActuatedLQIController::allocateYawTerm()
{
  Eigen::VectorXd target_thrust_yaw_term = Eigen::VectorXd::Zero(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    double p_term = yaw_gains_.at(i)[0] * pid_controllers_.at(YAW).getErrP();
    double i_term = yaw_gains_.at(i)[1] * pid_controllers_.at(YAW).getErrI();
    double d_term = yaw_gains_.at(i)[2] * pid_controllers_.at(YAW).getErrD();
    target_thrust_yaw_term(i) = p_term + i_term + d_term;
    pid_msg_.yaw.p_term.at(i) = p_term;
    pid_msg_.yaw.i_term.at(i) = i_term;
    pid_msg_.yaw.d_term.at(i) = d_term;
  }

  // Feed-forward term for yaw
  Eigen::MatrixXd q_mat_inv = getQInv();
  double ff_ang_yaw = navigator_->getTargetCogAngularAcc().z();
  Eigen::VectorXd ff_term = q_mat_inv.col(3) * ff_ang_yaw;
  target_thrust_yaw_term += ff_term;

  // Constraint yaw (also I-term)
  int index;
  double max_term = target_thrust_yaw_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(YAW).getLimitSum();
  if (residual > 0)
  {
    pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getPrevErrI());
    target_thrust_yaw_term *= (1 - residual / max_term);
  }

  // Special process for yaw because of the limited bandwidth between PC and
  // spinal
  double max_yaw_scale = 0;  // To reconstruct yaw control term in spinal
  for (int i = 0; i < motor_num_; i++)
  {
    pid_msg_.yaw.total.at(i) = target_thrust_yaw_term(i);

    if (yaw_gains_[i][2] > max_yaw_scale)
    {
      max_yaw_scale = yaw_gains_[i][2];
      candidate_yaw_term_ = target_thrust_yaw_term(i);
    }
  }
}

void UnderActuatedLQIController::sendGain()
{
  aerial_robot_msgs::msg::FourAxisGain four_axis_gain_msg;
  spinal_msgs::msg::RollPitchYawTerms rpy_gain_msg;  // Send to spinal
  rpy_gain_msg.motors.resize(motor_num_);

  for (int i = 0; i < motor_num_; ++i)
  {
    four_axis_gain_msg.roll_p_gain.push_back(roll_gains_.at(i)[0]);
    four_axis_gain_msg.roll_i_gain.push_back(roll_gains_.at(i)[1]);
    four_axis_gain_msg.roll_d_gain.push_back(roll_gains_.at(i)[2]);

    four_axis_gain_msg.pitch_p_gain.push_back(pitch_gains_.at(i)[0]);
    four_axis_gain_msg.pitch_i_gain.push_back(pitch_gains_.at(i)[1]);
    four_axis_gain_msg.pitch_d_gain.push_back(pitch_gains_.at(i)[2]);

    four_axis_gain_msg.yaw_p_gain.push_back(yaw_gains_.at(i)[0]);
    four_axis_gain_msg.yaw_i_gain.push_back(yaw_gains_.at(i)[1]);
    four_axis_gain_msg.yaw_d_gain.push_back(yaw_gains_.at(i)[2]);

    four_axis_gain_msg.z_p_gain.push_back(z_gains_.at(i)[0]);
    four_axis_gain_msg.z_i_gain.push_back(z_gains_.at(i)[1]);
    four_axis_gain_msg.z_d_gain.push_back(z_gains_.at(i)[2]);

    /* To flight controller via rosserial scaling by 1000 */
    rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i)[0] * 1000;
    rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i)[1] * 1000;
    rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i)[2] * 1000;

    rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i)[0] * 1000;
    rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i)[1] * 1000;
    rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i)[2] * 1000;

    rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i)[2] * 1000;
  }
  rpy_gain_pub_->publish(rpy_gain_msg);
  four_axis_gain_pub_->publish(four_axis_gain_msg);
}

void UnderActuatedLQIController::sendCmd()
{
  PosePIDControllerBase::sendCmd();

  sendFourAxisCommand();
  sendRotationalInertiaComp();
}

void UnderActuatedLQIController::sendFourAxisCommand()
{
  spinal_msgs::msg::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_->publish(flight_command_data);
}

void UnderActuatedLQIController::sendRotationalInertiaComp()
{
  if (!gyro_moment_compensation_) return;

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));

  spinal_msgs::msg::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;  // To spinal
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for (int i = 0; i < motor_num_; ++i)
  {
    /* The p matrix pseudo inverse and inertia */
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
    if (lqi_mode_ == 4)
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;
    else
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0;
  }

  /* The articulated inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  p_matrix_pseudo_inverse_inertia_pub_->publish(p_pseudo_inverse_with_inertia_msg);
}

rcl_interfaces::msg::SetParametersResult UnderActuatedLQIController::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  std::string prefix = "controller.lqi.";

  // Check if parameter has correct data type
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto asDouble = [&](const rclcpp::Parameter &p, double &out) -> bool
  {
    switch (p.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        out = p.as_double();
        return true;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        out = static_cast<double>(p.as_int());
        return true;
      default:
        result.successful = false;
        result.reason = "Parameter '" + p.get_name() + "' must be a double; got type '" + p.get_type_name() + "'";
        RCLCPP_WARN(node_->get_logger(), "[LQI] %s", result.reason.c_str());
        return false;
    }
  };

  auto asBool = [&](const rclcpp::Parameter &p, bool &out) -> bool
  {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    {
      result.successful = false;
      result.reason = "Parameter '" + p.get_name() + "' must be a bool; got type '" + p.get_type_name() + "'";
      RCLCPP_WARN(node_->get_logger(), "[LQI] %s", result.reason.c_str());
      return false;
    }
    out = p.as_bool();
    return true;
  };

  for (const auto &param : parameters)
  {
    if (param.get_name().find(prefix) != 0) continue;

    std::string key = param.get_name().substr(prefix.size());

    if (key == "z_p")
    {
      if (!asDouble(param, lqi_z_weight_.x())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the P-gain weight of z to %f", lqi_z_weight_.x());
    }
    else if (key == "z_i")
    {
      if (!asDouble(param, lqi_z_weight_.y())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the I-gain weight of z to %f", lqi_z_weight_.y());
    }
    else if (key == "z_d")
    {
      if (!asDouble(param, lqi_z_weight_.z())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the D-gain weight of z to %f", lqi_z_weight_.z());
    }
    else if (key == "roll_pitch_p")
    {
      if (!asDouble(param, lqi_roll_pitch_weight_.x())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the P-gain weight of roll and pitch to %f",
                  lqi_roll_pitch_weight_.x());
    }
    else if (key == "roll_pitch_i")
    {
      if (!asDouble(param, lqi_roll_pitch_weight_.y())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the I-gain weight of roll and pitch to %f",
                  lqi_roll_pitch_weight_.y());
    }
    else if (key == "roll_pitch_d")
    {
      if (!asDouble(param, lqi_roll_pitch_weight_.z())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the D-gain weight of roll and pitch to %f",
                  lqi_roll_pitch_weight_.z());
    }
    else if (key == "yaw_p")
    {
      if (!asDouble(param, lqi_yaw_weight_.x())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the P-gain weight of yaw to %f", lqi_yaw_weight_.x());
    }
    else if (key == "yaw_i")
    {
      if (!asDouble(param, lqi_yaw_weight_.y())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the I-gain weight of yaw to %f", lqi_yaw_weight_.y());
    }
    else if (key == "yaw_d")
    {
      if (!asDouble(param, lqi_yaw_weight_.z())) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed the D-gain weight of yaw to %f", lqi_yaw_weight_.z());
    }
    else if (key == "clamp_gain")
    {
      if (!asBool(param, clamp_gain_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed clamp_gain to %s", clamp_gain_ ? "true" : "false");
    }
    else if (key == "realtime_update")
    {
      if (!asBool(param, realtime_update_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed realtime_update to %s", realtime_update_ ? "true" : "false");
    }
    else if (key == "gyro_moment_compensation")
    {
      if (!asBool(param, gyro_moment_compensation_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[LQI] Changed gyro_moment_compensation to %s",
                  gyro_moment_compensation_ ? "true" : "false");
    }
  }

  if (!realtime_update_)
  {
    // Instantly modify gain if model has no joints, i.e., is fixed
    (void)updateGain(true);
  }
  return result;
}

}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedLQIController, aerial_robot_control::ControlBase);
