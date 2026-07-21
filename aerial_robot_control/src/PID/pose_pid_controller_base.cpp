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
#include "aerial_robot_control/PID/pose_pid_controller_base.hpp"

namespace aerial_robot_control
{

PosePIDControllerBase::PosePIDControllerBase()
  : ControlBase(),
    pid_controllers_(0),
    pos_(0, 0, 0),
    target_pos_(0, 0, 0),
    vel_(0, 0, 0),
    target_vel_(0, 0, 0),
    rpy_(0, 0, 0),
    target_rpy_(0, 0, 0),
    target_acc_(0, 0, 0),
    target_omega_(0, 0, 0),
    start_roll_pitch_integration_(false)
{
  pid_msg_.x.total.resize(1);
  pid_msg_.x.p_term.resize(1);
  pid_msg_.x.i_term.resize(1);
  pid_msg_.x.d_term.resize(1);
  pid_msg_.y.total.resize(1);
  pid_msg_.y.p_term.resize(1);
  pid_msg_.y.i_term.resize(1);
  pid_msg_.y.d_term.resize(1);
  pid_msg_.z.total.resize(1);
  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);
  pid_msg_.roll.total.resize(1);
  pid_msg_.roll.p_term.resize(1);
  pid_msg_.roll.i_term.resize(1);
  pid_msg_.roll.d_term.resize(1);
  pid_msg_.pitch.total.resize(1);
  pid_msg_.pitch.p_term.resize(1);
  pid_msg_.pitch.i_term.resize(1);
  pid_msg_.pitch.d_term.resize(1);
  pid_msg_.yaw.total.resize(1);
  pid_msg_.yaw.p_term.resize(1);
  pid_msg_.yaw.i_term.resize(1);
  pid_msg_.yaw.d_term.resize(1);
}

void PosePIDControllerBase::initialize(rclcpp::Node::SharedPtr node,
                                       std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                                       double ctrl_loop_dt)
{
  ControlBase::initialize(node, robot_model, estimator, navigator, ctrl_loop_dt);
  // ------------------------------------------------------------------

  // ------------------------------------------------------------------
  const std::string prefix = "controller.";
  const std::string xy_ns = prefix + "xy";
  const std::string x_ns = prefix + "x";
  const std::string y_ns = prefix + "y";
  const std::string z_ns = prefix + "z";

  const std::string roll_pitch_ns = prefix + "roll_pitch";
  const std::string roll_ns = prefix + "roll";
  const std::string pitch_ns = prefix + "pitch";
  const std::string yaw_ns = prefix + "yaw";

  double limit_sum, limit_p, limit_i, limit_d;
  double limit_err_p, limit_err_i, limit_err_d;
  double p_gain, i_gain, d_gain;

  // ------------------------------------------------------------------
  // Load PID parameters
  // ------------------------------------------------------------------
  auto loadParam = [&](const std::string &ns)
  {
    getParam<double>(ns + ".limit_sum", limit_sum, 1.0e6);
    getParam<double>(ns + ".limit_p", limit_p, 1.0e6);
    getParam<double>(ns + ".limit_i", limit_i, 1.0e6);
    getParam<double>(ns + ".limit_d", limit_d, 1.0e6);
    getParam<double>(ns + ".limit_err_p", limit_err_p, 1.0e6);
    getParam<double>(ns + ".limit_err_i", limit_err_i, 1.0e6);
    getParam<double>(ns + ".limit_err_d", limit_err_d, 1.0e6);

    getParam<double>(ns + ".p_gain", p_gain, 0.0);
    getParam<double>(ns + ".i_gain", i_gain, 0.0);
    getParam<double>(ns + ".d_gain", d_gain, 0.0);
  };

  /* X & Y */
  if (node_->has_parameter(xy_ns + ".p_gain"))
  {
    loadParam(xy_ns);
    pid_controllers_.push_back(
        PID("x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
    pid_controllers_.push_back(
        PID("y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  }
  else
  {
    loadParam(x_ns);
    pid_controllers_.push_back(
        PID("x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

    loadParam(y_ns);
    pid_controllers_.push_back(
        PID("y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  }

  /* Z */
  getParam<double>(z_ns + ".landing_err_z", landing_err_z_, -0.5);
  getParam<double>(z_ns + ".safe_landing_height", safe_landing_height_, 0.1);
  getParam<double>(z_ns + ".force_landing_descending_rate", force_landing_descending_rate_, -0.1);
  if (force_landing_descending_rate_ >= 0) force_landing_descending_rate_ = -0.1;

  loadParam(z_ns);
  pid_controllers_.push_back(
      PID("z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

  /* Roll & Pitch */
  getParam<double>(roll_pitch_ns + ".start_integration_height", start_roll_pitch_integration_height_, 0.01);

  // PID.yaml always declares the shared roll_pitch parameters.  Prefer explicit
  // per-axis parameters supplied by an airframe so that legacy configurations
  // with different roll and pitch gains are not silently masked by the common
  // defaults.
  if (node_->has_parameter(roll_ns + ".p_gain") && node_->has_parameter(pitch_ns + ".p_gain"))
  {
    loadParam(roll_ns);
    pid_controllers_.push_back(PID("roll", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p,
                                   limit_err_i, limit_err_d));

    loadParam(pitch_ns);
    pid_controllers_.push_back(PID("pitch", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p,
                                   limit_err_i, limit_err_d));
  }
  else
  {
    loadParam(roll_pitch_ns);
    pid_controllers_.push_back(PID("roll", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p,
                                   limit_err_i, limit_err_d));
    pid_controllers_.push_back(PID("pitch", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p,
                                   limit_err_i, limit_err_d));
  }

  /* Yaw */
  loadParam(yaw_ns);
  getParam<bool>(yaw_ns + ".need_d_control", need_yaw_d_control_, false);
  pid_controllers_.push_back(
      PID("yaw", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

  // ------------------------------------------------------------------
  // Parameter-change callback
  // ------------------------------------------------------------------
  param_cb_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&PosePIDControllerBase::parametersCallback, this, std::placeholders::_1));

  // ------------------------------------------------------------------
  // Publisher
  // ------------------------------------------------------------------
  pid_pub_ = node_->create_publisher<aerial_robot_msgs::msg::PoseControlPid>("debug/pose/pid", 10);
}

void PosePIDControllerBase::reset()
{
  ControlBase::reset();
  start_roll_pitch_integration_ = false;

  for (auto &controller : pid_controllers_) controller.reset();

  target_pos_ = KDL::Vector(0, 0, 0);
  target_vel_ = KDL::Vector(0, 0, 0);
  target_acc_ = KDL::Vector(0, 0, 0);
  target_rpy_ = KDL::Vector(0, 0, 0);
  target_omega_ = KDL::Vector(0, 0, 0);
}

bool PosePIDControllerBase::update()
{
  if (!ControlBase::update()) return false;

  // Only execute controller if the core gives the green light and sets the first timestamp
  controlCore();
  sendCmd();

  return true;
}

void PosePIDControllerBase::controlCore()
{
  // ------------------------------------------------------------------
  // Get current state from estimator
  // ------------------------------------------------------------------
  pos_ = estimator_->getCogPos(estimate_mode_);
  rpy_ = estimator_->getCogEuler(estimate_mode_);
  cog_rot_ = estimator_->getCogOrientation(estimate_mode_);
  vel_ = estimator_->getCogVel(estimate_mode_);
  omega_ = estimator_->getCogAngularVel(estimate_mode_);

  // ------------------------------------------------------------------
  // Get target state from navigator
  // ------------------------------------------------------------------
  target_pos_ = navigator_->getTargetCogPos();
  target_rpy_ = navigator_->getTargetCogRPY();
  target_vel_ = navigator_->getTargetCogVel();
  target_acc_ = navigator_->getTargetCogAcc();
  target_ang_acc_ = navigator_->getTargetCogAngularAcc();

  KDL::Vector target_omega_raw = navigator_->getTargetCogOmega();  // W.r.t. target CoG
  target_rot_ = KDL::Rotation::RPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
  target_omega_ = cog_rot_.Inverse() * target_rot_ * target_omega_raw;  // W.r.t. current CoG

  // ------------------------------------------------------------------
  // Call PID controller for each axis and compute control output
  // by multiplying the state error with the corresponding PID gains
  // ------------------------------------------------------------------
  /* Time step from last iteration */
  const double dt = node_->now().seconds() - control_timestamp_;

  /* X & Y */
  const double err_x = target_pos_.x() - pos_.x();  // for P & I terms
  const double err_y = target_pos_.y() - pos_.y();
  const double err_v_x = target_vel_.x() - vel_.x();  // for D term
  const double err_v_y = target_vel_.y() - vel_.y();
  switch (navigator_->getXyControlMode())
  {
    case aerial_robot_navigation::POS_CONTROL_MODE:
      pid_controllers_.at(X).update(err_x, err_v_x, target_acc_.x(), dt);  // target_acc for feedforward term
      pid_controllers_.at(Y).update(err_y, err_v_y, target_acc_.y(), dt);
      break;
    case aerial_robot_navigation::VEL_CONTROL_MODE:
      pid_controllers_.at(X).update(0, err_v_x, target_acc_.x(), dt);
      pid_controllers_.at(Y).update(0, err_v_y, target_acc_.y(), dt);
      break;
    case aerial_robot_navigation::ACC_CONTROL_MODE:
      pid_controllers_.at(X).update(0, 0, target_acc_.x(), dt);
      pid_controllers_.at(Y).update(0, 0, target_acc_.y(), dt);
      break;
    default:
      break;
  }

  if (navigator_->getForceLandingFlag())
  {
    // Reset I-term
    pid_controllers_.at(X).reset();
    pid_controllers_.at(Y).reset();
  }

  /* Z */
  double err_z = target_pos_.z() - pos_.z();
  double err_v_z = target_vel_.z() - vel_.z();
  double dt_z = dt;
  const double z_p_limit = pid_controllers_.at(Z).getLimitP();
  bool final_landing_phase = false;

  if (navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE)
  {
    if (-err_z > safe_landing_height_)
    {
      // Robot is too high, descend slowly
      err_z = landing_err_z_;
      if (vel_.z() < landing_err_z_)
      {
        // Freeze I-term when descending slowly to avoid windup
        dt_z = 0;
      }
    }
    else
    {
      // Zero out P-term in final safe landing phase
      pid_controllers_.at(Z).setLimitP(0);  // TODO: does this make sense? Does this make the robot hover at landing
                                            // height?
      final_landing_phase = true;
    }
  }

  if (navigator_->getForceLandingFlag())
  {
    // Zero out P-term during force landing
    pid_controllers_.at(Z).setLimitP(0);
    err_z = force_landing_descending_rate_;
    err_v_z = 0;
    target_acc_[2] = 0;
  }

  pid_controllers_.at(Z).update(err_z, err_v_z, target_acc_.z(), dt_z);

  if (pid_controllers_.at(Z).getErrI() < 0) pid_controllers_.at(Z).setErrI(0);

  if (navigator_->getForceLandingFlag() || final_landing_phase)
  {
    pid_controllers_.at(Z).setLimitP(z_p_limit);
    pid_controllers_.at(Z).setErrP(0);
  }

  /* Roll & Pitch */
  double dt_roll_pitch = dt;
  if (!start_roll_pitch_integration_)
  {
    if (pos_.z() - navigator_->getInitHeight() > start_roll_pitch_integration_height_)
    {
      start_roll_pitch_integration_ = true;

      spinal_msgs::msg::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal_msgs::msg::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
      navigator_->getFlightConfigPublisher()->publish(flight_config_cmd);

      RCLCPP_WARN(node_->get_logger(), "[PID] Starting roll & pitch I-term control");
    }
    // Freeze I-term to avoid windup before takeoff
    dt_roll_pitch = 0;
  }

  const double err_roll = angles::shortest_angular_distance(rpy_.x(), target_rpy_.x());
  const double err_pitch = angles::shortest_angular_distance(rpy_.y(), target_rpy_.y());
  const double err_omega_x = target_omega_.x() - omega_.x();
  const double err_omega_y = target_omega_.y() - omega_.y();
  pid_controllers_.at(ROLL).update(err_roll, err_omega_x, target_ang_acc_.x(), dt_roll_pitch);
  pid_controllers_.at(PITCH).update(err_pitch, err_omega_y, target_ang_acc_.y(), dt_roll_pitch);

  /* Yaw */
  const double err_yaw = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
  double err_omega_z = target_omega_.z() - omega_.z();
  if (!need_yaw_d_control_)
  {
    err_omega_z = target_omega_.z();
  }  // Remainder handled in spinal
  pid_controllers_.at(YAW).update(err_yaw, err_omega_z, target_ang_acc_.z(), dt);

  /* Update timestamp */
  control_timestamp_ = node_->now().seconds();

  // ------------------------------------------------------------------
  // Build debug message
  // ------------------------------------------------------------------
  pid_msg_.header.stamp = rclcpp::Time(static_cast<int64_t>(estimator_->getImuLatestTimeStamp() * 1e9));

  pid_msg_.x.total.at(0) = pid_controllers_.at(X).result();
  pid_msg_.x.p_term.at(0) = pid_controllers_.at(X).getPTerm();
  pid_msg_.x.i_term.at(0) = pid_controllers_.at(X).getITerm();
  pid_msg_.x.d_term.at(0) = pid_controllers_.at(X).getDTerm();
  pid_msg_.x.target_p = target_pos_.x();
  pid_msg_.x.err_p = target_pos_.x() - pos_.x();
  pid_msg_.x.target_d = target_vel_.x();
  pid_msg_.x.err_d = target_vel_.x() - vel_.x();

  pid_msg_.y.total.at(0) = pid_controllers_.at(Y).result();
  pid_msg_.y.p_term.at(0) = pid_controllers_.at(Y).getPTerm();
  pid_msg_.y.i_term.at(0) = pid_controllers_.at(Y).getITerm();
  pid_msg_.y.d_term.at(0) = pid_controllers_.at(Y).getDTerm();
  pid_msg_.y.target_p = target_pos_.y();
  pid_msg_.y.err_p = target_pos_.y() - pos_.y();
  pid_msg_.y.target_d = target_vel_.y();
  pid_msg_.y.err_d = target_vel_.y() - vel_.y();

  pid_msg_.z.total.at(0) = pid_controllers_.at(Z).result();
  pid_msg_.z.p_term.at(0) = pid_controllers_.at(Z).getPTerm();
  pid_msg_.z.i_term.at(0) = pid_controllers_.at(Z).getITerm();
  pid_msg_.z.d_term.at(0) = pid_controllers_.at(Z).getDTerm();
  pid_msg_.z.target_p = target_pos_.z();
  pid_msg_.z.err_p = target_pos_.z() - pos_.z();
  pid_msg_.z.target_d = target_vel_.z();
  pid_msg_.z.err_d = target_vel_.z() - vel_.z();

  pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = target_rpy_.x() - rpy_.x();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = target_omega_.x() - omega_.x();

  pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = target_rpy_.y() - rpy_.y();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = target_omega_.y() - omega_.y();

  pid_msg_.yaw.total.at(0) = pid_controllers_.at(YAW).result();
  pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(YAW).getPTerm();
  pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(YAW).getITerm();
  pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(YAW).getDTerm();
  pid_msg_.yaw.target_p = target_rpy_.z();
  pid_msg_.yaw.err_p = err_yaw;
  pid_msg_.yaw.target_d = target_omega_.z();
  pid_msg_.yaw.err_d = target_omega_.z() - omega_.z();
}

// -------------------------------------------------------------------------
// Publish debug message
// -------------------------------------------------------------------------
void PosePIDControllerBase::sendCmd() { pid_pub_->publish(pid_msg_); }

// -------------------------------------------------------------------------
// Helper function for child classes:
// Compute the pseudo-inverse of the wrench allocation matrix Q for feedforward control
// -------------------------------------------------------------------------
Eigen::MatrixXd PosePIDControllerBase::getQInv()
{
  const std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto &rotor_direction = robot_model_->getRotorDirection();
  const double m_f_rate = robot_model_->getMFRate();
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::MatrixXd q_mat_ = Eigen::MatrixXd::Zero(4, motor_num_);
  for (unsigned int i = 0; i < motor_num_; ++i)
  {
    q_mat_(0, i) = rotors_normal.at(i).z() / robot_model_->getMass();
    q_mat_.block(1, i, 3, 1) = inertia.inverse() * (rotors_origin.at(i).cross(rotors_normal.at(i)) +
                                                    m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
  }
  return aerial_robot_model::pseudoinverse(q_mat_);
}

// -------------------------------------------------------------------------
// Callback function for updating PID parameters
// NOTE: Every node acts as its own parameter server and can be dynamically
// reconfigured at runtime using standard parameter services or the CLI
// -------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult PosePIDControllerBase::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  std::string prefix = "controller.";

  // Map axis name to controller index
  static const std::unordered_map<std::string, int> name_to_idx = {
    { "x", X }, { "y", Y }, { "z", Z }, { "roll", ROLL }, { "pitch", PITCH }, { "yaw", YAW }
  };

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
        RCLCPP_WARN(node_->get_logger(), "[PID] %s", result.reason.c_str());
        return false;
    }
  };

  auto asBool = [&](const rclcpp::Parameter &p, bool &out) -> bool
  {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    {
      result.successful = false;
      result.reason = "Parameter '" + p.get_name() + "' must be a bool; got type '" + p.get_type_name() + "'";
      RCLCPP_WARN(node_->get_logger(), "[PID] %s", result.reason.c_str());
      return false;
    }
    out = p.as_bool();
    return true;
  };

  for (const auto &param : parameters)
  {
    // Expected pattern: "controller.<axis>.{p,i,d}_gain"
    // e.g. "controller.x.p_gain"
    std::string param_id = param.get_name().substr(prefix.size());  // "<axis>.p_gain"
    auto dot = param_id.find('.');
    std::string axis = param_id.substr(0, dot);
    std::string key = param_id.substr(dot + 1);

    // Shared axes ("xy", "roll_pitch") treated as special cases
    std::vector<int> indices;
    if (axis == "xy")
    {
      indices = { X, Y };
    }
    else if (axis == "roll_pitch")
    {
      indices = { ROLL, PITCH };
    }
    else
    {
      auto iterator = name_to_idx.find(axis);
      // "find()" returns "end()" if key not present / "->second" gets value from map entry
      if (iterator != name_to_idx.end()) indices = { iterator->second };
    }

    if (indices.empty()) continue;

    bool is_double_key = (key == "p_gain" || key == "i_gain" || key == "d_gain" || key == "limit_sum" ||
                          key == "limit_p" || key == "limit_i" || key == "limit_d" || key == "limit_err_p" ||
                          key == "limit_err_i" || key == "limit_err_d");
    double value = 0.0;
    if (is_double_key && !asDouble(param, value)) return result;

    // Update the corresponding PID controller in each axis
    for (int idx : indices)
    {
      if (key == "p_gain")
      {
        pid_controllers_.at(idx).setPGain(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed p_gain for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "i_gain")
      {
        pid_controllers_.at(idx).setIGain(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed i_gain for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "d_gain")
      {
        pid_controllers_.at(idx).setDGain(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed d_gain for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_sum")
      {
        pid_controllers_.at(idx).setLimitSum(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_sum for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_p")
      {
        pid_controllers_.at(idx).setLimitP(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_p for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_i")
      {
        pid_controllers_.at(idx).setLimitI(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_i for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_d")
      {
        pid_controllers_.at(idx).setLimitD(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_d for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_err_p")
      {
        pid_controllers_.at(idx).setLimitErrP(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_err_p for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_err_i")
      {
        pid_controllers_.at(idx).setLimitErrI(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_err_i for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
      else if (key == "limit_err_d")
      {
        pid_controllers_.at(idx).setLimitErrD(value);
        RCLCPP_INFO(node_->get_logger(), "[PID] Changed limit_err_d for controller '%s' to %f",
                    pid_controllers_.at(idx).getName().c_str(), value);
      }
    }

    if (param.get_name() == "controller.z.landing_err_z")
    {
      if (!asDouble(param, landing_err_z_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[PID] Changed landing_err_z to %f", value);
      continue;
    }
    if (param.get_name() == "controller.z.safe_landing_height")
    {
      if (!asDouble(param, safe_landing_height_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[PID] Changed safe_landing_height to %f", value);
      continue;
    }
    if (param.get_name() == "controller.z.force_landing_descending_rate")
    {
      if (!asDouble(param, force_landing_descending_rate_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[PID] Changed force_landing_descending_rate to %f", value);
      continue;
    }
    if (param.get_name() == "controller.roll_pitch.start_integration_height")
    {
      if (!asDouble(param, start_roll_pitch_integration_height_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[PID] Changed start_integration_height to %f", value);
      continue;
    }
    if (param.get_name() == "controller.yaw.need_d_control")
    {
      if (!asBool(param, need_yaw_d_control_)) return result;
      RCLCPP_INFO(node_->get_logger(), "[PID] Changed need_d_control to %f", value);
      continue;
    }
  }
  return result;
}

}
