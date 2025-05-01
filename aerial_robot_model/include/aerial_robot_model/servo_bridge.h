// src/servo_bridge/servo_bridge.hpp
#ifndef SERVO_BRIDGE_H_
#define SERVO_BRIDGE_H_

#include <kalman_filter/lpf_filter.h>
#include <urdf/model.h>

#include <boost/algorithm/clamp.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spinal/msg/joint_profiles.hpp>
#include <spinal/msg/servo_control_cmd.hpp>
#include <spinal/msg/servo_states.hpp>
#include <spinal/msg/servo_torque_cmd.hpp>
#include <spinal/msg/uav_info.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>

namespace ValueType {
enum : int { BIT = 0, RADIAN = 1 };
}

class SingleServoHandle {
 public:
  SingleServoHandle(const std::string& name, int id, int angle_sgn, double zero_point_offset, double angle_scale,
                    double upper_limit, double lower_limit, double torque_scale, bool receive_real_state,
                    bool filter_flag = false, double sample_freq = 0.0, double cutoff_freq = 0.0)
      : name_(name),
        id_(id),
        curr_angle_val_(0.0),
        target_angle_val_(0.0),
        init_target_angle_val_(false),
        curr_torque_val_(0.0),
        target_torque_val_(0.0),
        angle_sgn_(angle_sgn),
        zero_point_offset_(zero_point_offset),
        angle_scale_(angle_scale),
        upper_limit_(upper_limit),
        lower_limit_(lower_limit),
        torque_scale_(torque_scale),
        receive_real_state_(receive_real_state),
        filter_flag_(filter_flag) {
    if (filter_flag_) {
      if (sample_freq <= 0.0 || cutoff_freq <= 0.0) {
        throw std::runtime_error("filtering config is invalid");
      }
      lpf_angle_ = IirFilter(sample_freq, cutoff_freq, 1);
    }
  }

  ~SingleServoHandle() = default;

  using Ptr = std::shared_ptr<SingleServoHandle>;

  inline void setCurrAngleVal(const double& val, int value_type) {
    if (value_type == ValueType::BIT) {
      curr_angle_val_ = angle_scale_ * angle_sgn_ * (val - zero_point_offset_);
    } else {
      curr_angle_val_ = val;
    }
    if (filter_flag_) {
      curr_angle_val_ = lpf_angle_.filterFunction(curr_angle_val_);
    }
    if (!init_target_angle_val_) {
      target_angle_val_ = std::clamp(curr_angle_val_, lower_limit_, upper_limit_);
      init_target_angle_val_ = true;
    }
  }

  inline void setTargetAngleVal(const double& val, int value_type) {
    if (value_type == ValueType::BIT) {
      target_angle_val_ =
          std::clamp(angle_scale_ * angle_sgn_ * (val - zero_point_offset_), lower_limit_, upper_limit_);
    } else {
      target_angle_val_ = std::clamp(val, lower_limit_, upper_limit_);
    }
    if (!receive_real_state_) {
      curr_angle_val_ = target_angle_val_;
    }
  }

  inline void setCurrTorqueVal(const double& val) { curr_torque_val_ = torque_scale_ * angle_sgn_ * val; }

  inline void setTargetTorqueVal(const double& val) { target_torque_val_ = val; }

  /// 現在角度を BIT (生値) または RADIAN で返す
  inline double getCurrAngleVal(int value_type) const {
    if (value_type == ValueType::BIT) {
      return std::clamp((curr_angle_val_ * angle_sgn_ / angle_scale_) + zero_point_offset_,
                        static_cast<double>(INT16_MIN) / 2.0, static_cast<double>(INT16_MAX) / 2.0);
    }
    return curr_angle_val_;
  }

  /// 目標角度を BIT (生値) または RADIAN で返す
  inline double getTargetAngleVal(int value_type) const {
    if (value_type == ValueType::BIT) {
      return std::clamp((target_angle_val_ * angle_sgn_ / angle_scale_) + zero_point_offset_,
                        static_cast<double>(INT16_MIN) / 2.0, static_cast<double>(INT16_MAX) / 2.0);
    }
    return target_angle_val_;
  }

  /// 現在トルク（Nm）
  inline double getCurrTorqueVal() const { return curr_torque_val_; }

  /// 目標トルクを BIT (生値) または Nm で返す
  inline double getTargetTorqueVal(int value_type) const {
    if (value_type == ValueType::BIT) {
      return (target_torque_val_ * angle_sgn_ / torque_scale_);
    }
    return target_torque_val_;
  }

  // getters (match ROS1版)
  inline const std::string& getName() const { return name_; }
  inline const int& getId() const { return id_; }
  inline const int& getAngleSgn() const { return angle_sgn_; }
  inline const int& getZeroPointOffset() const { return zero_point_offset_; }
  inline const double& getAngleScale() const { return angle_scale_; }
  inline const double& getTorqueScale() const { return torque_scale_; }

 private:
  int id_;
  std::string name_;
  double curr_angle_val_, target_angle_val_;
  double curr_torque_val_, target_torque_val_;
  int angle_sgn_, zero_point_offset_;
  double angle_scale_, upper_limit_, lower_limit_, torque_scale_;
  bool receive_real_state_, filter_flag_, init_target_angle_val_;
  IirFilter lpf_angle_;
};

using SingleServoHandlePtr = SingleServoHandle::Ptr;
using ServoGroupHandler = std::vector<SingleServoHandlePtr>;

class ServoBridge {
 public:
  explicit ServoBridge(rclcpp::Node::SharedPtr node);
  ~ServoBridge() = default;

  /// Publish combined joint_states (non-simulation)
  void servoStatePublish();

 private:
  // callbacks
  void servoStatesCallback(spinal::msg::ServoStates::ConstSharedPtr msg, const std::string& servo_group_name);

  void servoCtrlCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg, const std::string& servo_group_name);

  void servoTorqueCtrlCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg, const std::string& servo_group_name);

  void servoEnableCallback(std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> res, const std::string& servo_group_name);

  void uavInfoCallback(spinal::msg::UavInfo::ConstSharedPtr msg);

  // node handle
  rclcpp::Node::SharedPtr node_;

  // publishers & subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mujoco_control_input_pub_;
  rclcpp::Publisher<spinal::msg::JointProfiles>::SharedPtr joint_profile_pub_;

  rclcpp::Subscription<spinal::msg::UavInfo>::SharedPtr uav_info_sub_;

  std::map<std::string, rclcpp::Subscription<spinal::msg::ServoStates>::SharedPtr> servo_states_subs_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> servo_ctrl_subs_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> servo_torque_ctrl_subs_;

  std::map<std::string, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> servo_enable_srvs_;

  std::map<std::string, rclcpp::Publisher<spinal::msg::ServoControlCmd>::SharedPtr> servo_target_pos_pubs_;
  std::map<std::string, rclcpp::Publisher<spinal::msg::ServoControlCmd>::SharedPtr> servo_target_torque_pubs_;
  std::map<std::string, rclcpp::Publisher<spinal::msg::ServoTorqueCmd>::SharedPtr> servo_enable_pubs_;

  // simulation-only: individual float64 command pubs
  std::map<std::string, std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>> servo_target_pos_sim_pubs_;

  // servo handlers & flags
  std::map<std::string, ServoGroupHandler> servos_handler_;
  std::map<std::string, bool> no_real_state_flags_;

  // configuration
  bool simulation_mode_;
  bool use_mujoco_;
  double moving_check_rate_;
  double moving_angle_thresh_;
  bool send_init_joint_pose_;
  int send_init_joint_pose_cnt_;
};

#endif  // SERVO_BRIDGE_H_
