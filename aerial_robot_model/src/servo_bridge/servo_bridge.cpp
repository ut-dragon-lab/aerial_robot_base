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

#include "aerial_robot_model/servo_bridge.h"

#include <urdf/model.h>

#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

ServoBridge::ServoBridge(rclcpp::Node::SharedPtr node) : node_(node) {
  // 1) Simulation / mujoco flags
  node_->get_parameter_or("sim", simulation_mode_, false);
  node_->get_parameter_or("use_mujoco", use_mujoco_, false);
  if (use_mujoco_) {
    RCLCPP_WARN(node_->get_logger(), "use mujoco simulator");
    simulation_mode_ = false;
  }
  if (simulation_mode_) {
    auto cli = node_->create_client<controller_manager_msgs::srv::LoadController>(
        node_->get_namespace() + std::string("/controller_manager/load_controller"));
    if (!cli->wait_for_service(5s)) {
      RCLCPP_ERROR(node_->get_logger(), "cannot find %s; aborting", cli->get_service_name());
      return;
    }
  }

  // 2) Load URDF
  std::string urdf_xml;
  node_->get_parameter_or("robot_description", urdf_xml, std::string(""));
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse robot_description URDF");
  }

  // 3) Groups (always include "common")
  auto groups =
      node_->get_parameter_or<std::vector<std::string>>("servo_controller.groups", std::vector<std::string>{"common"});
  if (std::find(groups.begin(), groups.end(), "common") == groups.end()) {
    groups.insert(groups.begin(), "common");
  }

  // 4) Per‐group setup
  for (const auto& group : groups) {
    const std::string base = "servo_controller." + group + ".";
    // parameters
    bool no_real;
    node_->get_parameter_or(base + "no_real_state", no_real, false);
    no_real_state_flags_[group] = no_real;

    std::string state_sub_topic;
    node_->get_parameter_or(base + "state_sub_topic", state_sub_topic,
                            std::string(group == "common" ? "servo/states" : ""));
    std::string pos_pub_topic;
    node_->get_parameter_or(base + "pos_pub_topic", pos_pub_topic,
                            std::string(group == "common" ? "servo/target_states" : ""));
    std::string torque_pub_topic;
    node_->get_parameter_or(base + "torque_pub_topic", torque_pub_topic,
                            std::string(group == "common" ? "servo/target_current" : ""));
    std::string enable_pub_topic;
    node_->get_parameter_or(base + "servo_enable_pub_topic", enable_pub_topic,
                            std::string(group == "common" ? "servo/torque_enable" : ""));
    int angle_sgn_cmn;
    node_->get_parameter_or(base + "angle_sgn", angle_sgn_cmn, 1);
    int zero_offset_cmn;
    node_->get_parameter_or(base + "zero_point_offset", zero_offset_cmn, 0);
    double angle_scale_cmn;
    node_->get_parameter_or(base + "angle_scale", angle_scale_cmn, 1.0);
    double torque_scale_cmn;
    node_->get_parameter_or(base + "torque_scale", torque_scale_cmn, 1.0);
    bool filter_flag_cmn;
    node_->get_parameter_or(base + "filter_flag", filter_flag_cmn, false);
    double sample_freq_cmn;
    node_->get_parameter_or(base + "sample_freq", sample_freq_cmn, 0.0);
    double cutoff_freq_cmn;
    node_->get_parameter_or(base + "cutoff_freq", cutoff_freq_cmn, 0.0);
    double init_value_cmn;
    node_->get_parameter_or(base + "init_value", init_value_cmn, 0.0);

    //--- common‐group interfaces ---
    if (!state_sub_topic.empty()) {
      servo_states_subs_[group] = node_->create_subscription<spinal_msgs::msg::ServoStates>(
          state_sub_topic, rclcpp::SystemDefaultsQoS(),
          [this, group](spinal_msgs::msg::ServoStates::ConstSharedPtr msg) { this->servoStatesCallback(msg, group); });
    }
    if (!pos_pub_topic.empty()) {
      servo_target_pos_pubs_[group] =
          node_->create_publisher<spinal_msgs::msg::ServoControlCmd>(pos_pub_topic, rclcpp::SystemDefaultsQoS());
    }
    if (!torque_pub_topic.empty()) {
      servo_target_torque_pubs_[group] =
          node_->create_publisher<spinal_msgs::msg::ServoControlCmd>(torque_pub_topic, rclcpp::SystemDefaultsQoS());
    }
    if (!enable_pub_topic.empty()) {
      servo_enable_pubs_[group] =
          node_->create_publisher<spinal_msgs::msg::ServoTorqueCmd>(enable_pub_topic, rclcpp::SystemDefaultsQoS());
    }
    // service for torque‐enable
    servo_enable_srvs_[group] = node_->create_service<std_srvs::srv::SetBool>(
        group + "/torque_enable", [this, group](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          this->servoEnableCallback(req, res, group);
        });

    //--- per‐servo controllers in this group ---
    auto joints = node_->get_parameter_or<std::vector<std::string>>("servo_controller." + group + ".names",
                                                                    std::vector<std::string>{});

    ServoGroupHandler group_handler;
    for (const auto& joint : joints) {
      const std::string jbase = base + joint + ".";
      int id, a_sgn, z_off;
      double a_scale, t_scale, s_freq, c_freq, init_value;
      bool f_flag;
      node_->get_parameter_or(jbase + "id", id, 0);
      node_->get_parameter_or(jbase + "angle_sgn", a_sgn, angle_sgn_cmn);
      node_->get_parameter_or(jbase + "zero_point_offset", z_off, zero_offset_cmn);
      node_->get_parameter_or(jbase + "angle_scale", a_scale, angle_scale_cmn);
      node_->get_parameter_or(jbase + "torque_scale", t_scale, torque_scale_cmn);
      node_->get_parameter_or(jbase + "filter_flag", f_flag, filter_flag_cmn);
      node_->get_parameter_or(jbase + "sample_freq", s_freq, sample_freq_cmn);
      node_->get_parameter_or(jbase + "cutoff_freq", c_freq, cutoff_freq_cmn);
      node_->get_parameter_or(jbase + "init_value", init_value, init_value_cmn);

      // get joint info from URDF
      auto j = urdf_model.getJoint(joint);
      if (!j) {
        RCLCPP_ERROR(node_->get_logger(), "ServoBridge: Joint '%s' not found in URDF, skipping controller setup",
                     joint.c_str());
        continue;
      }
      if (!j->limits) {
        RCLCPP_ERROR(node_->get_logger(), "ServoBridge: Joint '%s' has no <limit> element in URDF, skipping",
                     joint.c_str());
        continue;
      }
      double upper = j->limits->upper;
      double lower = j->limits->lower;

      auto single_hander = std::make_shared<SingleServoHandle>(joint, id, a_sgn, z_off, a_scale, upper, lower, t_scale,
                                                               !no_real_state_flags_.at(group), f_flag, s_freq, c_freq);

      // register initial target position for each joint (only for simulation)
      if (simulation_mode_) {
        single_hander->setTargetAngleVal(init_value, ValueType::RADIAN);
      }

      group_handler.push_back(single_hander);
    }
    servos_handler_[group] = std::move(group_handler);

    servo_ctrl_subs_.insert(
        std::make_pair(group, node_->create_subscription<sensor_msgs::msg::JointState>(
                                  group + "_ctrl", rclcpp::SystemDefaultsQoS(),
                                  [this, group](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                                    this->servoCtrlCallback(msg, group);
                                  })));
    if (simulation_mode_ && group != "common") {
      uint64_t depth = rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE;
      auto result = node_->list_parameters({"servo_controller." + group + ".simulation"}, depth);
      if (result.names.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "please set gazebo servo parameters for %s, using sub namespace 'simulation:'", group.c_str());
        return;
      }
      std::string controller_type;
      node_->get_parameter_or("servo_controller." + group + ".simulation.type", controller_type, std::string(""));
      servo_target_pos_sim_pubs_.insert(
          std::make_pair(group, node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                                    controller_type + "/commands", rclcpp::SystemDefaultsQoS())));

      // send initial position
      auto pos_sim_msg = std_msgs::msg::Float64MultiArray();
      for (const auto& it : servos_handler_[group]) {
        pos_sim_msg.data.push_back(it->getTargetAngleVal(ValueType::RADIAN));
      }
      auto pub = servo_target_pos_sim_pubs_[group];

      // wait until subscribers are ready
      while (rclcpp::ok() && pub->get_subscription_count() == 0) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for joints control subscribers...");
        rclcpp::Rate(10).sleep();
      }
      pub->publish(pos_sim_msg);
    }
  }

  // 5) joint_states publisher
  if (!simulation_mode_) {
    servo_states_pub_ =
        node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS());
  }
  // 6) mujoco output
  mujoco_control_input_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("mujoco/ctrl_input", rclcpp::SystemDefaultsQoS());
  // 7) joint_profile output
  joint_profile_pub_ =
      node_->create_publisher<spinal_msgs::msg::JointProfiles>("joint_profiles", rclcpp::SystemDefaultsQoS());
  // 8) UAV info subscriber
  uav_info_sub_ = node_->create_subscription<spinal_msgs::msg::UavInfo>(
      "uav_info", rclcpp::SystemDefaultsQoS(), std::bind(&ServoBridge::uavInfoCallback, this, std::placeholders::_1));
}

// --- servoStatesCallback ---------------------------------------------------
void ServoBridge::servoStatesCallback(const spinal_msgs::msg::ServoStates::ConstSharedPtr state_msg,
                                      const std::string& servo_group_name) {
  if (servo_group_name != "common") {
    auto& handlers = servos_handler_[servo_group_name];
    for (const auto& it : state_msg->servos) {
      auto it_hdl = std::find_if(handlers.begin(), handlers.end(),
                                 [&](const SingleServoHandlePtr& h) { return it.index == h->getId(); });
      if (it_hdl == handlers.end()) {
        RCLCPP_ERROR(node_->get_logger(), "[servo bridge, state cb]: no handler for servo index %d", it.index);
        return;
      }
      (*it_hdl)->setCurrAngleVal(static_cast<double>(it.angle), ValueType::BIT);
      (*it_hdl)->setCurrTorqueVal(static_cast<double>(it.load));
    }
    return;
  }

  for (const auto& it : state_msg->servos) {
    for (auto& [group, handlers] : servos_handler_) {
      if (no_real_state_flags_.at(group)) {
        RCLCPP_DEBUG(node_->get_logger(), "%s: no_real_state, skip", group.c_str());
        continue;
      }

      if (servo_states_subs_.count(group)) {
        RCLCPP_DEBUG(node_->get_logger(), "%s: has own state sub, skip common", group.c_str());
        continue;
      }
      auto it_hdl = std::find_if(handlers.begin(), handlers.end(),
                                 [&](const SingleServoHandlePtr& h) { return it.index == h->getId(); });
      if (it_hdl == handlers.end()) continue;

      (*it_hdl)->setCurrAngleVal(static_cast<double>(it.angle), ValueType::BIT);
      (*it_hdl)->setCurrTorqueVal(static_cast<double>(it.load));
      RCLCPP_DEBUG(node_->get_logger(), "servo index %d found in group %s", it.index, group.c_str());
      break;
    }
  }

  auto out = sensor_msgs::msg::JointState();
  out.header.stamp = node_->now();
  for (const auto& [group, handlers] : servos_handler_) {
    for (const auto& h : handlers) {
      out.name.push_back(h->getName());
      out.position.push_back(h->getCurrAngleVal(ValueType::RADIAN));
      out.effort.push_back(h->getCurrTorqueVal());
    }
  }
  servo_states_pub_->publish(out);
}

// --- servoCtrlCallback -----------------------------------------------------
void ServoBridge::servoCtrlCallback(const sensor_msgs::msg::JointState::ConstSharedPtr servo_ctrl_msg,
                                    const std::string& servo_group_name) {
  auto target_angle_msg = spinal_msgs::msg::ServoControlCmd();
  auto target_torque_msg = spinal_msgs::msg::ServoControlCmd();
  auto mujoco_msg = sensor_msgs::msg::JointState();

  const auto& names = servo_ctrl_msg->name;
  const auto& pos = servo_ctrl_msg->position;
  const auto& eff = servo_ctrl_msg->effort;

  if (!names.empty()) {
    // named
    if (pos.size() != names.size()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[servo bridge, servo control control]: the servo position num and name num are different in ros "
                   "msgs [%zu vs %zu]",
                   names.size(), pos.size());
      return;
    }
    for (size_t i = 0; i < names.size(); ++i) {
      mujoco_msg.name.push_back(names[i]);
      mujoco_msg.position.push_back(pos[i]);

      // handler lookup
      auto& handlers = servos_handler_[servo_group_name];
      auto it_hdl = std::find_if(handlers.begin(), handlers.end(),
                                 [&](const SingleServoHandlePtr& h) { return names[i] == h->getName(); });
      if (it_hdl == handlers.end()) {
        RCLCPP_ERROR(node_->get_logger(), "[servo bridge, servo control callback]: no matching servo handler for %s",
                     names[i].c_str());
        return;
      }

      (*it_hdl)->setTargetAngleVal(pos[i], ValueType::RADIAN);
      target_angle_msg.index.push_back((*it_hdl)->getId());
      target_angle_msg.angles.push_back((*it_hdl)->getTargetAngleVal(ValueType::BIT));

      if (eff.size() == names.size()) {
        (*it_hdl)->setTargetTorqueVal(eff[i]);
        target_torque_msg.index.push_back((*it_hdl)->getId());
        target_torque_msg.angles.push_back((*it_hdl)->getTargetTorqueVal(ValueType::BIT));
      }
    }

    if (simulation_mode_) {
      auto pos_sim_msg = std_msgs::msg::Float64MultiArray();
      for (const auto& it : servos_handler_[servo_group_name]) {
        pos_sim_msg.data.push_back(it->getTargetAngleVal(ValueType::RADIAN));
      }
      auto pub = servo_target_pos_sim_pubs_[servo_group_name];
      pub->publish(pos_sim_msg);
    }
  } else {
    // unnamed: pre-defined order
    if (pos.size() != servos_handler_[servo_group_name].size()) {
      RCLCPP_ERROR(node_->get_logger(), "[servo bridge, ctrl cb]: expected %zu positions, got %zu",
                   servos_handler_[servo_group_name].size(), pos.size());
      return;
    }
    for (size_t i = 0; i < pos.size(); ++i) {
      auto& h = servos_handler_[servo_group_name][i];
      h->setTargetAngleVal(pos[i], ValueType::RADIAN);
      target_angle_msg.index.push_back(h->getId());
      target_angle_msg.angles.push_back(h->getTargetAngleVal(ValueType::BIT));
      if (eff.size() == pos.size()) {
        h->setTargetTorqueVal(eff[i]);
        target_torque_msg.index.push_back(h->getId());
        target_torque_msg.angles.push_back(h->getTargetTorqueVal(ValueType::BIT));
      }
      mujoco_msg.name.push_back(h->getName());
      mujoco_msg.position.push_back(pos[i]);
      if (simulation_mode_) {
        auto pos_sim_msg = std_msgs::msg::Float64MultiArray();
        for (const auto& it : servos_handler_[servo_group_name]) {
          pos_sim_msg.data.push_back(it->getTargetAngleVal(ValueType::RADIAN));
        }
        auto pub = servo_target_pos_sim_pubs_[servo_group_name];
        pub->publish(pos_sim_msg);
      }
    }
  }

  mujoco_control_input_pub_->publish(mujoco_msg);
  auto& pos_pub = servo_target_pos_pubs_.count(servo_group_name) ? servo_target_pos_pubs_[servo_group_name]
                                                                 : servo_target_pos_pubs_["common"];
  pos_pub->publish(target_angle_msg);

  if (!target_torque_msg.index.empty()) {
    auto& torque_pub = servo_target_torque_pubs_.count(servo_group_name) ? servo_target_torque_pubs_[servo_group_name]
                                                                         : servo_target_torque_pubs_["common"];
    torque_pub->publish(target_torque_msg);
  }
}

// --- servoTorqueCtrlCallback ------------------------------------------------
void ServoBridge::servoTorqueCtrlCallback(const sensor_msgs::msg::JointState::ConstSharedPtr servo_ctrl_msg,
                                          const std::string& servo_group_name) {
  auto target_torque_msg = spinal_msgs::msg::ServoControlCmd();
  const auto& names = servo_ctrl_msg->name;
  const auto& eff = servo_ctrl_msg->effort;

  if (!names.empty()) {
    if (eff.size() != names.size()) {
      RCLCPP_ERROR(node_->get_logger(), "[servo bridge, torque cb]: name/effort size mismatch (%zu vs %zu)",
                   names.size(), eff.size());
      return;
    }
    for (size_t i = 0; i < names.size(); ++i) {
      auto& handlers = servos_handler_[servo_group_name];
      auto it_hdl = std::find_if(handlers.begin(), handlers.end(),
                                 [&](const SingleServoHandlePtr& h) { return names[i] == h->getName(); });
      if (it_hdl == handlers.end()) {
        RCLCPP_ERROR(node_->get_logger(), "[servo bridge, torque cb]: no handler for %s", names[i].c_str());
        return;
      }
      (*it_hdl)->setTargetTorqueVal(eff[i]);
      target_torque_msg.index.push_back((*it_hdl)->getId());
      target_torque_msg.angles.push_back((*it_hdl)->getTargetTorqueVal(ValueType::BIT));
    }
  } else {
    // unnamed
    if (eff.size() != servos_handler_[servo_group_name].size()) {
      RCLCPP_ERROR(node_->get_logger(), "[servo bridge, torque cb]: expected %zu efforts, got %zu",
                   servos_handler_[servo_group_name].size(), eff.size());
      return;
    }
    for (size_t i = 0; i < eff.size(); ++i) {
      auto& h = servos_handler_[servo_group_name][i];
      h->setTargetTorqueVal(eff[i]);
      target_torque_msg.index.push_back(h->getId());
      target_torque_msg.angles.push_back(h->getTargetTorqueVal(ValueType::BIT));
    }
  }

  auto& torque_pub = servo_target_torque_pubs_.count(servo_group_name) ? servo_target_torque_pubs_[servo_group_name]
                                                                       : servo_target_torque_pubs_["common"];
  torque_pub->publish(target_torque_msg);
}

// --- servoEnableCallback ----------------------------------------------------
void ServoBridge::servoEnableCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res,
                                      const std::string& servo_group_name) {
  // build torque‐enable message
  auto torque_msg = spinal_msgs::msg::ServoTorqueCmd();
  for (auto& h : servos_handler_[servo_group_name]) {
    torque_msg.index.push_back(h->getId());
    torque_msg.torque_enable.push_back(req->data);
  }

  auto& pub =
      servo_enable_pubs_.count(servo_group_name) ? servo_enable_pubs_[servo_group_name] : servo_enable_pubs_["common"];
  pub->publish(torque_msg);

  res->success = true;
  res->message = "ok";
}

// --- uavInfoCallback --------------------------------------------------------
void ServoBridge::uavInfoCallback(const spinal_msgs::msg::UavInfo::ConstSharedPtr /*uav_msg*/) {
  // publish joint profiles
  auto out = spinal_msgs::msg::JointProfiles();
  for (const auto& [group, handlers] : servos_handler_) {
    for (const auto& h : handlers) {
      auto jp = spinal_msgs::msg::JointProfile();
      jp.servo_id = h->getId();
      jp.angle_sgn = h->getAngleSgn();
      jp.angle_scale = h->getAngleScale();
      jp.zero_point_offset = h->getZeroPointOffset();
      if (group == "joints")
        jp.type = spinal_msgs::msg::JointProfile::JOINT;
      else if (group == "gimbals")
        jp.type = spinal_msgs::msg::JointProfile::GIMBAL;
      else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid servo group '%s', must be 'joints' or 'gimbals'", group.c_str());
        continue;
      }
      out.joints.push_back(jp);
    }
  }
  joint_profile_pub_->publish(out);
}
