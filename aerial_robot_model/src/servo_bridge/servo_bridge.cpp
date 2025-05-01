// src/servo_bridge.cpp

#include "aerial_robot_model/servo_bridge.h"

#include <urdf/model.h>

#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

ServoBridge::ServoBridge(rclcpp::Node::SharedPtr node) : node_(node) {
  // 1) Simulation / mujoco flags
  node_->declare_parameter<bool>("use_sim_time", false);
  node_->get_parameter("use_sim_time", simulation_mode_);
  node_->declare_parameter<bool>("use_mujoco", false);
  node_->get_parameter("use_mujoco", use_mujoco_);
  if (use_mujoco_) {
    RCLCPP_WARN(node_->get_logger(), "use mujoco simulator; disabling ROS time");
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
  node_->declare_parameter<std::string>("robot_description", "");
  node_->get_parameter("robot_description", urdf_xml);
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse robot_description URDF");
  }

  // 3) Groups (always include "common")
  node_->declare_parameter<std::vector<std::string>>("servo_controller.groups", std::vector<std::string>{"common"});
  std::vector<std::string> groups;
  node_->get_parameter("servo_controller.groups", groups);
  if (std::find(groups.begin(), groups.end(), "common") == groups.end()) {
    groups.insert(groups.begin(), "common");
  }

  // 4) Per‐group setup
  for (const auto& group : groups) {
    const std::string base = "servo_controller." + group + ".";

    // parameters (with ROS1互換のデフォルト)
    bool no_real = node_->declare_parameter<bool>(base + "no_real_state", false);
    no_real_state_flags_[group] = no_real;

    std::string state_sub_topic =
        node_->declare_parameter<std::string>(base + "state_sub_topic", (group == "common" ? "servo/states" : ""));
    std::string pos_pub_topic =
        node_->declare_parameter<std::string>(base + "pos_pub_topic", (group == "common" ? "servo/target_states" : ""));
    std::string torque_pub_topic = node_->declare_parameter<std::string>(
        base + "torque_pub_topic", (group == "common" ? "servo/target_current" : ""));
    std::string enable_pub_topic = node_->declare_parameter<std::string>(
        base + "servo_enable_pub_topic", (group == "common" ? "servo/torque_enable" : ""));

    int angle_sgn = node_->declare_parameter<int>(base + "angle_sgn", 1);
    int zero_offset = node_->declare_parameter<int>(base + "zero_point_offset", 0);
    double angle_scale = node_->declare_parameter<double>(base + "angle_scale", 1.0);
    double torque_scale = node_->declare_parameter<double>(base + "torque_scale", 1.0);
    bool filter_flag = node_->declare_parameter<bool>(base + "filter_flag", false);
    double sample_freq = node_->declare_parameter<double>(base + "sample_freq", 0.0);
    double cutoff_freq = node_->declare_parameter<double>(base + "cutoff_freq", 0.0);

    // simulation‐only defaults (ROS1 互換)
    std::string sim_type = node_->declare_parameter<std::string>(base + "simulation.type", "");
    std::string sim_pid = node_->declare_parameter<std::string>(base + "simulation.pid", "");
    double sim_init = node_->declare_parameter<double>(base + "simulation.init_value", 0.0);

    //--- common‐group interfaces ---
    if (!state_sub_topic.empty()) {
      servo_states_subs_[group] = node_->create_subscription<spinal::msg::ServoStates>(
          state_sub_topic, rclcpp::SystemDefaultsQoS(),
          [this, group](spinal::msg::ServoStates::ConstSharedPtr msg) { this->servoStatesCallback(msg, group); });
    }
    if (!pos_pub_topic.empty()) {
      servo_target_pos_pubs_[group] =
          node_->create_publisher<spinal::msg::ServoControlCmd>(pos_pub_topic, rclcpp::SystemDefaultsQoS());
    }
    if (!torque_pub_topic.empty()) {
      servo_target_torque_pubs_[group] =
          node_->create_publisher<spinal::msg::ServoControlCmd>(torque_pub_topic, rclcpp::SystemDefaultsQoS());
    }
    if (!enable_pub_topic.empty()) {
      servo_enable_pubs_[group] =
          node_->create_publisher<spinal::msg::ServoTorqueCmd>(enable_pub_topic, rclcpp::SystemDefaultsQoS());
    }
    // service for torque‐enable
    servo_enable_srvs_[group] = node_->create_service<std_srvs::srv::SetBool>(
        group + "/torque_enable", [this, group](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          this->servoEnableCallback(req, res, group);
        });

    //--- per‐servo controllers in this group ---
    node_->declare_parameter<std::vector<std::string>>(base + "controllers", std::vector<std::string>{});
    std::vector<std::string> controllers;
    node_->get_parameter(base + "controllers", controllers);

    ServoGroupHandler handler;
    for (const auto& ctrl : controllers) {
      const std::string cbase = base + "controllers." + ctrl + ".";

      // declare & read back servo‐specific params
      node_->declare_parameter<std::string>(cbase + "name", "");
      node_->declare_parameter<int>(cbase + "id", 0);
      node_->declare_parameter<int>(cbase + "angle_sgn", 1);
      node_->declare_parameter<int>(cbase + "zero_point_offset", 0);
      node_->declare_parameter<double>(cbase + "angle_scale", 1.0);
      node_->declare_parameter<double>(cbase + "torque_scale", 1.0);
      node_->declare_parameter<bool>(cbase + "filter_flag", false);
      node_->declare_parameter<double>(cbase + "sample_freq", 0.0);
      node_->declare_parameter<double>(cbase + "cutoff_freq", 0.0);

      std::string name;
      int id, a_sgn, z_off;
      double a_scale, t_scale, s_freq, c_freq;
      bool f_flag;
      node_->get_parameter(cbase + "name", name);
      node_->get_parameter(cbase + "id", id);
      node_->get_parameter(cbase + "angle_sgn", a_sgn);
      node_->get_parameter(cbase + "zero_point_offset", z_off);
      node_->get_parameter(cbase + "angle_scale", a_scale);
      node_->get_parameter(cbase + "torque_scale", t_scale);
      node_->get_parameter(cbase + "filter_flag", f_flag);
      node_->get_parameter(cbase + "sample_freq", s_freq);
      node_->get_parameter(cbase + "cutoff_freq", c_freq);

      // URDF から joint limit
      auto j = urdf_model.getJoint(name);
      double upper = j->limits->upper;
      double lower = j->limits->lower;

      handler.push_back(std::make_shared<SingleServoHandle>(name, id, a_sgn, z_off, a_scale, upper, lower, t_scale,
                                                            !no_real_state_flags_.at(group), f_flag, s_freq, c_freq));

      if (simulation_mode_) {
        // ROS 1版と同様に gazebo controller のロード／起動＆
        // servo_target_pos_sim_pubs_ の初期設定を行ってください
      }
    }
    servos_handler_[group] = std::move(handler);
  }

  // 5) joint_states publisher （simulation時は不要）
  if (!simulation_mode_) {
    servo_states_pub_ =
        node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS());
  }
  // 6) mujoco output
  mujoco_control_input_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("mujoco/ctrl_input", rclcpp::SystemDefaultsQoS());
  // 7) joint_profile output
  joint_profile_pub_ =
      node_->create_publisher<spinal::msg::JointProfiles>("joint_profiles", rclcpp::SystemDefaultsQoS());
  // 8) UAV info subscriber
  uav_info_sub_ = node_->create_subscription<spinal::msg::UavInfo>(
      "uav_info", rclcpp::SystemDefaultsQoS(), std::bind(&ServoBridge::uavInfoCallback, this, std::placeholders::_1));
}

// --- servoStatesCallback ---------------------------------------------------
void ServoBridge::servoStatesCallback(const spinal::msg::ServoStates::ConstSharedPtr state_msg,
                                      const std::string& servo_group_name) {
  // 1) "common" 以外はそのグループだけ更新して終了
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

  // 2) "common" グループは、各サブグループを検索して更新
  for (const auto& it : state_msg->servos) {
    for (auto& [group, handlers] : servos_handler_) {
      // real state を使わない設定ならスキップ
      if (no_real_state_flags_.at(group)) {
        RCLCPP_DEBUG(node_->get_logger(), "%s: no_real_state, skip", group.c_str());
        continue;
      }
      // 独立グループで専用に登録されていれば common では処理しない
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

  // 3) 最後にまとめて joint_states トピックに publish
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
  auto target_angle_msg = spinal::msg::ServoControlCmd();
  auto target_torque_msg = spinal::msg::ServoControlCmd();
  auto mujoco_msg = sensor_msgs::msg::JointState();

  const auto& names = servo_ctrl_msg->name;
  const auto& pos = servo_ctrl_msg->position;
  const auto& eff = servo_ctrl_msg->effort;

  if (!names.empty()) {
    // named
    if (pos.size() != names.size()) {
      RCLCPP_ERROR(node_->get_logger(), "[servo bridge, ctrl cb]: name/position size mismatch (%zu vs %zu)",
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
        RCLCPP_ERROR(node_->get_logger(), "[servo bridge, ctrl cb]: no handler for %s", names[i].c_str());
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

      if (simulation_mode_) {
        auto pub = servo_target_pos_sim_pubs_[servo_group_name][std::distance(handlers.begin(), it_hdl)];
        auto m = std_msgs::msg::Float64();
        m.data = pos[i];
        pub->publish(m);
      }
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
        auto pub = servo_target_pos_sim_pubs_[servo_group_name][i];
        auto m = std_msgs::msg::Float64();
        m.data = pos[i];
        pub->publish(m);
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
  auto target_torque_msg = spinal::msg::ServoControlCmd();
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
  auto torque_msg = spinal::msg::ServoTorqueCmd();
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
void ServoBridge::uavInfoCallback(const spinal::msg::UavInfo::ConstSharedPtr /*uav_msg*/) {
  // publish joint profiles
  auto out = spinal::msg::JointProfiles();
  for (const auto& [group, handlers] : servos_handler_) {
    for (const auto& h : handlers) {
      auto jp = spinal::msg::JointProfile();
      jp.servo_id = h->getId();
      jp.angle_sgn = h->getAngleSgn();
      jp.angle_scale = h->getAngleScale();
      jp.zero_point_offset = h->getZeroPointOffset();
      if (group == "joints")
        jp.type = spinal::msg::JointProfile::JOINT;
      else if (group == "gimbals")
        jp.type = spinal::msg::JointProfile::GIMBAL;
      else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid servo group '%s', must be 'joints' or 'gimbals'", group.c_str());
        continue;
      }
      out.joints.push_back(jp);
    }
  }
  joint_profile_pub_->publish(out);
}
