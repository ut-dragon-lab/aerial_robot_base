// Software License Agreement (BSD License)

// Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <tf2_kdl/tf2_kdl.hpp>

#include "aerial_robot_model/model/aerial_robot_model_ros.h"

namespace aerial_robot_model {

RobotModelRos::RobotModelRos(rclcpp::Node::SharedPtr node)
    : node_(node),
      tf_broadcaster_(*node_),
      static_broadcaster_(*node_),
      // pluginlib: package name, base class
      robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel") {
  // 1) Declare & read the tf_prefix parameter
  node_->declare_parameter<std::string>("tf_prefix", "");
  node_->get_parameter("tf_prefix", tf_prefix_);

  // 2) Load the robot‐model plugin
  std::string plugin_name;
  if (node_->get_parameter("robot_model_plugin_name", plugin_name)) {
    try {
      robot_model_ = robot_model_loader_.createSharedInstance(plugin_name);
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load plugin '%s': %s", plugin_name.c_str(), ex.what());
      // fallback to default
      robot_model_ = std::make_shared<RobotModel>();
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Parameter 'robot_model_plugin_name' not set; using default RobotModel");
    robot_model_ = std::make_shared<RobotModel>();
  }

  // initialize robot model plugin
  robot_model_->initialize(node_);

  // 3) If model is fixed, publish a single static transform
  if (robot_model_->isModelFixed()) {
    auto tf = robot_model_->getCog<geometry_msgs::msg::TransformStamped>();
    tf.header.stamp = node_->now();

    // prepend prefix if given
    auto root_frame =
        tf_prefix_.empty() ? robot_model_->getRootFrameName() : tf_prefix_ + "/" + robot_model_->getRootFrameName();
    tf.header.frame_id = root_frame;

    auto cog_frame = tf_prefix_.empty() ? "cog" : tf_prefix_ + "/cog";
    tf.child_frame_id = cog_frame;

    static_broadcaster_.sendTransform(tf);

  } else {
    // 4) Otherwise subscribe to joint_states and broadcast CoG dynamically
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SystemDefaultsQoS(),
        std::bind(&RobotModelRos::jointStateCallback, this, std::placeholders::_1));
  }

  // 5) Advertise the add_extra_module service
  add_extra_module_srv_ = node_->create_service<aerial_robot_model::srv::AddExtraModule>(
      "add_extra_module", std::bind(&RobotModelRos::addExtraModuleCallback, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));
}

void RobotModelRos::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // cache the latest JointState
  joint_state_ = *msg;

  // update internal model
  robot_model_->updateRobotModel(*msg);

  // get new CoG pose and broadcast
  auto tf = robot_model_->getCog<geometry_msgs::msg::TransformStamped>();
  tf.header = msg->header;

  auto root_frame =
      tf_prefix_.empty() ? robot_model_->getRootFrameName() : tf_prefix_ + "/" + robot_model_->getRootFrameName();
  tf.header.frame_id = root_frame;

  auto cog_frame = tf_prefix_.empty() ? "cog" : tf_prefix_ + "/cog";
  tf.child_frame_id = cog_frame;

  tf_broadcaster_.sendTransform(tf);
}

void RobotModelRos::addExtraModuleCallback(const std::shared_ptr<rmw_request_id_t> /*req_id*/,
                                           const std::shared_ptr<aerial_robot_model::srv::AddExtraModule::Request> req,
                                           std::shared_ptr<aerial_robot_model::srv::AddExtraModule::Response> res) {
  switch (req->action) {
    case aerial_robot_model::srv::AddExtraModule::Request::ADD: {
      // build KDL::Frame from the incoming transform
      geometry_msgs::msg::TransformStamped ts;
      ts.transform = req->transform;
      KDL::Frame f = tf2::transformToKDL(ts);
      // build inertia
      KDL::RigidBodyInertia rbi(req->inertia.m, KDL::Vector(req->inertia.com.x, req->inertia.com.y, req->inertia.com.z),
                                KDL::RotationalInertia(req->inertia.ixx, req->inertia.iyy, req->inertia.izz,
                                                       req->inertia.ixy, req->inertia.ixz, req->inertia.iyz));

      res->status = robot_model_->addExtraModule(req->module_name, req->parent_link_name, f, rbi);
      break;
    }
    case aerial_robot_model::srv::AddExtraModule::Request::REMOVE: {
      res->status = robot_model_->removeExtraModule(req->module_name);
      break;
    }
    default: {
      RCLCPP_WARN(node_->get_logger(), "[add_extra_module] invalid action %d", req->action);
      res->status = false;
      break;
    }
  }
  // no return value; 'status' field carries the result
}

}  // namespace aerial_robot_model
