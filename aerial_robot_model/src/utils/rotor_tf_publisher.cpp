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

#include <tf2_ros/static_transform_broadcaster.h>
#include <urdf/model.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_kdl/tf2_kdl.hpp>
#include <vector>

using namespace std::chrono_literals;
using std::string;

class RotorTfPublisher {
 public:
  RotorTfPublisher(rclcpp::Node::SharedPtr node) {
    node_ = node;
    // parameters
    // node_->declare_parameter<string>("rotor_joint_name", "rotor");
    // node_->declare_parameter<string>("tf_prefix", "");
    node_->get_parameter("rotor_joint_name", rotor_joint_name_);
    node_->get_parameter("tf_prefix", tf_prefix_);

    std::string urdf_xml;
    node_->declare_parameter<std::string>("robot_description", "");
    node_->get_parameter("robot_description", urdf_xml);

    if (!model_.initString(urdf_xml)) {
      RCLCPP_ERROR(rclcpp::get_logger("rotor_tf_publisher"), "Failed to parse 'robot_description' URDF");
      return;
    }

    if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
      RCLCPP_ERROR(rclcpp::get_logger("rotor_tf_publisher"), "Failed to extract KDL tree from URDF");
      return;
    }

    // collect all the rotor‐attached segments
    addChildren(tree_.getRootSegment()->second);

    // schedule one‐shot timer to broadcast static TFs after 100ms
    timer_ = node_->create_wall_timer(100ms, [this]() {
      broadcastStaticTransforms();
      timer_->cancel();
    });

    RCLCPP_INFO(node_->get_logger(), "rotor_publisher initialized");

    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_->shared_from_this());
  }

 private:
  /// pair of segment + its root/tip frame names
  struct SegmentPair {
    KDL::Segment segment;
    string root, tip;
  };

  /// recurse the KDL::Tree, collect segments whose joint name contains rotor_joint_name_
  void addChildren(const KDL::TreeElement& elem) {
    const auto& seg = GetTreeElementSegment(elem);
    const string root = seg.getName();

    for (const auto& child : GetTreeElementChildren(elem)) {
      const auto& child_seg = GetTreeElementSegment(child->second);
      if (child_seg.getJoint().getName().find(rotor_joint_name_) != string::npos) {
        segments_rotor_.emplace(child_seg.getJoint().getName(), SegmentPair{child_seg, root, child_seg.getName()});
      }
      addChildren(child->second);
    }
  }

  /// build and send all static transforms
  void broadcastStaticTransforms() {
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.reserve(segments_rotor_.size());
    auto now = node_->get_clock()->now();
    for (auto& kv : segments_rotor_) {
      const auto& sp = kv.second;
      auto msg = tf2::kdlToTransform(sp.segment.pose(0));
      msg.header.stamp = now;

      if (tf_prefix_.empty()) {
        msg.header.frame_id = sp.root;
        msg.child_frame_id = sp.tip;
      } else {
        msg.header.frame_id = tf_prefix_ + "/" + sp.root;
        msg.child_frame_id = tf_prefix_ + "/" + sp.tip;
      }
      tfs.push_back(msg);
    }
    static_broadcaster_->sendTransform(tfs);
    RCLCPP_INFO(node_->get_logger(), "Published %zu static rotor TF(s)", tfs.size());
  }

  // members
  rclcpp::Node::SharedPtr node_;
  KDL::Tree tree_;
  urdf::Model model_;
  string rotor_joint_name_;
  string tf_prefix_;
  std::map<string, SegmentPair> segments_rotor_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto ros_node = std::make_shared<rclcpp::Node>("rotor_tf_publisher", options);

  auto rotor_tf_publisher = std::make_shared<RotorTfPublisher>(ros_node);
  rclcpp::spin(ros_node);
  rclcpp::shutdown();
  return 0;
}
