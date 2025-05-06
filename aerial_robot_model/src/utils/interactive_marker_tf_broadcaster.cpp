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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

using namespace std::chrono_literals;

class TfPublisher : public rclcpp::Node {
 public:
  TfPublisher() : Node("interactive_marker_tf_broadcaster") {
    // 1) parameters
    this->declare_parameter<std::string>("target_frame", "root");
    this->declare_parameter<std::string>("reference_frame", "fixed_frame");
    this->declare_parameter<double>("tf_loop_rate", 60.0);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("reference_frame", reference_frame_);
    this->get_parameter("tf_loop_rate", tf_loop_rate_);

    // 2) interactive marker server
    //    Note: Humble+ requires you pass two QoSes
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(this->get_name(), this->shared_from_this(),
                                                                             rclcpp::QoS{10}, rclcpp::QoS{10});

    // 3) transform broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this->shared_from_this());

    // 4) build the marker and register feedback callback
    intMarkerInit();

    // 5) timer for publishing tf at fixed rate
    tf_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / tf_loop_rate_),
                                        std::bind(&TfPublisher::tfPublish, this));

    RCLCPP_INFO(this->get_logger(), "Interactive‐marker TF broadcaster started");
  }

 private:
  // parameters
  std::string target_frame_;
  std::string reference_frame_;
  double tf_loop_rate_;

  // interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // timer
  rclcpp::TimerBase::SharedPtr tf_timer_;

  // protected by mutex
  std::mutex mutex_;
  tf2::Transform target_pose_{tf2::Transform::getIdentity()};

  // feedback from the interactive marker
  void tfProcessFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      tf2::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                        feedback->pose.orientation.w);
      tf2::Vector3 o(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      {
        std::lock_guard<std::mutex> lk(mutex_);
        target_pose_.setOrigin(o);
        target_pose_.setRotation(q);
      }
    }
    server_->applyChanges();
  }

  // publish the TF at fixed rate
  void tfPublish() {
    geometry_msgs::msg::TransformStamped msg;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      auto origin = target_pose_.getOrigin();
      msg.transform.translation.x = origin.getX();
      msg.transform.translation.y = origin.getY();
      msg.transform.translation.z = origin.getZ();
      // getRotation() returns by value; capture it in a local
      tf2::Quaternion rot = target_pose_.getRotation();
      msg.transform.rotation.x = rot.x();
      msg.transform.rotation.y = rot.y();
      msg.transform.rotation.z = rot.z();
      msg.transform.rotation.w = rot.w();
    }
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = reference_frame_;
    msg.child_frame_id = target_frame_;

    tf_broadcaster_->sendTransform(msg);
  }

  // set up the 6‐DOF interactive marker
  void intMarkerInit() {
    visualization_msgs::msg::InteractiveMarker int_marker;
    visualization_msgs::msg::InteractiveMarkerControl control;
    int_marker.header.frame_id = reference_frame_;
    int_marker.name = target_frame_ + "_control";

    // X axis
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Y axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Z axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server_->insert(int_marker, std::bind(&TfPublisher::tfProcessFeedback, this, std::placeholders::_1));
    server_->applyChanges();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
