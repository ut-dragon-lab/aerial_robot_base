// -*- mode: c++ -*-
/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
 * All rights reserved.
 */

#include "aerial_robot_simulation/spinal_interface.h"

#include <flight_control/simulation/flight_control_ros_module.h>
#include <thruster/simulation/thruster_ros_module.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace aerial_robot_simulation {

class MujocoAttitudeControllerNode {
public:
  explicit MujocoAttitudeControllerNode(
      const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node)
      : node_(node) {
    node_->declare_parameter<double>("update_rate", 1000.0);
    node_->declare_parameter<std::vector<std::string>>(
        "rotor_joints", {"rotor1", "rotor2", "rotor3", "rotor4"});
    node_->declare_parameter<bool>("use_ground_truth", true);

    rotor_joints_ = node_->get_parameter("rotor_joints").as_string_array();
    if (rotor_joints_.empty()) {
      rotor_joints_ = {"rotor1", "rotor2", "rotor3", "rotor4"};
    }

    spinal_iface_.init(node_);
    spinal_iface_.useGroundTruth(
        node_->get_parameter("use_ground_truth").as_bool());

    thruster_ros_mod_.init(node_);
    flight_control_ros_mod_.init(
        node_, spinal_iface_.getEstimatorPtr()->getEstimator(),
        thruster_ros_mod_.getThrusterManager());

    spinal_iface_.getEstimatorPtr()->getImuPub()->on_activate();
    flight_control_ros_mod_.activate();
    thruster_ros_mod_.activate();

    rotor_force_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "mujoco/rotor_forces", rclcpp::QoS(1));

    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "mujoco/imu", sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          latest_imu_ = *msg;
          have_imu_ = true;
        });

    mag_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>(
        "mujoco/mag", sensor_qos,
        [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
          latest_mag_ = *msg;
          have_mag_ = true;
        });

    ground_truth_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth", rclcpp::QoS(1),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_ground_truth_ = *msg;
          have_ground_truth_ = true;
        });

    const double update_rate =
        std::max(1.0, node_->get_parameter("update_rate").as_double());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate));
    update_timer_ = node_->create_wall_timer(period, [this]() { update(); });

    RCLCPP_INFO(
        node_->get_logger(),
        "[mujoco] attitude controller node started with %.1f Hz update rate",
        update_rate);
  }

private:
  void update() {
    if (!have_imu_) {
      return;
    }

    spinal_iface_.onGround(false);
    spinal_iface_.setImuValue(
        latest_imu_.linear_acceleration.x, latest_imu_.linear_acceleration.y,
        latest_imu_.linear_acceleration.z, latest_imu_.angular_velocity.x,
        latest_imu_.angular_velocity.y, latest_imu_.angular_velocity.z);

    if (have_mag_) {
      spinal_iface_.setMagValue(latest_mag_.magnetic_field.x,
                                latest_mag_.magnetic_field.y,
                                latest_mag_.magnetic_field.z);
    }

    if (have_ground_truth_) {
      spinal_iface_.setGroundTruthStates(
          latest_ground_truth_.pose.pose.orientation.x,
          latest_ground_truth_.pose.pose.orientation.y,
          latest_ground_truth_.pose.pose.orientation.z,
          latest_ground_truth_.pose.pose.orientation.w,
          latest_ground_truth_.twist.twist.angular.x,
          latest_ground_truth_.twist.twist.angular.y,
          latest_ground_truth_.twist.twist.angular.z);
    }

    spinal_iface_.stateEstimate();

    flight_control_ros_mod_.update();
    thruster_ros_mod_.sendCommand();
    publishRotorForces_();
    flight_control_ros_mod_.publish();
    thruster_ros_mod_.publish();
  }

  void publishRotorForces_() {
    ThrusterManager *thruster = thruster_ros_mod_.getThrusterManager();
    if (thruster == nullptr || !rotor_force_pub_) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();

    const size_t n =
        std::min(rotor_joints_.size(), static_cast<size_t>(MAX_THRUSTER_NUM));
    msg.name.resize(n);
    msg.effort.resize(n);
    for (size_t i = 0; i < n; ++i) {
      msg.name[i] = rotor_joints_[i];
      msg.effort[i] = static_cast<double>(
          thruster->getTargetThrust(static_cast<uint8_t>(i)));
    }
    rotor_force_pub_->publish(msg);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  hardware_interface::SpinalInterface spinal_iface_;
  ThrusterRosModule thruster_ros_mod_;
  FlightControlRosModule flight_control_ros_mod_;

  std::vector<std::string> rotor_joints_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rotor_force_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  sensor_msgs::msg::Imu latest_imu_;
  sensor_msgs::msg::MagneticField latest_mag_;
  nav_msgs::msg::Odometry latest_ground_truth_;
  bool have_imu_{false};
  bool have_mag_{false};
  bool have_ground_truth_{false};
};

} // namespace aerial_robot_simulation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "mujoco_attitude_controller");
  auto controller =
      std::make_shared<aerial_robot_simulation::MujocoAttitudeControllerNode>(
          node);
  (void)controller;
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
