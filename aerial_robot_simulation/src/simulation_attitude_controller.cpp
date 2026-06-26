// -*- mode: c++ -*-
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
#include "aerial_robot_simulation/simulation_attitude_controller.h"

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flight_controllers
{
controller_interface::CallbackReturn SimulationAttitudeController::on_init()
{
  auto_declare<std::string>("imu", "spinal_imu");
  auto_declare<std::string>("mag", "spinal_mag");
  auto_declare<std::vector<std::string>>("rotor_joints", { "rotor1", "rotor2", "rotor3", "rotor4" });
  spinal_iface_.init(get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SimulationAttitudeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::vector<std::string> rotor_joints = get_node()->get_parameter("rotor_joints").as_string_array();
  for (const auto &joint : rotor_joints)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration SimulationAttitudeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  static const std::array<std::string, 10> imu_ifaces = { "orientation.x",         "orientation.y",
                                                          "orientation.z",         "orientation.w",
                                                          "angular_velocity.x",    "angular_velocity.y",
                                                          "angular_velocity.z",    "linear_acceleration.x",
                                                          "linear_acceleration.y", "linear_acceleration.z" };

  const std::string imu = get_node()->get_parameter("imu").as_string();
  for (const auto &iface : imu_ifaces)
  {
    config.names.push_back(imu + "/" + iface);
  }

  static const std::array<std::string, 3> mag_ifaces = { "field_tesla.x", "field_tesla.y", "field_tesla.z" };

  const std::string mag = get_node()->get_parameter("mag").as_string();
  for (const auto &iface : mag_ifaces)
  {
    config.names.push_back(mag + "/" + iface);
  }

  return config;
}

controller_interface::CallbackReturn SimulationAttitudeController::on_configure(const rclcpp_lifecycle::State &)
{
  spinal_iface_.useGroundTruth(true);

  thruster_ros_mod_.init(get_node());
  flight_control_ros_mod_.init(get_node(), spinal_iface_.getEstimatorPtr()->getEstimator(),
                               thruster_ros_mod_.getThrusterManager());

  RCLCPP_INFO(get_node()->get_logger(), "[sim] SimulationAttitudeController: on_configure");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimulationAttitudeController::on_activate(const rclcpp_lifecycle::State &)
{
  spinal_iface_.getEstimatorPtr()->getImuPub()->on_activate();
  flight_control_ros_mod_.activate();
  thruster_ros_mod_.activate();
  RCLCPP_INFO(get_node()->get_logger(), "[sim] SimulationAttitudeController: on_activate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimulationAttitudeController::on_deactivate(const rclcpp_lifecycle::State &)
{
  flight_control_ros_mod_.deactivate();
  thruster_ros_mod_.deactivate();
  RCLCPP_INFO(get_node()->get_logger(), "[sim] SimulationAttitudeController: on_deactivate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimulationAttitudeController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  spinal_iface_.onGround(false);

  const double q_x = state_interfaces_[0].get_value();
  const double q_y = state_interfaces_[1].get_value();
  const double q_z = state_interfaces_[2].get_value();
  const double q_w = state_interfaces_[3].get_value();

  const double ang_x = state_interfaces_[4].get_value();
  const double ang_y = state_interfaces_[5].get_value();
  const double ang_z = state_interfaces_[6].get_value();

  const double acc_x = state_interfaces_[7].get_value();
  const double acc_y = state_interfaces_[8].get_value();
  const double acc_z = state_interfaces_[9].get_value();

  const double mag_x = state_interfaces_[10].get_value();
  const double mag_y = state_interfaces_[11].get_value();
  const double mag_z = state_interfaces_[12].get_value();

  spinal_iface_.setImuValue(acc_x, acc_y, acc_z, ang_x, ang_y, ang_z);
  spinal_iface_.setMagValue(mag_x, mag_y, mag_z);
  spinal_iface_.setGroundTruthStates(q_x, q_y, q_z, q_w, ang_x, ang_y, ang_z);
  spinal_iface_.stateEstimate();

  flight_control_ros_mod_.update();
  thruster_ros_mod_.sendCommand();
  writeRotorCommands_();
  flight_control_ros_mod_.publish();
  thruster_ros_mod_.publish();

  return controller_interface::return_type::OK;
}

void SimulationAttitudeController::writeRotorCommands_()
{
  ThrusterManager *thruster = thruster_ros_mod_.getThrusterManager();
  const size_t n = std::min(command_interfaces_.size(), static_cast<size_t>(MAX_THRUSTER_NUM));
  for (size_t i = 0; i < n; ++i)
  {
    command_interfaces_[i].set_value(static_cast<double>(thruster->getTargetThrust(static_cast<uint8_t>(i))));
  }
}

}

PLUGINLIB_EXPORT_CLASS(flight_controllers::SimulationAttitudeController, controller_interface::ControllerInterface)
