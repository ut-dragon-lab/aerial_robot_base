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

#include "aerial_robot_simulation/simulation_attitude_controller.h"

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flight_controllers {
controller_interface::CallbackReturn SimulationAttitudeController::on_init() {
  auto_declare<std::string>("imu", "spinal_imu");
  auto_declare<std::string>("mag", "spinal_mag");
  spinal_iface_.init(get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SimulationAttitudeController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::InterfaceConfiguration SimulationAttitudeController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // IMU sensor value
  static const std::array<std::string, 10> imu_ifaces = {
      "orientation.x",         "orientation.y",        "orientation.z",      "orientation.w",
      "angular_velocity.x",    "angular_velocity.y",   "angular_velocity.z", "linear_acceleration.x",
      "linear_acceleration.y", "linear_acceleration.z"};

  const std::string imu = get_node()->get_parameter("imu").as_string();
  for (const auto &iface : imu_ifaces) {
    config.names.push_back(imu + "/" + iface);
  }

  // Mag sensor value
  static const std::array<std::string, 3> mag_ifaces = {"field_tesla.x", "field_tesla.y", "field_tesla.z"};

  const std::string mag = get_node()->get_parameter("mag").as_string();
  for (const auto &iface : mag_ifaces) {
    config.names.push_back(mag + "/" + iface);
  }

  return config;
}

controller_interface::CallbackReturn SimulationAttitudeController::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(), "SimulationAttitudeController: on_configure");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimulationAttitudeController::on_activate(const rclcpp_lifecycle::State &) {
  // activate publishers
  spinal_iface_.getEstimatorPtr()->getAttEstimator()->getImuPub()->on_activate();
  RCLCPP_INFO(get_node()->get_logger(), "SimulationAttitudeController: on_activate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimulationAttitudeController::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(), "SimulationAttitudeController: on_deactivate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimulationAttitudeController::update(const rclcpp::Time & /*time*/,
                                                                       const rclcpp::Duration & /*period*/
) {
  spinal_iface_.onGround(false);

  // angular velocity
  double ang_x = state_interfaces_[4].get_value();
  double ang_y = state_interfaces_[5].get_value();
  double ang_z = state_interfaces_[6].get_value();

  // linear acceleration
  double acc_x = state_interfaces_[7].get_value();
  double acc_y = state_interfaces_[8].get_value();
  double acc_z = state_interfaces_[9].get_value();

  // mag value
  double mag_x = state_interfaces_[10].get_value();
  double mag_y = state_interfaces_[11].get_value();
  double mag_z = state_interfaces_[12].get_value();

  spinal_iface_.setImuValue(acc_x, acc_y, acc_z, ang_x, ang_y, ang_z);
  spinal_iface_.setMagValue(mag_x, mag_y, mag_z);
  spinal_iface_.stateEstimate();
  return controller_interface::return_type::OK;
}

}  // namespace flight_controllers

PLUGINLIB_EXPORT_CLASS(flight_controllers::SimulationAttitudeController, controller_interface::ControllerInterface)
