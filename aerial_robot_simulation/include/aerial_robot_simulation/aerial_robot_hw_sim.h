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

#ifndef AERIAL_ROBOT_HW_SIM_H_
#define AERIAL_ROBOT_HW_SIM_H_

#include <aerial_robot_simulation/spinal_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hardware_interface/handle.hpp>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace gz_ros2_control {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AerialRobotHwSimPrivate;

class AerialRobotHwSim : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(const rclcpp::Time& sim_time, const rclcpp::Duration& period) override;

  // Documentation Inherited
  hardware_interface::return_type write(const rclcpp::Time& sim_time, const rclcpp::Duration& period) override;

  // Documentation Inherited
  bool initSim(rclcpp::Node::SharedPtr& model_nh, std::map<std::string, sim::Entity>& joints,
               const hardware_interface::HardwareInfo& hardware_info, sim::EntityComponentManager& ecm,
               int& update_rate) override;

 private:
  // Number of rotors
  size_t rotor_n_dof_;

  // Rotor state & command buffers
  std::vector<double> rotor_efforts_;          // current effort state
  std::vector<double> rotor_efforts_cmd_;      // commanded effort
  std::vector<double> rotor_velocity_states_;  // current rotor speed

  // Control mode: FORCE, VEL, POS
  uint8_t control_mode_;
  geometry_msgs::msg::TwistStamped cmd_vel_;
  geometry_msgs::msg::PoseStamped cmd_pos_;
  ;

  hardware_interface::HardwareInfo hw_info_;

  void registerSensors(const hardware_interface::HardwareInfo& hardware_info);

  void querySimPrams();

  std::unique_ptr<AerialRobotHwSimPrivate> dataPtr;
};

}  // namespace gz_ros2_control

namespace ign_ros2_control {
using IgnitionSystem = gz_ros2_control::AerialRobotHwSim;
}  // namespace ign_ros2_control

#endif  // AERIAL_ROBOT_HW_SIM_H_
