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

#ifndef SIMULATION_ATTITUDE_CONTROLLER_H
#define SIMULATION_ATTITUDE_CONTROLLER_H

#include <aerial_robot_simulation/spinal_interface.h>

#include <boost/scoped_ptr.hpp>
// #include <flight_control/flight_control.h>
#include <urdf/model.h>

#include <memory>
#include <spinal_msgs/msg/desire_coord.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <lifecycle_msgs/msg/state.hpp>

namespace flight_controllers {

class SimulationAttitudeController : public controller_interface::ControllerInterface {
 public:
  SimulationAttitudeController() = default;
  virtual ~SimulationAttitudeController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  hardware_interface::SpinalInterface spinal_iface_;
};

}  // namespace flight_controllers

#endif
