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
#pragma once

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_simulation/noise_model.h>
#include <tinyxml2.h>
#include <urdf/model.h>

#include <ignition/math/Vector3.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace hardware_interface {

class RotorHandle {
 public:
  /**
   * @brief Construct a new RotorHandle with ROS2 node and URDF joint
   */
  RotorHandle(const rclcpp::Node::SharedPtr& node, urdf::JointConstSharedPtr urdf_joint)
      : force_(std::make_shared<double>(0.0)), max_pwm_(2000) {
    name_ = urdf_joint->name;
    direction_ = urdf_joint->axis.z;

    // XML-based parameter from aerial_robot_model
    auto doc = aerial_robot_model::RobotModel::getRobotModelXml("robot_description", node);
    auto* elem = doc->RootElement()->FirstChildElement("m_f_rate");
    if (elem && elem->QueryDoubleAttribute("value", &m_f_rate_) == tinyxml2::XML_SUCCESS) {
      RCLCPP_DEBUG(node->get_logger(), "m_f_rate: %f", m_f_rate_);
    } else {
      RCLCPP_ERROR(node->get_logger(), "RotorHandle: failed to load m_f_rate from model XML");
    }

    // Motor parameters
    node->declare_parameter("motor_info.rotor_damping_rate", 1.0);
    node->get_parameter("motor_info.rotor_damping_rate", rotor_damping_rate_);
    node->declare_parameter("motor_info.rotor_force_noise", 0.0);
    node->get_parameter("motor_info.rotor_force_noise", rotor_force_noise_);
    node->declare_parameter("motor_info.dual_rotor_moment_noise", 0.0);
    node->get_parameter("motor_info.dual_rotor_moment_noise", dual_rotor_moment_noise_);
    node->declare_parameter("motor_info.speed_rate", 1.0);
    node->get_parameter("motor_info.speed_rate", speed_rate_);
  }

  inline std::string getName() const { return name_; }
  double getForce() const { return *force_; }

  inline void setForce(double target_force, bool direct = false) {
    if (direct || target_force < 1e-6) {
      *force_ = target_force;
    } else {
      double current = *force_;
      *force_ = (1 - rotor_damping_rate_) * current + rotor_damping_rate_ * target_force +
                aerial_robot_simulation::gaussianKernel(rotor_force_noise_);
    }
  }

  inline ignition::math::Vector3d getTorque() const {
    // X=moment noise, Z=force * direction * m_f_rate_
    return ignition::math::Vector3d(aerial_robot_simulation::gaussianKernel(dual_rotor_moment_noise_), 0,
                                    getForce() * direction_ * m_f_rate_);
  }

  inline double getSpeed() const { return *force_ * speed_rate_; }

 private:
  std::string name_;
  std::shared_ptr<double> force_;
  double direction_{1.0};
  double m_f_rate_{0.0};
  double speed_rate_{1.0};

  double rotor_damping_rate_{1.0};
  double rotor_force_noise_{0.0};
  double dual_rotor_moment_noise_{0.0};

  double max_pwm_;
};

}  // namespace hardware_interface
