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

#include <aerial_robot_simulation/rotor_handle.h>
#include <state_estimate/state_estimate.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <stdexcept>

namespace hardware_interface {

class SpinalInterface {
 public:
  SpinalInterface();

  bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

  uint8_t getJointNum() const { return joint_num_; }
  bool isInit() const { return is_init_; }

  void useGroundTruth(bool flag);
  void setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y, double gyro_z);
  void setMagValue(double mag_x, double mag_y, double mag_z);
  void setGroundTruthStates(double q_x, double q_y, double q_z, double q_w, double w_x, double w_y, double w_z);

  void stateEstimate();
  inline void onGround(bool flag) { on_ground_ = flag; }

  StateEstimate* getEstimatorPtr() { return &spinal_state_estimator_; }

 private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  uint8_t joint_num_{0};
  bool on_ground_{true};
  StateEstimate spinal_state_estimator_;
  bool is_init_{false};
};

}  // namespace hardware_interface

// namespace rotor_limits_interface
// {

//   class EffortRotorSaturationHandle
//   {
//   public:
//     EffortRotorSaturationHandle(
//                                 const hardware_interface::RotorHandle & jh,
//                                 urdf::JointConstSharedPtr urdf_joint);

//     std::string getName() const { return jh_.getName(); }

//     void enforceLimits(const rclcpp::Duration & period)
//     {
//       if (jh_.getForce() == 0) return;
//       jh_.setForce(
//                    std::clamp(jh_.getForce(), min_force_, max_force_));
//     }

//   private:
//     hardware_interface::RotorHandle jh_;
//     double max_force_{0.0};
//     double min_force_{0.0};
//   };

//   template <class HandleType>
//   class RotorLimitsInterface : public hardware_interface::ResourceManager<HandleType>
//   {
//   public:
//     HandleType getHandle(const std::string& name)
//     {
//       // Rethrow exception with a meaningful type
//       try
//         {
//           return this->hardware_interface::ResourceManager<HandleType>::getHandle(name);
//         }
//       catch(const std::logic_error& e)
//         {
//           throw joint_limits_interface::JointLimitsInterfaceException(e.what());
//         }
//     }

//     void enforceLimits(const ros::Duration& period)
//     {
//       typedef typename hardware_interface::ResourceManager<HandleType>::ResourceMap::iterator ItratorType;
//       for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
//         {
//           it->second.enforceLimits(period);
//         }
//     }
//   };

//   class EffortRotorSaturationInterface : public RotorLimitsInterface<EffortRotorSaturationHandle> {};
// };
