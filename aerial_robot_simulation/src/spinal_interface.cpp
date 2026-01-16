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

#include "aerial_robot_simulation/spinal_interface.h"

namespace hardware_interface {

SpinalInterface::SpinalInterface() { on_ground_ = true; }

bool SpinalInterface::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
  node_ = node;
  spinal_state_estimator_.init(node_);
  is_init_ = true;
  return true;
}

void SpinalInterface::stateEstimate() {
  if (on_ground_) {
    setImuValue(0.0, 0.0, 9.8,  // temp
                0.0, 0.0, 0.0);
  }
  spinal_state_estimator_.update();
}

void SpinalInterface::setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y,
                                  double gyro_z) {
  spinal_state_estimator_.getEstimator()->getAttEstimator()->setAcc(acc_x, acc_y, acc_z);
  spinal_state_estimator_.getEstimator()->getAttEstimator()->setGyro(gyro_x, gyro_y, gyro_z);
}

void SpinalInterface::setMagValue(double mag_x, double mag_y, double mag_z) {
  spinal_state_estimator_.getEstimator()->getAttEstimator()->setMag(mag_x, mag_y, mag_z);
}

void SpinalInterface::setGroundTruthStates(double q_x, double q_y, double q_z, double q_w, double w_x, double w_y,
                                           double w_z) {
  ap::Quaternion q(q_w, q_x, q_y, q_z);
  ap::Matrix3f rot;
  q.rotation_matrix(rot);
  ap::Vector3f ang_vel(w_x, w_y, w_z);
  spinal_state_estimator_.getEstimator()->getAttEstimator()->setGroundTruthStates(rot, ang_vel);
}

void SpinalInterface::useGroundTruth(bool flag) { spinal_state_estimator_.getEstimator()->getAttEstimator()->useGroundTruth(flag); }

}  // namespace hardware_interface

// namespace rotor_limits_interface
// {

//   EffortRotorSaturationHandle::EffortRotorSaturationHandle(
//                                                            const hardware_interface::RotorHandle & jh,
//                                                            urdf::JointConstSharedPtr urdf_joint)
//     : jh_(jh)
//   {
//     if (!urdf_joint)
//       {
//         throw std::runtime_error(
//                                  "Cannot load param for rotor '" + getName() + "'. No URDF segment found.");
//       }
//     min_force_ = urdf_joint->limits->lower;
//     max_force_ = urdf_joint->limits->upper;
//     RCLCPP_DEBUG(
//                  rclcpp::get_logger("spinal_interface"),
//                  "Loaded rotor '%s' with limits [%f, %f]",
//                  jh_.getName().c_str(), min_force_, max_force_);
//   }

// } // namespace rotor_limits_interface
