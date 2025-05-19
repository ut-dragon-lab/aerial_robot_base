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
  spinal_state_estimator_.getAttEstimator()->setAcc(acc_x, acc_y, acc_z);
  spinal_state_estimator_.getAttEstimator()->setGyro(gyro_x, gyro_y, gyro_z);
}

void SpinalInterface::setMagValue(double mag_x, double mag_y, double mag_z) {
  spinal_state_estimator_.getAttEstimator()->setMag(mag_x, mag_y, mag_z);
}

void SpinalInterface::setGroundTruthStates(double q_x, double q_y, double q_z, double q_w, double w_x, double w_y,
                                           double w_z) {
  ap::Quaternion q(q_w, q_x, q_y, q_z);
  ap::Matrix3f rot;
  q.rotation_matrix(rot);
  ap::Vector3f ang_vel(w_x, w_y, w_z);
  spinal_state_estimator_.getAttEstimator()->setGroundTruthStates(rot, ang_vel);
}

void SpinalInterface::useGroundTruth(bool flag) { spinal_state_estimator_.getAttEstimator()->useGroundTruth(flag); }

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
