#include "aerial_robot_simulation/aerial_robot_hw_sim.h"

namespace aerial_robot_simulation {

CallbackReturn AerialRobotHwSim::on_init(const hardware_interface::HardwareInfo& info) {
  hw_info_ = info;
  // IMU
  imu_acc_ = {0.0, 0.0, 0.0};
  imu_gyro_ = {0.0, 0.0, 0.0};
  // Magnetometer
  mag_field_ = {0.0, 0.0, 0.0};

  // Parse rotor count
  rotor_n_dof_ = info_.joints.size();
  rotor_efforts_.assign(rotor_n_dof_, 0.0);
  rotor_efforts_cmd_.assign(rotor_n_dof_, 0.0);
  rotor_velocity_states_.assign(rotor_n_dof_, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AerialRobotHwSim::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  // IMU data
  interfaces.emplace_back(hw_info_.name + "/imu/ax", "acceleration", &imu_acc_[0]);
  interfaces.emplace_back(hw_info_.name + "/imu/ay", "acceleration", &imu_acc_[1]);
  interfaces.emplace_back(hw_info_.name + "/imu/az", "acceleration", &imu_acc_[2]);
  interfaces.emplace_back(hw_info_.name + "/imu/gx", "velocity", &imu_gyro_[0]);
  interfaces.emplace_back(hw_info_.name + "/imu/gy", "velocity", &imu_gyro_[1]);
  interfaces.emplace_back(hw_info_.name + "/imu/gz", "velocity", &imu_gyro_[2]);

  // Magnetometer data
  interfaces.emplace_back(hw_info_.name + "/mag/x", "magnetic", &mag_field_[0]);
  interfaces.emplace_back(hw_info_.name + "/mag/y", "magnetic", &mag_field_[1]);
  interfaces.emplace_back(hw_info_.name + "/mag/z", "magnetic", &mag_field_[2]);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AerialRobotHwSim::export_command_interfaces() {
  // No command interfaces for now; controllers will be added later
  return {};
}

CallbackReturn AerialRobotHwSim::on_activate(const rclcpp_lifecycle::State& /*previous*/) {
  return CallbackReturn::SUCCESS;
}

CallbackReturn AerialRobotHwSim::on_deactivate(const rclcpp_lifecycle::State& /*previous*/) {
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AerialRobotHwSim::read(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/) {
  RCLCPP_ERROR(node_->get_logger(), "called1");
  // State buffers are updated via bridge methods
  if (spinal_iface_.isInit()) {
    spinal_iface_.setImuValue(imu_acc_[0], imu_acc_[1], imu_acc_[2], imu_gyro_[0], imu_gyro_[1], imu_gyro_[2]);
    spinal_iface_.setMagValue(mag_field_[0], mag_field_[1], mag_field_[2]);
    spinal_iface_.stateEstimate();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AerialRobotHwSim::write(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/) {
  // No write behavior for now
  return hardware_interface::return_type::OK;
}

void AerialRobotHwSim::setImuData(const ignition::math::Vector3d& linear_acceleration,
                                  const ignition::math::Vector3d& angular_velocity) {
  imu_acc_[0] = linear_acceleration.X();
  imu_acc_[1] = linear_acceleration.Y();
  imu_acc_[2] = linear_acceleration.Z();
  imu_gyro_[0] = angular_velocity.X();
  imu_gyro_[1] = angular_velocity.Y();
  imu_gyro_[2] = angular_velocity.Z();
}

void AerialRobotHwSim::setMagData(const ignition::math::Vector3d& magnetic_field) {
  mag_field_[0] = magnetic_field.X();
  mag_field_[1] = magnetic_field.Y();
  mag_field_[2] = magnetic_field.Z();
}

void AerialRobotHwSim::configSpinalIface(rclcpp::Node::SharedPtr node) {
  node_ = node;
  spinal_iface_.init(node_, rotor_n_dof_);
}

}  // namespace aerial_robot_simulation
