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

  static const std::array<std::string, 10> imu_ifaces = {
      "orientation.x",         "orientation.y",        "orientation.z",      "orientation.w",
      "angular_velocity.x",    "angular_velocity.y",   "angular_velocity.z", "linear_acceleration.x",
      "linear_acceleration.y", "linear_acceleration.z"};

  const std::string imu = get_node()->get_parameter("imu").as_string();
  for (const auto &iface : imu_ifaces) {
    config.names.push_back(imu + "/" + iface);
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

  spinal_iface_.setImuValue(acc_x, acc_y, acc_z, ang_x, ang_y, ang_z);
  spinal_iface_.stateEstimate();
  return controller_interface::return_type::OK;
}

}  // namespace flight_controllers

PLUGINLIB_EXPORT_CLASS(flight_controllers::SimulationAttitudeController, controller_interface::ControllerInterface)
