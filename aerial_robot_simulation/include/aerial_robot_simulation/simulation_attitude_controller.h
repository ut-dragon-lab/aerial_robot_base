#ifndef SIMULATION_ATTITUDE_CONTROLLER_H
#define SIMULATION_ATTITUDE_CONTROLLER_H

#include <aerial_robot_simulation/spinal_interface.h>

#include <boost/scoped_ptr.hpp>
// #include <flight_control/flight_control.h>
#include <urdf/model.h>

#include <memory>
#include <spinal/msg/desire_coord.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

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
