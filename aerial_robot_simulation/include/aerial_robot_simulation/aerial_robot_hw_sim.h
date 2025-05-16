#ifndef AERIAL_ROBOT_HW_SIM_H_
#define AERIAL_ROBOT_HW_SIM_H_

#include <aerial_robot_simulation/spinal_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

using hardware_interface::CallbackReturn;

namespace aerial_robot_simulation {

class AerialRobotHwSim : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AerialRobotHwSim)

  // Initialize hardware from URDF/parameters
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  // Expose state (position, velocity, effort, etc.)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Expose command interfaces (effort, velocity, position, etc.)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Lifecycle transitions
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Read sensor / simulation data into state interfaces_
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Write commands (TODO)
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void configSpinalIface(rclcpp::Node::SharedPtr node);

  void setImuData(const ignition::math::Vector3d& linear_acceleration,
                  const ignition::math::Vector3d& angular_velocity);

  void setMagData(const ignition::math::Vector3d& magnetic_field);

 private:
  // Number of rotors
  size_t rotor_n_dof_;

  // Rotor state & command buffers
  std::vector<double> rotor_efforts_;          // current effort state
  std::vector<double> rotor_efforts_cmd_;      // commanded effort
  std::vector<double> rotor_velocity_states_;  // current rotor speed

  // Spinal interface (no inheritance necessary)
  hardware_interface::SpinalInterface spinal_iface_;

  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Control mode: FORCE, VEL, POS
  uint8_t control_mode_;
  geometry_msgs::msg::TwistStamped cmd_vel_;
  geometry_msgs::msg::PoseStamped cmd_pos_;

  std::array<double, 3> imu_acc_{};
  std::array<double, 3> imu_gyro_{};
  std::array<double, 3> mag_field_{};

  hardware_interface::HardwareInfo hw_info_;
};

}  // namespace aerial_robot_simulation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aerial_robot_simulation::AerialRobotHwSim, hardware_interface::SystemInterface)

#endif  // AERIAL_ROBOT_SYSTEM_HPP_
