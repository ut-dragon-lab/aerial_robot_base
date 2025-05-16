#ifndef AERIAL_ROBOT_IGN_HPP_
#define AERIAL_ROBOT_IGN_HPP_

#include <ignition/gazebo/System.hh>
// #include <ignition/gazebo/ISystemConfigure.hh>
// #include <ignition/gazebo/ISystemPreUpdate.hh>
#include <aerial_robot_simulation/aerial_robot_hw_sim.h>
#include <tinyxml2.h>

#include <chrono>
#include <controller_manager/controller_manager.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/MagneticField.hh>
#include <ignition/gazebo/components/Magnetometer.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
namespace aerial_robot_simulation {

/// Ignition Gazebo plugin to publish ground truth and mocap, and inject simulation context
class AerialRobotIgn : public ignition::gazebo::System,
                       public ignition::gazebo::ISystemConfigure,
                       public ignition::gazebo::ISystemPreUpdate,
                       public ignition::gazebo::ISystemPostUpdate {
 public:
  /// Called once when the plugin is loaded
  void Configure(const ignition::gazebo::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 ignition::gazebo::EntityComponentManager &ecm, ignition::gazebo::EventManager &eventMgr) override;

  /// Called each simulation iteration before physics update
  void PreUpdate(const ignition::gazebo::UpdateInfo &info, ignition::gazebo::EntityComponentManager &ecm) override;

  // Called each simulation iteration after physics update
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
                  const ignition::gazebo::EntityComponentManager &ecm) override;

 private:
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  std::shared_ptr<AerialRobotHwSim> hw_iface_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;

  ignition::gazebo::Entity model_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity baselink_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity imu_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity mag_entity_{ignition::gazebo::kNullEntity};

  std::string baselink_name_;
  std::string imuSensorName_;
  std::string magSensorName_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pub_;

  std::string robot_description_;

  // Timing
  rclcpp::Time ros_start_time_;
  rclcpp::Time ros_last_time_;
  rclcpp::Time last_ground_truth_time_, last_mocap_time_;
  double ground_truth_pub_rate_, mocap_pub_rate_;

  // Noise & drift parameters
  double mocap_pos_noise_, mocap_rot_noise_;
  double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
  double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
  double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
};

}  // namespace aerial_robot_simulation

#endif  // AERIAL_ROBOT_IGNITION_PLUGIN_HPP_
