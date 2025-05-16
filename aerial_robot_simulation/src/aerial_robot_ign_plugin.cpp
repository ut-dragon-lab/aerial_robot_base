#include "aerial_robot_simulation/aerial_robot_ign_plugin.h"

namespace aerial_robot_simulation {

void AerialRobotIgn::Configure(const ignition::gazebo::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                               ignition::gazebo::EntityComponentManager &ecm,
                               ignition::gazebo::EventManager &eventMgr) {
  // Initialize underlying ROS2 node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"use_sim_time", true}});
  node_ = std::make_shared<rclcpp::Node>("aerial_robot_ign", opts);
  // node_ = std::make_shared<rclcpp::Node>("aerial_robot_ign");

  hw_iface_ = std::make_shared<AerialRobotHwSim>();
  hw_iface_->configSpinalIface(node_);
  // auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      exec_, node_->get_name(), node_->get_namespace(), rclcpp::NodeOptions());

  model_entity_ = entity;

  node_->declare_parameter("robot_description", robot_description_);
  auto robot_model_xml = aerial_robot_model::RobotModel::getRobotModelXml("robot_description", node_);

  if (!robot_model_xml) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load or parse robot_description parameter");
    baselink_name_ = std::string("fc");
  }

  // tinyxml2::XMLElement* robot_elem = robot_model_xml->FirstChildElement("robot");
  // if (!robot_elem) {
  //   RCLCPP_ERROR(node_->get_logger(), "<robot> tag missing in plugin SDF");
  //   return;
  // }

  // tinyxml2::XMLElement* baselink_elem = robot_elem->FirstChildElement("baselink");
  // if (!baselink_elem) {
  //   RCLCPP_ERROR(node_->get_logger(), "<baselink> tag missing in plugin SDF");
  //   return;
  // }

  // const char* name_attr = baselink_elem->Attribute("name");
  // if (!name_attr) {
  //   RCLCPP_ERROR(node_->get_logger(), "\"name\" attribute missing in <baselink> tag");
  //   return;
  // }

  // auto baselink_attr = robot_model_xml->FirstChildElement("robot")->FirstChildElement("baselink");
  // if(baselink_attr){
  //   baselink_name_ = std::string(baselink_attr->Attribute("name"));
  // } else {
  //   RCLCPP_ERROR(node_->get_logger(), "<baselink> tag missing in plugin SDF");
  // }

  baselink_entity_ = ecm.EntityByComponents(ignition::gazebo::components::Name(baselink_name_));
  imu_entity_ = ecm.EntityByComponents(ignition::gazebo::components::Name("spinal_imu"));
  mag_entity_ = ecm.EntityByComponents(ignition::gazebo::components::Name("magnetometer"));

  // Setup publishers
  ground_truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 1);
  mocap_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("mocap/pose", 1);

  // Load parameters (noise, drift rates, publish rates)
  ground_truth_pub_rate_ = node_->declare_parameter<double>("ground_truth_pub_rate", 0.01);           // [sec]
  ground_truth_pos_noise_ = node_->declare_parameter<double>("ground_truth_pos_noise", 0.0);          // [m]
  ground_truth_vel_noise_ = node_->declare_parameter<double>("ground_truth_vel_noise", 0.0);          // [m/s]
  ground_truth_rot_noise_ = node_->declare_parameter<double>("ground_truth_rot_noise", 0.0);          // [rad]
  ground_truth_angular_noise_ = node_->declare_parameter<double>("ground_truth_angular_noise", 0.0);  // [rad/s]
  ground_truth_rot_drift_ = node_->declare_parameter<double>("ground_truth_rot_drift", 0.0);          // [rad]
  ground_truth_vel_drift_ = node_->declare_parameter<double>("ground_truth_vel_drift", 0.0);          // [m/s]
  ground_truth_angular_drift_ = node_->declare_parameter<double>("ground_truth_angular_drift", 0.0);  // [rad/s]
  ground_truth_rot_drift_frequency_ =
      node_->declare_parameter<double>("ground_truth_rot_drift_frequency", 0.0);  // [1/s]
  ground_truth_vel_drift_frequency_ =
      node_->declare_parameter<double>("ground_truth_vel_drift_frequency", 0.0);  // [1/s]
  ground_truth_angular_drift_frequency_ =
      node_->declare_parameter<double>("ground_truth_angular_drift_frequency", 0.0);  // [1/s]
  mocap_pub_rate_ = node_->declare_parameter<double>("mocap_pub_rate", 0.01);         // [sec]
  mocap_pos_noise_ = node_->declare_parameter<double>("mocap_pos_noise", 0.001);      // [m]
  mocap_rot_noise_ = node_->declare_parameter<double>("mocap_rot_noise", 0.001);      // [rad]

  ros_start_time_ = node_->now();
  last_ground_truth_time_ = ros_start_time_;
  last_mocap_time_ = ros_start_time_;
  ros_last_time_ = node_->now();
}

void AerialRobotIgn::PreUpdate(const ignition::gazebo::UpdateInfo &info,
                               ignition::gazebo::EntityComponentManager &ecm) {
  auto poseComp = ecm.Component<ignition::gazebo::components::Pose>(baselink_entity_);
  auto velComp = ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(baselink_entity_);
  auto angComp = ecm.Component<ignition::gazebo::components::AngularVelocity>(baselink_entity_);

  rclcpp::Time now = node_->now();
  rclcpp::Duration period = now - ros_last_time_;
  ros_last_time_ = now;
  if (poseComp && velComp && angComp) {
    auto pose = poseComp->Data();
    auto linVel = velComp->Data();
    auto angVel = angComp->Data();
    if (period.seconds() > ground_truth_pub_rate_) {
      last_ground_truth_time_ = now;
      nav_msgs::msg::Odometry odom;
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      odom.header.stamp = rclcpp::Time(static_cast<int64_t>(ns), RCL_ROS_TIME);
      odom.header.frame_id = "world";
      odom.child_frame_id = baselink_name_;

      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();

      odom.twist.twist.linear.x = linVel.X();
      odom.twist.twist.linear.y = linVel.Y();
      odom.twist.twist.linear.z = linVel.Z();
      odom.twist.twist.angular.x = angVel.X();
      odom.twist.twist.angular.y = angVel.Y();
      odom.twist.twist.angular.z = angVel.Z();

      ground_truth_pub_->publish(odom);
    }
  }

  // Read IMU data
  auto accComp = ecm.Component<ignition::gazebo::components::LinearAcceleration>(imu_entity_);
  auto gyroComp = ecm.Component<ignition::gazebo::components::AngularVelocity>(imu_entity_);
  if (accComp && gyroComp) {
    hw_iface_->setImuData(accComp->Data(), gyroComp->Data());
  }

  // Read Magnetometer data
  auto magComp = ecm.Component<ignition::gazebo::components::MagneticField>(mag_entity_);
  if (magComp) {
    hw_iface_->setMagData(magComp->Data());
  }
  controller_manager_->read(now, period);
  controller_manager_->update(now, period);
}

void AerialRobotIgn::PostUpdate(const ignition::gazebo::UpdateInfo &info,
                                const ignition::gazebo::EntityComponentManager &ecm) {
  // Compute current ROS time and period
  rclcpp::Time now = node_->now();
  rclcpp::Duration period = now - ros_last_time_;

  // Call ros2_control write
  controller_manager_->write(now, period);
}

}  // namespace aerial_robot_simulation

IGNITION_ADD_PLUGIN(aerial_robot_simulation::AerialRobotIgn, ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure, ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemPostUpdate)
