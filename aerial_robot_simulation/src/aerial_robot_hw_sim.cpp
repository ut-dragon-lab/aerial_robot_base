// -*- mode: c++ -*-
/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *   3. Neither the name of the DRAGON Laboratory nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 This file was implemented with reference to the interface of GazeboSimSystemclass in gz_ros2_controller
(https://github.com/ros-controls/gz_ros2_control/tree/humble). In addition to the default interface, bridges for
magnetometer and pseudo-motion capture are incorporated.
*/

#include "aerial_robot_simulation/aerial_robot_hw_sim.h"

#include <ignition/msgs/imu.pb.h>

#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Magnetometer.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/math.hh>
#include <ignition/transport/Node.hh>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#define GZ_TRANSPORT_NAMESPACE ignition::transport::
#define GZ_MSGS_NAMESPACE ignition::msgs::
#include <hardware_interface/hardware_info.hpp>

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position{ 0.0 };

  /// \brief Current joint velocity
  double joint_velocity{ 0.0 };

  /// \brief Current joint effort
  double joint_effort{ 0.0 };

  /// \brief Current cmd joint position
  double joint_position_cmd{ 0.0 };

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd{ 0.0 };

  /// \brief Current cmd joint effort
  double joint_effort_cmd{ 0.0 };

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated{ false };

  /// \brief flag if this joint command represents rotor thrust
  bool is_rotor{ false };

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint{ sim::kNullEntity };

  /// \brief parent link where rotor yaw drag torque is applied
  sim::Entity sim_parent_link{ sim::kNullEntity };

  /// \brief child link where rotor thrust is applied
  sim::Entity sim_child_link{ sim::kNullEntity };

  /// \brief rotor spin direction, taken from the joint axis z sign
  double rotor_direction{ 1.0 };

  /// \brief force to yaw torque conversion rate
  double m_f_rate{ 0.0 };

  /// \brief visual joint speed per thrust force [rad/s/N]
  double rotor_visual_speed_rate{ 1.0 };

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method{
    gz_ros2_control::GazeboSimSystemInterface::NONE
  };
};

struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  std::vector<std::string> interfaces_to_mimic;
};

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const GZ_MSGS_NAMESPACE IMU &_msg);
};

class MagData
{
public:
  std::string name{};
  std::string topicName{};
  sim::Entity sim_mag_sensor_ = sim::kNullEntity;
  std::array<double, 3> mag_sensor_data_;
  void OnMag(const GZ_MSGS_NAMESPACE Magnetometer &_msg);
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU &_msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

void MagData::OnMag(const GZ_MSGS_NAMESPACE Magnetometer &_msg)
{
  this->mag_sensor_data_[0] = _msg.field_tesla().x();
  this->mag_sensor_data_[1] = _msg.field_tesla().y();
  this->mag_sensor_data_[2] = _msg.field_tesla().z();
}

class gz_ros2_control::AerialRobotHwSimPrivate
{
public:
  AerialRobotHwSimPrivate() = default;

  ~AerialRobotHwSimPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus .
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief vector with the mags .
  std::vector<std::shared_ptr<MagData>> mags_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager *ecm;

  /// \brief controller update rate
  int *update_rate;

  /// \brief Ignition communication node.
  GZ_TRANSPORT_NAMESPACE Node node;

  /// \brief mapping of mimicked joints to index of joint they mimic
  std::vector<MimicJoint> mimic_joints_;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;

  // Noise nad drift parameters
  double mocap_rot_noise_, mocap_pos_noise_;
  double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
  ignition::math::Vector3d ground_truth_rot_curr_drift_, ground_truth_vel_curr_drift_, ground_truth_angular_curr_drift_;
  double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
  double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;

  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;

  // Publish rates
  double ground_truth_pub_rate_;
  double mocap_pub_rate_;

  // Node to get ros2 param
  rclcpp::Node::SharedPtr param_node_;

  // Clients
  std::shared_ptr<rclcpp::SyncParametersClient> client_ptr_;

  // Time
  rclcpp::Time last_ground_truth_time_, last_mocap_time_, last_pos_read_time_;

  // Last position of baselink frame
  ignition::math::Vector3d last_baselink_pos_;
};

namespace gz_ros2_control
{

bool AerialRobotHwSim::initSim(rclcpp::Node::SharedPtr &model_nh, std::map<std::string, sim::Entity> &enableJoints,
                               const hardware_interface::HardwareInfo &hardware_info, sim::EntityComponentManager &ecm,
                               int &update_rate)
{
  this->dataPtr = std::make_unique<AerialRobotHwSimPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = &update_rate;

  // Publisher for world pose/twist
  this->dataPtr->mocap_pub_ = this->nh_->create_publisher<geometry_msgs::msg::PoseStamped>("mocap/pose", 10);
  this->dataPtr->ground_truth_pub_ = this->nh_->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);

  // Clients to get parameters for simulation
  this->dataPtr->param_node_ = std::make_shared<rclcpp::Node>("param_client_node");
  std::string robot_ns = this->nh_->get_namespace();
  this->dataPtr->client_ptr_ = std::make_shared<rclcpp::SyncParametersClient>(this->dataPtr->param_node_,
                                                                              robot_ns + "/sim_param_server");
  if (!this->dataPtr->client_ptr_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR_STREAM(this->dataPtr->param_node_->get_logger(), "[sim] Param client was not found.");
    return 1;
  }
  querySimPrams();

  RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                     "[sim] The mocap rotational noise has been set to: " << this->dataPtr->mocap_rot_noise_);

  RCLCPP_DEBUG(this->nh_->get_logger(), "[sim] n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  constexpr double default_gain = 0.1;

  try
  {
    this->dataPtr->position_proportional_gain_ = this->nh_->declare_parameter<double>("position_proportional_gain",
                                                                                      default_gain);
  }
  catch (rclcpp::exceptions::ParameterAlreadyDeclaredException &ex)
  {
    this->nh_->get_parameter("position_proportional_gain", this->dataPtr->position_proportional_gain_);
  }

  RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] The position_proportional_gain has been set to: "
                                                  << this->dataPtr->position_proportional_gain_);

  if (this->dataPtr->n_dof_ == 0)
  {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "[sim] There is no joint available");
    return false;
  }

  double m_f_rate = 0.0;
  const auto m_f_rate_it = hardware_info.hardware_parameters.find("m_f_rate");
  if (m_f_rate_it != hardware_info.hardware_parameters.end())
  {
    try
    {
      m_f_rate = std::stod(m_f_rate_it->second);
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Loaded m_f_rate: " << m_f_rate);
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "[sim] Invalid m_f_rate hardware parameter: " << m_f_rate_it->second);
    }
  }
  else
  {
    RCLCPP_WARN(this->nh_->get_logger(), "[sim] m_f_rate hardware parameter is not set; yaw drag torque is 0");
  }

  double rotor_visual_speed_rate = 1.0;
  const auto rotor_visual_speed_rate_it = hardware_info.hardware_parameters.find("rotor_visual_speed_rate");
  if (rotor_visual_speed_rate_it != hardware_info.hardware_parameters.end())
  {
    try
    {
      rotor_visual_speed_rate = std::stod(rotor_visual_speed_rate_it->second);
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Loaded rotor_visual_speed_rate: " << rotor_visual_speed_rate);
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "[sim] Invalid rotor_visual_speed_rate hardware parameter: "
                                                      << rotor_visual_speed_rate_it->second);
    }
  }

  auto find_link_by_name = [&ecm](const std::string &link_name)
  {
    sim::Entity result = sim::kNullEntity;
    ecm.Each<sim::components::Link, sim::components::Name>(
        [&](const sim::Entity &entity, const sim::components::Link *, const sim::components::Name *name) -> bool
        {
          if (name->Data() == link_name)
          {
            result = entity;
            return false;
          }
          return true;
        });
    return result;
  };

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++)
  {
    auto &joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it = enableJoints.find(joint_name);
    if (it == enableJoints.end())
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "[sim] Skipping joint in the URDF named '"
                                                      << joint_name << "' which is not in the gazebo model.e");
      continue;
    }

    sim::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;

    sim::Joint joint(simjoint);
    const auto parent_link_name = joint.ParentLinkName(ecm);
    if (parent_link_name)
    {
      this->dataPtr->joints_[j].sim_parent_link = find_link_by_name(*parent_link_name);
    }

    const auto child_link_name = joint.ChildLinkName(ecm);
    if (child_link_name)
    {
      this->dataPtr->joints_[j].sim_child_link = find_link_by_name(*child_link_name);
    }

    const auto joint_axes = joint.Axis(ecm);
    if (joint_axes && !joint_axes->empty())
    {
      const double axis_z = joint_axes->front().Xyz().Z();
      this->dataPtr->joints_[j].rotor_direction = axis_z < 0.0 ? -1.0 : 1.0;
    }
    this->dataPtr->joints_[j].m_f_rate = m_f_rate;
    this->dataPtr->joints_[j].rotor_visual_speed_rate = rotor_visual_speed_rate;

    // Create joint position component if one doesn't exist
    if (!ecm.EntityHasComponentType(simjoint, sim::components::JointPosition().TypeId()))
    {
      ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!ecm.EntityHasComponentType(simjoint, sim::components::JointVelocity().TypeId()))
    {
      ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint force component if one doesn't exist
    if (!ecm.EntityHasComponentType(simjoint, sim::components::JointForce().TypeId()))
    {
      ecm.CreateComponent(simjoint, sim::components::JointForce());
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Loading joint: " << joint_name);

    std::string suffix = "";

    // Check if joint is mimicked
    if (joint_info.parameters.find("mimic") != joint_info.parameters.end())
    {
      const auto mimicked_joint = joint_info.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(hardware_info.joints.begin(), hardware_info.joints.end(),
                                                  [&mimicked_joint](const hardware_interface::ComponentInfo &info)
                                                  { return info.name == mimicked_joint; });
      if (mimicked_joint_it == hardware_info.joints.end())
      {
        throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }

      MimicJoint mimic_joint;
      mimic_joint.joint_index = j;
      mimic_joint.mimicked_joint_index = std::distance(hardware_info.joints.begin(), mimicked_joint_it);
      auto param_it = joint_info.parameters.find("multiplier");
      if (param_it != joint_info.parameters.end())
      {
        mimic_joint.multiplier = std::stod(joint_info.parameters.at("multiplier"));
      }
      else
      {
        mimic_joint.multiplier = 1.0;
      }

      // Check joint info of mimicked joint
      auto &joint_info_mimicked = hardware_info.joints[mimic_joint.mimicked_joint_index];
      const auto state_mimicked_interface = std::find_if(
          joint_info_mimicked.state_interfaces.begin(), joint_info_mimicked.state_interfaces.end(),
          [&mimic_joint](const hardware_interface::InterfaceInfo &interface_info)
          {
            bool pos = interface_info.name == "position";
            if (pos)
            {
              mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_POSITION);
            }
            bool vel = interface_info.name == "velocity";
            if (vel)
            {
              mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_VELOCITY);
            }
            bool eff = interface_info.name == "effort";
            if (vel)
            {
              mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_EFFORT);
            }
            return pos || vel || eff;
          });
      if (state_mimicked_interface == joint_info_mimicked.state_interfaces.end())
      {
        throw std::runtime_error(std::string("For mimic joint '") + joint_info.name +
                                 "' no state interface was found in mimicked joint '" + mimicked_joint + " ' to mimic");
      }
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Joint '" << joint_name << "'is mimicking joint '"
                                                                  << mimicked_joint
                                                                  << "' with multiplier: " << mimic_joint.multiplier);
      this->dataPtr->mimic_joints_.push_back(mimic_joint);
      suffix = "_mimic";
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value = [this, joint_name](const hardware_interface::InterfaceInfo &interface_info)
    {
      double initial_value{ 0.0 };
      if (!interface_info.initial_value.empty())
      {
        try
        {
          initial_value = std::stod(interface_info.initial_value);
          RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
        }
        catch (std::invalid_argument &)
        {
          RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                              "Failed converting initial_value string to real number for the joint "
                                  << joint_name << " and state interface " << interface_info.name
                                  << ". Actual value of parameter: " << interface_info.initial_value
                                  << ". Initial value will be set to 0.0");
          throw std::invalid_argument("[sim] Failed converting initial_value string");
        }
      }
      return initial_value;
    };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // Register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i)
    {
      if (joint_info.state_interfaces[i].name == "position")
      {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(joint_name + suffix, hardware_interface::HW_IF_POSITION,
                                                      &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity")
      {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(joint_name + suffix, hardware_interface::HW_IF_VELOCITY,
                                                      &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort")
      {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(joint_name + suffix, hardware_interface::HW_IF_EFFORT,
                                                      &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // Register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i)
    {
      if (joint_info.command_interfaces[i].name == "position")
      {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(joint_name + suffix, hardware_interface::HW_IF_POSITION,
                                                        &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position))
        {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      }
      else if (joint_info.command_interfaces[i].name == "velocity")
      {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(joint_name + suffix, hardware_interface::HW_IF_VELOCITY,
                                                        &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity))
        {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      }
      else if (joint_info.command_interfaces[i].name == "effort")
      {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
        this->dataPtr->joints_[j].is_rotor = joint_name.rfind("rotor", 0) == 0;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(joint_name + suffix, hardware_interface::HW_IF_EFFORT,
                                                        &this->dataPtr->joints_[j].joint_effort_cmd);
        if (this->dataPtr->joints_[j].is_rotor)
        {
          if (this->dataPtr->joints_[j].sim_child_link == sim::kNullEntity)
          {
            RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                               "[sim] Rotor joint " << joint_name << " has no child link for thrust");
          }
          else
          {
            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Rotor thrust command registered for " << joint_name);
          }
        }
        if (!std::isnan(initial_effort))
        {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
      // Independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position))
      {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[j].sim_joint,
                                            sim::components::JointPositionReset({ initial_position }));
      }
      if (!std::isnan(initial_velocity))
      {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[j].sim_joint,
                                            sim::components::JointVelocityReset({ initial_velocity }));
      }
    }

    // Check if joint is actuated (has command interfaces) or passive
    this->dataPtr->joints_[j].is_actuated = (joint_info.command_interfaces.size() > 0);
  }

  registerSensors(hardware_info);

  return true;
}

void AerialRobotHwSim::registerSensors(const hardware_interface::HardwareInfo &hardware_info)
{
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> sensor_components_;

  for (unsigned int j = 0; j < n_sensors; j++)
  {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    sensor_components_.push_back(component);
  }
  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures

  // Register IMU
  this->dataPtr->ecm->Each<sim::components::Imu, sim::components::Name>(
      [&](const sim::Entity &_entity, const sim::components::Imu *, const sim::components::Name *_name) -> bool
      {
        auto imuData = std::make_shared<ImuData>();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Loading sensor: " << _name->Data());

        auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(_entity);
        if (sensorTopicComp)
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] Topic name: " << sensorTopicComp->Data());
        }

        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
        imuData->name = _name->Data();
        imuData->sim_imu_sensors_ = _entity;

        hardware_interface::ComponentInfo component;
        for (auto &comp : sensor_components_)
        {
          if (comp.name == _name->Data())
          {
            component = comp;
          }
        }

        static const std::map<std::string, size_t> interface_name_map = {
          { "orientation.x", 0 },         { "orientation.y", 1 },         { "orientation.z", 2 },
          { "orientation.w", 3 },         { "angular_velocity.x", 4 },    { "angular_velocity.y", 5 },
          { "angular_velocity.z", 6 },    { "linear_acceleration.x", 7 }, { "linear_acceleration.y", 8 },
          { "linear_acceleration.z", 9 },
        };

        for (const auto &state_interface : component.state_interfaces)
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

          size_t data_index = interface_name_map.at(state_interface.name);
          this->dataPtr->state_interfaces_.emplace_back(imuData->name, state_interface.name,
                                                        &imuData->imu_sensor_data_[data_index]);
        }
        this->dataPtr->imus_.push_back(imuData);
        return true;
      });

  // Register Magnetometer
  this->dataPtr->ecm->Each<sim::components::Magnetometer, sim::components::Name>(
      [&](const sim::Entity &ent, const sim::components::Magnetometer *, const sim::components::Name *name)
      {
        auto magData = std::make_shared<MagData>();
        magData->name = name->Data();
        magData->sim_mag_sensor_ = ent;
        // Register magnetic field state interfaces
        this->dataPtr->state_interfaces_.emplace_back(magData->name, "field_tesla.x", &magData->mag_sensor_data_[0]);
        this->dataPtr->state_interfaces_.emplace_back(magData->name, "field_tesla.y", &magData->mag_sensor_data_[1]);
        this->dataPtr->state_interfaces_.emplace_back(magData->name, "field_tesla.z", &magData->mag_sensor_data_[2]);
        this->dataPtr->mags_.push_back(magData);
        return true;
      });
}

void AerialRobotHwSim::querySimPrams()
{
  std::vector<std::string> param_names;
  param_names.push_back("ground_truth_pub_rate");
  param_names.push_back("ground_truth_pos_noise");
  param_names.push_back("ground_truth_vel_noise");
  param_names.push_back("ground_truth_rot_noise");
  param_names.push_back("ground_truth_angular_noise");
  param_names.push_back("ground_truth_rot_drift");
  param_names.push_back("ground_truth_vel_drift");
  param_names.push_back("ground_truth_angular_drift");
  param_names.push_back("ground_truth_rot_drift_frequency");
  param_names.push_back("ground_truth_vel_drift_frequency");
  param_names.push_back("ground_truth_angular_drift_frequency");
  param_names.push_back("mocap_pub_rate");
  param_names.push_back("mocap_pos_noise");
  param_names.push_back("mocap_rot_noise");

  std::vector<rclcpp::Parameter> params = this->dataPtr->client_ptr_->get_parameters(param_names);
  this->dataPtr->ground_truth_pub_rate_ = params[0].get_value<double>();
  this->dataPtr->ground_truth_pos_noise_ = params[1].get_value<double>();
  this->dataPtr->ground_truth_vel_noise_ = params[2].get_value<double>();
  this->dataPtr->ground_truth_rot_noise_ = params[3].get_value<double>();
  this->dataPtr->ground_truth_angular_noise_ = params[4].get_value<double>();
  this->dataPtr->ground_truth_rot_drift_ = params[5].get_value<double>();
  this->dataPtr->ground_truth_vel_drift_ = params[6].get_value<double>();
  this->dataPtr->ground_truth_angular_drift_ = params[7].get_value<double>();
  this->dataPtr->ground_truth_rot_drift_frequency_ = params[8].get_value<double>();
  this->dataPtr->ground_truth_vel_drift_frequency_ = params[9].get_value<double>();
  this->dataPtr->ground_truth_angular_drift_frequency_ = params[10].get_value<double>();
  this->dataPtr->mocap_pub_rate_ = params[11].get_value<double>();
  this->dataPtr->mocap_pos_noise_ = params[12].get_value<double>();
  this->dataPtr->mocap_rot_noise_ = params[13].get_value<double>();
}

CallbackReturn AerialRobotHwSim::on_init(const hardware_interface::HardwareInfo &system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  if (system_info.hardware_class_type.compare("gz_ros2_control/AerialRobotHwSim") != 0)
  {
    RCLCPP_WARN(this->nh_->get_logger(),
                "The ign_ros2_control plugin got renamed to gz_ros2_control.\n"
                "Update the <ros2_control> tag and gazebo plugin to\n"
                "<hardware>\n"
                "  <plugin>gz_ros2_control/AerialRobotHwSim</plugin>\n"
                "</hardware>\n"
                "<gazebo>\n"
                "  <plugin filename=\"gz_ros2_control-system\""
                "name=\"gz_ros2_control::GazeboSimROS2ControlPlugin\">\n"
                "    ...\n"
                "  </plugin>\n"
                "</gazebo>");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AerialRobotHwSim::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(this->nh_->get_logger(), "[sim] System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AerialRobotHwSim::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> AerialRobotHwSim::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn AerialRobotHwSim::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  // return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn AerialRobotHwSim::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  // return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type AerialRobotHwSim::read(const rclcpp::Time &sim_time, const rclcpp::Duration &period)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i)
  {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity)
    {
      continue;
    }

    // Get the joint velocity
    const auto *jointVelocity = this->dataPtr->ecm->Component<sim::components::JointVelocity>(
        this->dataPtr->joints_[i].sim_joint);

    // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
    // Get the joint force
    // const auto * jointForce =
    //   _ecm.Component<sim::components::JointForce>(
    //   this->dataPtr->sim_joints_[j]);

    // Get the joint position
    const auto *jointPositions = this->dataPtr->ecm->Component<sim::components::JointPosition>(
        this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    // This->dataPtr->joint_effort_[j] = jointForce->Data()[0];
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i)
  {
    if (this->dataPtr->imus_[i]->topicName.empty())
    {
      auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(
          this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp)
      {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] IMU " << this->dataPtr->imus_[i]->name
                                                                 << " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
                                      this->dataPtr->imus_[i].get());
      }
    }
  }

  for (unsigned int i = 0; i < this->dataPtr->mags_.size(); ++i)
  {
    if (this->dataPtr->mags_[i]->topicName.empty())
    {
      auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(
          this->dataPtr->mags_[i]->sim_mag_sensor_);
      if (sensorTopicComp)
      {
        this->dataPtr->mags_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "[sim] MAG " << this->dataPtr->mags_[i]->name
                                                                 << " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(this->dataPtr->mags_[i]->topicName, &MagData::OnMag,
                                      this->dataPtr->mags_[i].get());
      }
    }
  }

  for (auto &imu : dataPtr->imus_)
  {
    auto imuEnt = imu->sim_imu_sensors_;
    if (imuEnt == sim::kNullEntity)
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "[sim] Cannot find FC link");
      continue;
    }
    auto poseComp = dataPtr->ecm->Component<sim::components::WorldPose>(imuEnt);
    auto angVelComp = dataPtr->ecm->Component<sim::components::AngularVelocity>(imuEnt);
    if (poseComp && angVelComp)
    {
      const auto &pose = poseComp->Data();
      const auto &ang_vel = angVelComp->Data();
      const auto &lin_vel = (pose.Pos() - this->dataPtr->last_baselink_pos_) /
                            (sim_time.seconds() - this->dataPtr->last_pos_read_time_.seconds());  // We don't have a
                                                                                                  // linear velocity
                                                                                                  // component for imu
                                                                                                  // link
      this->dataPtr->last_baselink_pos_ = pose.Pos();
      this->dataPtr->last_pos_read_time_ = sim_time;
      // Ground truth
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = sim_time;
      odom_msg.pose.pose.position.x = pose.Pos().X() +
                                      aerial_robot_simulation::gaussianKernel(this->dataPtr->ground_truth_pos_noise_);
      odom_msg.pose.pose.position.y = pose.Pos().Y() +
                                      aerial_robot_simulation::gaussianKernel(this->dataPtr->ground_truth_pos_noise_);
      odom_msg.pose.pose.position.z = pose.Pos().Z() +
                                      aerial_robot_simulation::gaussianKernel(this->dataPtr->ground_truth_pos_noise_);

      ignition::math::Vector3d delta_euler(
          aerial_robot_simulation::addNoise(this->dataPtr->ground_truth_rot_curr_drift_.X(),
                                            this->dataPtr->ground_truth_rot_drift_,
                                            this->dataPtr->ground_truth_rot_drift_frequency_, 0,
                                            this->dataPtr->ground_truth_rot_noise_, period.seconds()),
          aerial_robot_simulation::addNoise(this->dataPtr->ground_truth_rot_curr_drift_.Y(),
                                            this->dataPtr->ground_truth_rot_drift_,
                                            this->dataPtr->ground_truth_rot_drift_frequency_, 0,
                                            this->dataPtr->ground_truth_rot_noise_, period.seconds()),
          aerial_robot_simulation::addNoise(this->dataPtr->ground_truth_rot_curr_drift_.Z(),
                                            this->dataPtr->ground_truth_rot_drift_,
                                            this->dataPtr->ground_truth_rot_drift_frequency_, 0,
                                            this->dataPtr->ground_truth_rot_noise_, period.seconds()));
      ignition::math::Quaterniond q_noise = pose.Rot() * ignition::math::Quaterniond(delta_euler);
      odom_msg.pose.pose.orientation.x = q_noise.X();
      odom_msg.pose.pose.orientation.y = q_noise.Y();
      odom_msg.pose.pose.orientation.z = q_noise.Z();
      odom_msg.pose.pose.orientation.w = q_noise.W();

      odom_msg.twist.twist.linear.x = lin_vel.X() - +aerial_robot_simulation::addNoise(
                                                        this->dataPtr->ground_truth_vel_curr_drift_.X(),
                                                        this->dataPtr->ground_truth_vel_drift_,
                                                        this->dataPtr->ground_truth_vel_drift_frequency_, 0,
                                                        this->dataPtr->ground_truth_vel_noise_, period.seconds());
      odom_msg.twist.twist.linear.y = lin_vel.Y() + aerial_robot_simulation::addNoise(
                                                        this->dataPtr->ground_truth_vel_curr_drift_.Y(),
                                                        this->dataPtr->ground_truth_vel_drift_,
                                                        this->dataPtr->ground_truth_vel_drift_frequency_, 0,
                                                        this->dataPtr->ground_truth_vel_noise_, period.seconds());
      odom_msg.twist.twist.linear.z = lin_vel.Z() + aerial_robot_simulation::addNoise(
                                                        this->dataPtr->ground_truth_vel_curr_drift_.Z(),
                                                        this->dataPtr->ground_truth_vel_drift_,
                                                        this->dataPtr->ground_truth_vel_drift_frequency_, 0,
                                                        this->dataPtr->ground_truth_vel_noise_, period.seconds());

      odom_msg.twist.twist.angular.x = ang_vel.X() + aerial_robot_simulation::addNoise(
                                                         this->dataPtr->ground_truth_angular_curr_drift_.X(),
                                                         this->dataPtr->ground_truth_angular_drift_,
                                                         this->dataPtr->ground_truth_angular_drift_frequency_, 0,
                                                         this->dataPtr->ground_truth_angular_noise_, period.seconds());
      odom_msg.twist.twist.angular.y = ang_vel.Y() + aerial_robot_simulation::addNoise(
                                                         this->dataPtr->ground_truth_angular_curr_drift_.Y(),
                                                         this->dataPtr->ground_truth_angular_drift_,
                                                         this->dataPtr->ground_truth_angular_drift_frequency_, 0,
                                                         this->dataPtr->ground_truth_angular_noise_, period.seconds());
      odom_msg.twist.twist.angular.z = ang_vel.Z() + aerial_robot_simulation::addNoise(
                                                         this->dataPtr->ground_truth_angular_curr_drift_.Z(),
                                                         this->dataPtr->ground_truth_angular_drift_,
                                                         this->dataPtr->ground_truth_angular_drift_frequency_, 0,
                                                         this->dataPtr->ground_truth_angular_noise_, period.seconds());

      // TODO:Set ground truth value to spinal interface

      if (sim_time.seconds() - this->dataPtr->last_ground_truth_time_.seconds() > this->dataPtr->ground_truth_pub_rate_)
      {
        this->dataPtr->ground_truth_pub_->publish(odom_msg);
        this->dataPtr->last_ground_truth_time_ = sim_time;
      }

      if (sim_time.seconds() - this->dataPtr->last_mocap_time_.seconds() > this->dataPtr->mocap_pub_rate_)
      {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;

        pose_msg.pose.position.x = odom_msg.pose.pose.position.x +
                                   aerial_robot_simulation::gaussianKernel(this->dataPtr->mocap_pos_noise_);
        pose_msg.pose.position.y = odom_msg.pose.pose.position.y +
                                   aerial_robot_simulation::gaussianKernel(this->dataPtr->mocap_pos_noise_);
        pose_msg.pose.position.z = odom_msg.pose.pose.position.z +
                                   aerial_robot_simulation::gaussianKernel(this->dataPtr->mocap_pos_noise_);

        delta_euler = ignition::math::Vector3d(
            aerial_robot_simulation::gaussianKernel(this->dataPtr->mocap_rot_noise_),
            aerial_robot_simulation::gaussianKernel(this->dataPtr->mocap_rot_noise_),
            aerial_robot_simulation::gaussianKernel(this->dataPtr->mocap_rot_noise_));
        q_noise = pose.Rot() * ignition::math::Quaterniond(delta_euler);
        pose_msg.pose.orientation.x = q_noise.X();
        pose_msg.pose.orientation.y = q_noise.Y();
        pose_msg.pose.orientation.z = q_noise.Z();
        pose_msg.pose.orientation.w = q_noise.W();

        this->dataPtr->last_mocap_time_ = sim_time;
        this->dataPtr->mocap_pub_->publish(pose_msg);
      }
    }
    else
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "[sim] FC link doesn't have pose info");
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AerialRobotHwSim::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces)
{
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++)
  {
    for (const std::string &interface_name : stop_interfaces)
    {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name == (this->dataPtr->joints_[j].name + "/" + hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method &= static_cast<ControlMethod_>(VELOCITY & EFFORT);
      }
      else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                  hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method &= static_cast<ControlMethod_>(POSITION & EFFORT);
      }
      else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                  hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method &= static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string &interface_name : start_interfaces)
    {
      if (interface_name == (this->dataPtr->joints_[j].name + "/" + hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      }
      else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                  hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      }
      else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                  hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AerialRobotHwSim::write(const rclcpp::Time &sim_time, const rclcpp::Duration &period)
{
  std::map<sim::Entity, std::pair<ignition::math::Vector3d, ignition::math::Vector3d>> rotor_parent_wrenches;

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i)
  {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity)
    {
      continue;
    }

    if (this->dataPtr->joints_[i].joint_control_method & VELOCITY)
    {
      if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[i].sim_joint,
                                            sim::components::JointVelocityCmd({ 0 }));
      }
      else
      {
        const auto jointVelCmd = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[i].sim_joint);
        *jointVelCmd = sim::components::JointVelocityCmd({ this->dataPtr->joints_[i].joint_velocity_cmd });
      }
    }
    else if (this->dataPtr->joints_[i].joint_control_method & POSITION)
    {
      // Get error in position
      double error;
      error = (this->dataPtr->joints_[i].joint_position - this->dataPtr->joints_[i].joint_position_cmd) *
              *this->dataPtr->update_rate;

      // Calculate target velcity
      double target_vel = -this->dataPtr->position_proportional_gain_ * error;

      auto vel = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr)
      {
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[i].sim_joint,
                                            sim::components::JointVelocityCmd({ target_vel }));
      }
      else if (!vel->Data().empty())
      {
        vel->Data()[0] = target_vel;
      }
    }
    else if (this->dataPtr->joints_[i].joint_control_method & EFFORT)
    {
      if (this->dataPtr->joints_[i].is_rotor)
      {
        const double thrust = std::max(0.0, this->dataPtr->joints_[i].joint_effort_cmd);
        this->dataPtr->joints_[i].joint_effort = thrust;

        const double visual_speed = thrust * this->dataPtr->joints_[i].rotor_visual_speed_rate;
        this->dataPtr->joints_[i].joint_velocity_cmd = visual_speed;
        auto vel = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[i].sim_joint);
        if (vel == nullptr)
        {
          this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[i].sim_joint,
                                              sim::components::JointVelocityCmd({ visual_speed }));
        }
        else if (!vel->Data().empty())
        {
          vel->Data()[0] = visual_speed;
        }

        if (this->dataPtr->joints_[i].sim_child_link != sim::kNullEntity)
        {
          sim::Link rotor_link(this->dataPtr->joints_[i].sim_child_link);
          const auto rotor_pose = sim::worldPose(this->dataPtr->joints_[i].sim_child_link, *this->dataPtr->ecm);
          const auto thrust_axis = rotor_pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
          const auto yaw_drag_torque = thrust_axis * (thrust * this->dataPtr->joints_[i].rotor_direction *
                                                      this->dataPtr->joints_[i].m_f_rate);

          if (this->dataPtr->joints_[i].sim_parent_link != sim::kNullEntity)
          {
            const auto parent_pose = sim::worldPose(this->dataPtr->joints_[i].sim_parent_link, *this->dataPtr->ecm);
            const auto thrust_force = thrust_axis * thrust;
            const auto thrust_arm = rotor_pose.Pos() - parent_pose.Pos();
            auto &wrench = rotor_parent_wrenches[this->dataPtr->joints_[i].sim_parent_link];
            wrench.first += thrust_force;
            wrench.second += thrust_arm.Cross(thrust_force) + yaw_drag_torque;
          }
          else
          {
            rotor_link.AddWorldWrench(*this->dataPtr->ecm, thrust_axis * thrust, yaw_drag_torque);
          }
        }

        continue;
      }

      if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[i].sim_joint, sim::components::JointForceCmd({ 0 }));
      }
      else
      {
        const auto jointEffortCmd = this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
            this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = sim::components::JointForceCmd({ this->dataPtr->joints_[i].joint_effort_cmd });
      }
    }
    else if (this->dataPtr->joints_[i].is_actuated)
    {
      // Fallback case is a velocity command of zero (only for actuated joints)
      double target_vel = 0.0;
      auto vel = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr)
      {
        this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[i].sim_joint,
                                            sim::components::JointVelocityCmd({ target_vel }));
      }
      else if (!vel->Data().empty())
      {
        vel->Data()[0] = target_vel;
      }
    }
  }

  for (const auto &rotor_parent_wrench : rotor_parent_wrenches)
  {
    sim::Link parent_link(rotor_parent_wrench.first);
    parent_link.AddWorldWrench(*this->dataPtr->ecm, rotor_parent_wrench.second.first,
                               rotor_parent_wrench.second.second);
  }

  // Set values of all mimic joints with respect to mimicked joint
  for (const auto &mimic_joint : this->dataPtr->mimic_joints_)
  {
    for (const auto &mimic_interface : mimic_joint.interfaces_to_mimic)
    {
      if (mimic_interface == "position")
      {
        // Get the joint position
        double position_mimicked_joint = this->dataPtr->ecm
                                             ->Component<sim::components::JointPosition>(
                                                 this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)
                                             ->Data()[0];

        double position_mimic_joint = this->dataPtr->ecm
                                          ->Component<sim::components::JointPosition>(
                                              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)
                                          ->Data()[0];

        double position_error = position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

        double velocity_sp = (-1.0) * position_error * (*this->dataPtr->update_rate);

        auto vel = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

        if (vel == nullptr)
        {
          this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
                                              sim::components::JointVelocityCmd({ velocity_sp }));
        }
        else if (!vel->Data().empty())
        {
          vel->Data()[0] = velocity_sp;
        }
      }
      if (mimic_interface == "velocity")
      {
        // Get the velocity of mimicked joint
        double velocity_mimicked_joint = this->dataPtr->ecm
                                             ->Component<sim::components::JointVelocity>(
                                                 this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)
                                             ->Data()[0];

        if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
                this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
                                              sim::components::JointVelocityCmd({ 0 }));
        }
        else
        {
          const auto jointVelCmd = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
          *jointVelCmd = sim::components::JointVelocityCmd({ mimic_joint.multiplier * velocity_mimicked_joint });
        }
      }
      if (mimic_interface == "effort")
      {
        // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
        // Get the joint force
        // const auto * jointForce =
        //   _ecm.Component<sim::components::JointForce>(
        //   this->dataPtr->sim_joints_[j]);
        if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
                this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
                                              sim::components::JointForceCmd({ 0 }));
        }
        else
        {
          const auto jointEffortCmd = this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
          *jointEffortCmd = sim::components::JointForceCmd(
              { mimic_joint.multiplier * this->dataPtr->joints_[mimic_joint.mimicked_joint_index].joint_effort });
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}
}

PLUGINLIB_EXPORT_CLASS(gz_ros2_control::AerialRobotHwSim, gz_ros2_control::GazeboSimSystemInterface)
