// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  …
 *********************************************************************/

#pragma once

#include <aerial_robot_model/utils/kdl_utils.h>
#include <aerial_robot_model/utils/math_utils.h>
#include <tinyxml2.h>
#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <vector>

namespace aerial_robot_model {

// Basic Aerial Robot Model
class RobotModel {
 public:
  RobotModel();
  virtual ~RobotModel() = default;

  virtual void initialize(rclcpp::Node::SharedPtr node, bool init_with_rosparam = true, bool verbose = false,
                          bool fixed_model = true, double fc_f_min_thre = 0.0, double fc_t_min_thre = 0.0,
                          double epsilon = 10.0);

  void updateRobotModel();
  void updateRobotModel(const KDL::JntArray &joint_positions);
  void updateRobotModel(const sensor_msgs::msg::JointState &state);

  // kinematics
  bool initialized() const { return initialized_; }
  bool isModelFixed() const { return fixed_model_; }
  std::string getBaselinkName() const { return baselink_; }
  const std::map<std::string, KDL::RigidBodyInertia> &getInertiaMap() const { return inertia_map_; }
  double getMass() const { return mass_; }
  int getRotorNum() const { return rotor_num_; }
  const std::map<int, int> &getRotorDirection() const { return rotor_direction_; }
  std::string getRootFrameName() const { return GetTreeElementSegment(tree_.getRootSegment()->second).getName(); }
  int getJointNum() const { return joint_num_; }
  const KDL::JntArray &getJointPositions() const { return joint_positions_; }
  const std::map<std::string, uint32_t> &getJointIndexMap() const { return joint_index_map_; }
  const std::map<std::string, std::vector<std::string>> &getJointSegmentMap() const { return joint_segment_map_; }
  const std::map<std::string, int> &getJointHierarchy() const { return joint_hierarchy_; }
  const std::vector<std::string> &getJointNames() const { return joint_names_; }
  const std::vector<int> &getJointIndices() const { return joint_indices_; }
  const std::vector<std::string> &getJointParentLinkNames() const { return joint_parent_link_names_; }

  std::map<std::string, KDL::Frame> getSegmentsTf() {
    std::lock_guard<std::mutex> lock(mutex_seg_tf_);
    return seg_tf_map_;
  }
  KDL::Frame getSegmentTf(const std::string &seg_name) {
    std::lock_guard<std::mutex> lock(mutex_seg_tf_);
    return seg_tf_map_.at(seg_name);
  }

  template <typename T>
  T forwardKinematics(const std::string &link, const KDL::JntArray &joint_positions) const;
  template <typename T>
  T forwardKinematics(const std::string &link, const sensor_msgs::msg::JointState &state) const;

  std::map<std::string, KDL::Frame> fullForwardKinematics(const KDL::JntArray &js) {
    return fullForwardKinematicsImpl(js);
  }
  std::map<std::string, KDL::Frame> fullForwardKinematics(const sensor_msgs::msg::JointState &st) {
    return fullForwardKinematicsImpl(jointMsgToKdl(st));
  }

  const KDL::Tree &getTree() const { return tree_; }
  const urdf::Model &getUrdfModel() const { return model_; }
  double getVerbose() const { return verbose_; }

  template <typename T>
  T getCog() const;
  template <typename T>
  T getCogDesireOrientation() const;
  template <typename T>
  T getCog2Baselink() const;
  template <typename T>
  T getInertia() const;
  template <typename T>
  std::vector<T> getRotorsNormalFromCog() const;
  template <typename T>
  std::vector<T> getRotorsOriginFromCog() const;

  /// load robot model XML from parameter server
  static std::unique_ptr<tinyxml2::XMLDocument> getRobotModelXml(const std::string &param,
                                                                 rclcpp::Node::SharedPtr node);

  KDL::JntArray jointMsgToKdl(const sensor_msgs::msg::JointState &state) const;
  sensor_msgs::msg::JointState kdlJointToMsg(const KDL::JntArray &joint_positions) const;

  void setBaselinkName(const std::string &baselink) { baselink_ = baselink; }
  void setCogDesireOrientation(double roll, double pitch, double yaw) {
    setCogDesireOrientation(KDL::Rotation::RPY(roll, pitch, yaw));
  }
  void setCogDesireOrientation(const KDL::Rotation &cog_desire_orientation) {
    std::lock_guard<std::mutex> lock(mutex_desired_baselink_rot_);
    cog_desire_orientation_ = cog_desire_orientation;
  }

  bool addExtraModule(const std::string &module_name, const std::string &parent_link_name, const KDL::Frame &transform,
                      const KDL::RigidBodyInertia &inertia);
  bool removeExtraModule(const std::string &module_name);
  const std::map<std::string, KDL::Segment> &getExtraModuleMap() const { return extra_module_map_; }
  void setExtraModuleMap(const std::map<std::string, KDL::Segment> &map) { extra_module_map_ = map; }

  // statics (static thrust, joint torque)
  Eigen::VectorXd calcGravityWrenchOnRoot();
  virtual void calcStaticThrust();
  Eigen::MatrixXd calcWrenchMatrixOnCoG();
  virtual void calcWrenchMatrixOnRoot();

  const Eigen::VectorXd &getGravity() const { return gravity_; }
  const Eigen::VectorXd &getGravity3d() const { return gravity_3d_; }
  double getMFRate() const { return m_f_rate_; }
  const Eigen::VectorXd &getStaticThrust() const { return static_thrust_; }
  const std::vector<Eigen::MatrixXd> &getThrustWrenchAllocations() const { return thrust_wrench_allocations_; }
  const Eigen::MatrixXd &getThrustWrenchMatrix() const { return q_mat_; }
  const std::vector<Eigen::VectorXd> &getThrustWrenchUnits() const { return thrust_wrench_units_; }
  double getThrustUpperLimit() const { return thrust_max_; }
  double getThrustLowerLimit() const { return thrust_min_; }

  // control stability
  virtual void calcFeasibleControlFDists();
  virtual void calcFeasibleControlTDists();
  double calcTripleProduct(const Eigen::Vector3d &ui, const Eigen::Vector3d &uj, const Eigen::Vector3d &uk);
  std::vector<Eigen::Vector3d> calcV();
  double getEpsilon() const { return epsilon_; }
  const Eigen::VectorXd &getFeasibleControlFDists() const { return fc_f_dists_; }
  double getFeasibleControlFMin() const { return fc_f_min_; }
  double getFeasibleControlFMinThre() const { return fc_f_min_thre_; }
  const Eigen::VectorXd &getFeasibleControlTDists() const { return fc_t_dists_; }
  double getFeasibleControlTMin() const { return fc_t_min_; }
  double getFeasibleControlTMinThre() const { return fc_t_min_thre_; }

  void setFeasibleControlFMinThre(double v) { fc_f_min_thre_ = v; }
  void setFeasibleControlTMinThre(double v) { fc_t_min_thre_ = v; }

  virtual bool stabilityCheck(bool verbose = true);

  KDL::JntArray convertEigenToKDL(const Eigen::VectorXd &joint_vector);

 private:
  // --- 以下メンバ変数は変更なし ---
  bool initialized_{false};
  bool fixed_model_{true};
  rclcpp::Node::SharedPtr node_;
  double mass_{0.0};
  urdf::Model model_;
  std::string baselink_;
  KDL::Frame cog_;
  KDL::Rotation cog_desire_orientation_;
  KDL::Frame cog2baselink_transform_;
  std::vector<std::string> joint_names_;
  std::vector<int> joint_indices_;
  std::vector<std::string> joint_parent_link_names_;
  KDL::JntArray joint_positions_;
  KDL::RotationalInertia link_inertia_cog_;
  std::map<std::string, KDL::Segment> extra_module_map_;
  std::map<std::string, KDL::RigidBodyInertia> inertia_map_;
  std::map<std::string, uint32_t> joint_index_map_;
  std::map<std::string, std::vector<std::string>> joint_segment_map_;
  std::map<std::string, int> joint_hierarchy_;
  std::map<std::string, KDL::Frame> seg_tf_map_;
  int joint_num_{0};
  int rotor_num_{0};
  std::vector<KDL::Vector> rotors_origin_from_cog_;
  std::vector<KDL::Vector> rotors_normal_from_cog_;
  KDL::Tree tree_;
  std::string thrust_link_;
  bool verbose_{false};

  Eigen::VectorXd gravity_;
  Eigen::VectorXd gravity_3d_;
  double m_f_rate_{0.0};
  Eigen::MatrixXd q_mat_;
  std::map<int, int> rotor_direction_;
  Eigen::VectorXd static_thrust_;
  double thrust_max_{0.0};
  double thrust_min_{0.0};
  std::vector<Eigen::VectorXd> thrust_wrench_units_;
  std::vector<Eigen::MatrixXd> thrust_wrench_allocations_;

  double epsilon_{10.0};
  Eigen::VectorXd fc_f_dists_;
  Eigen::VectorXd fc_t_dists_;
  double fc_f_min_{0.0};
  double fc_t_min_{0.0};
  double fc_f_min_thre_{0.0};
  double fc_t_min_thre_{0.0};

  mutable std::mutex mutex_cog_;
  mutable std::mutex mutex_cog2baselink_;
  mutable std::mutex mutex_inertia_;
  mutable std::mutex mutex_rotor_origin_;
  mutable std::mutex mutex_rotor_normal_;
  mutable std::mutex mutex_seg_tf_;
  mutable std::mutex mutex_desired_baselink_rot_;

  void getParamFromRos();  // implement with rclcpp::Node
  void kinematicsInit();
  void stabilityInit();
  void staticsInit();

  KDL::RigidBodyInertia inertialSetup(const KDL::TreeElement &);
  void jointSegmentSetupRecursive(const KDL::TreeElement &, std::vector<std::string>);
  void makeJointSegmentMap();

  KDL::Frame forwardKinematicsImpl(const std::string &link, const KDL::JntArray &) const;
  std::map<std::string, KDL::Frame> fullForwardKinematicsImpl(const KDL::JntArray &);

 protected:
  virtual void updateRobotModelImpl(const KDL::JntArray &);

  void setCog(const KDL::Frame &f) {
    std::lock_guard<std::mutex> lock(mutex_cog_);
    cog_ = f;
  }
  void setCog2Baselink(const KDL::Frame &f) {
    std::lock_guard<std::mutex> lock(mutex_cog2baselink_);
    cog2baselink_transform_ = f;
  }
  void setInertia(const KDL::RotationalInertia &i) {
    std::lock_guard<std::mutex> lock(mutex_inertia_);
    link_inertia_cog_ = i;
  }
  void setRotorsNormalFromCog(const std::vector<KDL::Vector> &v) {
    std::lock_guard<std::mutex> lock(mutex_rotor_normal_);
    rotors_normal_from_cog_ = v;
  }
  void setRotorsOriginFromCog(const std::vector<KDL::Vector> &v) {
    std::lock_guard<std::mutex> lock(mutex_rotor_origin_);
    rotors_origin_from_cog_ = v;
  }
  void setSegmentsTf(const std::map<std::string, KDL::Frame> &m) {
    std::lock_guard<std::mutex> lock(mutex_seg_tf_);
    seg_tf_map_ = m;
  }
  void setStaticThrust(const Eigen::VectorXd &v) { static_thrust_ = v; }
  void setThrustWrenchMatrix(const Eigen::MatrixXd &m) { q_mat_ = m; }
};

// --- template implementations ---

// forwardKinematics for KDL::JntArray
template <>
inline Eigen::Affine3d RobotModel::forwardKinematics<Eigen::Affine3d>(const std::string &link,
                                                                      const KDL::JntArray &joint_positions) const {
  return aerial_robot_model::kdlToEigen(forwardKinematicsImpl(link, joint_positions));
}

template <>
inline geometry_msgs::msg::TransformStamped RobotModel::forwardKinematics<geometry_msgs::msg::TransformStamped>(
    const std::string &link, const KDL::JntArray &joint_positions) const {
  return aerial_robot_model::kdlToMsg(forwardKinematicsImpl(link, joint_positions));
}

template <>
inline KDL::Frame RobotModel::forwardKinematics<KDL::Frame>(const std::string &link,
                                                            const KDL::JntArray &joint_positions) const {
  return forwardKinematicsImpl(link, joint_positions);
}

template <>
inline tf2::Transform RobotModel::forwardKinematics<tf2::Transform>(const std::string &link,
                                                                    const KDL::JntArray &joint_positions) const {
  return aerial_robot_model::kdlToTf2(forwardKinematicsImpl(link, joint_positions));
}

// forwardKinematics for sensor_msgs::msg::JointState
template <>
inline Eigen::Affine3d RobotModel::forwardKinematics<Eigen::Affine3d>(const std::string &link,
                                                                      const sensor_msgs::msg::JointState &state) const {
  return aerial_robot_model::kdlToEigen(forwardKinematicsImpl(link, jointMsgToKdl(state)));
}

template <>
inline geometry_msgs::msg::TransformStamped RobotModel::forwardKinematics<geometry_msgs::msg::TransformStamped>(
    const std::string &link, const sensor_msgs::msg::JointState &state) const {
  return aerial_robot_model::kdlToMsg(forwardKinematicsImpl(link, jointMsgToKdl(state)));
}

template <>
inline KDL::Frame RobotModel::forwardKinematics<KDL::Frame>(const std::string &link,
                                                            const sensor_msgs::msg::JointState &state) const {
  return forwardKinematicsImpl(link, jointMsgToKdl(state));
}

template <>
inline tf2::Transform RobotModel::forwardKinematics<tf2::Transform>(const std::string &link,
                                                                    const sensor_msgs::msg::JointState &state) const {
  return aerial_robot_model::kdlToTf2(forwardKinematicsImpl(link, jointMsgToKdl(state)));
}

// getCog()
template <>
inline KDL::Frame RobotModel::getCog<KDL::Frame>() const {
  std::lock_guard<std::mutex> lock(mutex_cog_);
  return cog_;
}

template <>
inline Eigen::Affine3d RobotModel::getCog<Eigen::Affine3d>() const {
  return aerial_robot_model::kdlToEigen(getCog<KDL::Frame>());
}

template <>
inline geometry_msgs::msg::TransformStamped RobotModel::getCog<geometry_msgs::msg::TransformStamped>() const {
  return aerial_robot_model::kdlToMsg(getCog<KDL::Frame>());
}

template <>
inline tf2::Transform RobotModel::getCog<tf2::Transform>() const {
  return aerial_robot_model::kdlToTf2(getCog<KDL::Frame>());
}

// getCog2Baselink()
template <>
inline KDL::Frame RobotModel::getCog2Baselink<KDL::Frame>() const {
  std::lock_guard<std::mutex> lock(mutex_cog2baselink_);
  return cog2baselink_transform_;
}

template <>
inline Eigen::Affine3d RobotModel::getCog2Baselink<Eigen::Affine3d>() const {
  return aerial_robot_model::kdlToEigen(getCog2Baselink<KDL::Frame>());
}

template <>
inline geometry_msgs::msg::TransformStamped RobotModel::getCog2Baselink<geometry_msgs::msg::TransformStamped>() const {
  return aerial_robot_model::kdlToMsg(getCog2Baselink<KDL::Frame>());
}

template <>
inline tf2::Transform RobotModel::getCog2Baselink<tf2::Transform>() const {
  return aerial_robot_model::kdlToTf2(getCog2Baselink<KDL::Frame>());
}

// getCogDesireOrientation()
template <>
inline KDL::Rotation RobotModel::getCogDesireOrientation<KDL::Rotation>() const {
  std::lock_guard<std::mutex> lock(mutex_desired_baselink_rot_);
  return cog_desire_orientation_;
}

template <>
inline Eigen::Matrix3d RobotModel::getCogDesireOrientation<Eigen::Matrix3d>() const {
  return aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>());
}

// getInertia()
template <>
inline KDL::RotationalInertia RobotModel::getInertia<KDL::RotationalInertia>() const {
  std::lock_guard<std::mutex> lock(mutex_inertia_);
  return link_inertia_cog_;
}

template <>
inline Eigen::Matrix3d RobotModel::getInertia<Eigen::Matrix3d>() const {
  return aerial_robot_model::kdlToEigen(getInertia<KDL::RotationalInertia>());
}

// getRotorsNormalFromCog()
template <>
inline std::vector<KDL::Vector> RobotModel::getRotorsNormalFromCog<KDL::Vector>() const {
  std::lock_guard<std::mutex> lock(mutex_rotor_normal_);
  return rotors_normal_from_cog_;
}

template <>
inline std::vector<Eigen::Vector3d> RobotModel::getRotorsNormalFromCog<Eigen::Vector3d>() const {
  return aerial_robot_model::kdlToEigen(getRotorsNormalFromCog<KDL::Vector>());
}

template <>
inline std::vector<geometry_msgs::msg::PointStamped>
RobotModel::getRotorsNormalFromCog<geometry_msgs::msg::PointStamped>() const {
  return aerial_robot_model::kdlToMsg(getRotorsNormalFromCog<KDL::Vector>());
}

template <>
inline std::vector<tf2::Vector3> RobotModel::getRotorsNormalFromCog<tf2::Vector3>() const {
  return aerial_robot_model::kdlToTf2(getRotorsNormalFromCog<KDL::Vector>());
}

// getRotorsOriginFromCog()
template <>
inline std::vector<KDL::Vector> RobotModel::getRotorsOriginFromCog<KDL::Vector>() const {
  std::lock_guard<std::mutex> lock(mutex_rotor_origin_);
  return rotors_origin_from_cog_;
}

template <>
inline std::vector<Eigen::Vector3d> RobotModel::getRotorsOriginFromCog<Eigen::Vector3d>() const {
  return aerial_robot_model::kdlToEigen(getRotorsOriginFromCog<KDL::Vector>());
}

template <>
inline std::vector<geometry_msgs::msg::PointStamped>
RobotModel::getRotorsOriginFromCog<geometry_msgs::msg::PointStamped>() const {
  return aerial_robot_model::kdlToMsg(getRotorsOriginFromCog<KDL::Vector>());
}

template <>
inline std::vector<tf2::Vector3> RobotModel::getRotorsOriginFromCog<tf2::Vector3>() const {
  return aerial_robot_model::kdlToTf2(getRotorsOriginFromCog<KDL::Vector>());
}

}  // namespace aerial_robot_model
