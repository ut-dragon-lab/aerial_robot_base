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

#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>

#include "aerial_robot_model/model/aerial_robot_model.h"

namespace aerial_robot_model {
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotModel");

RobotModel::RobotModel()
    : baselink_("fc"),
      thrust_link_("thrust"),
      rotor_num_(0),
      joint_num_(0),
      thrust_max_(0),
      thrust_min_(0),
      initialized_(false),
      mass_(0.0) {}

void RobotModel::initialize(rclcpp::Node::SharedPtr node, bool init_with_rosparam, bool verbose, bool fixed_model,
                            double fc_f_min_thre, double fc_t_min_thre, double epsilon) {
  node_ = node;
  verbose_ = verbose;
  fixed_model_ = fixed_model;
  fc_f_min_thre_ = fc_f_min_thre;
  fc_t_min_thre_ = fc_t_min_thre;
  epsilon_ = epsilon;
  if (init_with_rosparam) {
    getParamFromRos();
  }

  gravity_.resize(6);
  gravity_ << 0, 0, 9.80665, 0, 0, 0;
  gravity_3d_.resize(3);
  gravity_3d_ << 0, 0, 9.80665;

  kinematicsInit();
  stabilityInit();
  staticsInit();

  if (fixed_model_) {
    updateRobotModel();
  }
}

void RobotModel::getParamFromRos() {
  // declare & get parameters
  node_->declare_parameter("kinematic_verbose", verbose_);
  node_->declare_parameter("fc_f_min_thre", fc_f_min_thre_);
  node_->declare_parameter("fc_t_min_thre", fc_t_min_thre_);
  node_->declare_parameter("epsilon", epsilon_);

  node_->get_parameter("kinematic_verbose", verbose_);
  node_->get_parameter("fc_f_min_thre", fc_f_min_thre_);
  node_->get_parameter("fc_t_min_thre", fc_t_min_thre_);
  node_->get_parameter("epsilon", epsilon_);
}

void RobotModel::kinematicsInit() {
  // URDF
  std::string urdf_xml;
  if (!node_->get_parameter("robot_description", urdf_xml)) {
    RCLCPP_ERROR(LOGGER, "parameter 'robot_description' not set");
    return;
  }
  if (!model_.initString(urdf_xml)) {
    RCLCPP_ERROR(LOGGER, "Failed to parse URDF from parameter");
    return;
  }
  if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
    RCLCPP_ERROR(LOGGER, "Failed to extract KDL tree from URDF");
    return;
  }

  // baselink, thrust_link(tinyxml2)
  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(urdf_xml.c_str());
  auto* robot_el = xml_doc.FirstChildElement("robot");
  if (robot_el) {
    if (auto* bl = robot_el->FirstChildElement("baselink")) {
      baselink_ = bl->Attribute("name");
    } else {
      RCLCPP_DEBUG(LOGGER, "no <baselink> in URDF");
    }
    if (auto* tl = robot_el->FirstChildElement("thrust_link")) {
      thrust_link_ = tl->Attribute("name");
    } else {
      RCLCPP_DEBUG(LOGGER, "no <thrust_link> in URDF");
    }
  }

  // inertial setup
  inertialSetup(tree_.getRootSegment()->second);
  makeJointSegmentMap();

  rotors_origin_from_cog_.resize(rotor_num_);
  rotors_normal_from_cog_.resize(rotor_num_);
}

void RobotModel::stabilityInit() {
  fc_f_dists_.resize(rotor_num_ * (rotor_num_ - 1));
  fc_t_dists_.resize(rotor_num_ * (rotor_num_ - 1));
}

void RobotModel::staticsInit() {
  // m_f_rate の取得
  std::string urdf_xml = node_->get_parameter("robot_description").as_string();
  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(urdf_xml.c_str());
  if (auto* mfr = xml_doc.FirstChildElement("robot")->FirstChildElement("m_f_rate")) {
    mfr->QueryDoubleAttribute("value", &m_f_rate_);
  } else {
    RCLCPP_ERROR(LOGGER, "no <m_f_rate> in URDF");
  }

  // rotor の thrust limits
  std::vector<urdf::LinkSharedPtr> links;
  model_.getLinks(links);
  for (auto& link : links) {
    if (link->parent_joint && link->parent_joint->name == "rotor1") {
      thrust_max_ = link->parent_joint->limits->upper;
      thrust_min_ = link->parent_joint->limits->lower;
      break;
    }
  }

  q_mat_.resize(6, rotor_num_);
  static_thrust_.resize(rotor_num_);
  thrust_wrench_units_.resize(rotor_num_);
  thrust_wrench_allocations_.resize(rotor_num_);
}

KDL::RigidBodyInertia RobotModel::inertialSetup(const KDL::TreeElement& tree_element) {
  const auto& seg = GetTreeElementSegment(tree_element);
  auto current_inertia = seg.getInertia();
  if (verbose_) {
    RCLCPP_WARN(LOGGER, "segment %s mass: %f", seg.getName().c_str(), current_inertia.getMass());
  }

  // root-link special handling
  if (seg.getName().find("root") != std::string::npos) {
    auto child = GetTreeElementChildren(tree_element).at(0)->second;
    auto& child_seg = GetTreeElementSegment(child);
    inertia_map_.emplace(child_seg.getName(), child_seg.getInertia());
    if (verbose_) {
      RCLCPP_WARN(LOGGER, "Add root link: %s", child_seg.getName().c_str());
    }
  }

  // joint-bearing segments
  if (seg.getJoint().getType() != KDL::Joint::None && seg.getJoint().getName().find("rotor") == std::string::npos) {
    inertia_map_.emplace(seg.getName(), current_inertia);
    joint_index_map_.emplace(seg.getJoint().getName(), tree_element.q_nr);
    joint_names_.push_back(seg.getJoint().getName());
    joint_indices_.push_back(tree_element.q_nr);
    joint_parent_link_names_.push_back(GetTreeElementParent(tree_element)->first);
    if (verbose_) {
      RCLCPP_WARN(LOGGER, "Add inertia-link: %s", seg.getName().c_str());
    }
  }

  // rotor detection
  if (seg.getJoint().getName().find("rotor") != std::string::npos) {
    auto uj = model_.getJoint(seg.getJoint().getName());
    if (uj->type == urdf::Joint::CONTINUOUS) {
      rotor_direction_.emplace(std::atoi(seg.getJoint().getName().substr(5).c_str()), uj->axis.z);
    }
  }

  // recurse children
  KDL::RigidBodyInertia combined = current_inertia;
  for (auto& child_elem : GetTreeElementChildren(tree_element)) {
    auto child_inertia = inertialSetup(child_elem->second);
    combined = combined + (seg.getFrameToTip() * child_inertia);
  }

  // count rotors under thrust_link_
  if (seg.getName().find(thrust_link_) != std::string::npos) {
    ++rotor_num_;
  }

  // update base inertia
  auto it = inertia_map_.find(seg.getName());
  if (it != inertia_map_.end()) {
    it->second = combined;
    if (verbose_) {
      RCLCPP_WARN(LOGGER, "Base %s total mass: %f", seg.getName().c_str(), it->second.getMass());
    }
    combined = KDL::RigidBodyInertia::Zero();
  }

  return combined;
}

void RobotModel::makeJointSegmentMap() {
  joint_segment_map_.clear();
  for (auto& p : joint_index_map_) {
    joint_segment_map_[p.first] = {};
  }
  jointSegmentSetupRecursive(tree_.getRootSegment()->second, {});
}

void RobotModel::jointSegmentSetupRecursive(const KDL::TreeElement& elem, std::vector<std::string> current_joints) {
  const auto& seg = GetTreeElementSegment(elem);
  bool added = false;

  if (seg.getJoint().getType() != KDL::Joint::None && seg.getJoint().getName().find("rotor") == std::string::npos) {
    current_joints.push_back(seg.getJoint().getName());
    joint_hierarchy_.emplace(seg.getJoint().getName(), current_joints.size() - 1);

    ++joint_num_;
    added = true;
  }

  if (inertia_map_.count(seg.getName()) || seg.getName().find(thrust_link_) != std::string::npos) {
    for (auto& j : current_joints) {
      joint_segment_map_[j].push_back(seg.getName());
    }
  }

  for (auto& child : GetTreeElementChildren(elem)) {
    jointSegmentSetupRecursive(child->second, current_joints);
  }
}
bool RobotModel::addExtraModule(const std::string& module_name, const std::string& parent_link_name,
                                const KDL::Frame& transform, const KDL::RigidBodyInertia& inertia) {
  if (extra_module_map_.find(module_name) == extra_module_map_.end()) {
    if (inertia_map_.find(parent_link_name) == inertia_map_.end()) {
      RCLCPP_WARN(LOGGER,
                  "[extra module]: fail to add new extra module %s, because its parent link (%s) does not exist",
                  module_name.c_str(), parent_link_name.c_str());
      return false;
    }

    if (!aerial_robot_model::isValidRotation(transform.M)) {
      RCLCPP_WARN(LOGGER, "[extra module]: fail to add new extra module %s, because its orientation is invalid",
                  module_name.c_str());
      return false;
    }

    if (inertia.getMass() <= 0) {
      RCLCPP_WARN(LOGGER, "[extra module]: fail to add new extra module %s, becuase its mass %f is invalid",
                  module_name.c_str(), inertia.getMass());
      return false;
    }

    KDL::Segment extra_module(parent_link_name, KDL::Joint(KDL::Joint::None), transform, inertia);
    extra_module_map_.insert(std::make_pair(module_name, extra_module));
    RCLCPP_INFO(LOGGER, "[extra module]: succeed to add new extra module %s", module_name.c_str());

    if (fixed_model_) {
      // update robot model instantly
      updateRobotModel();
    }

    return true;
  } else {
    RCLCPP_WARN(LOGGER, "[extra module]: fail to add new extra module %s, becuase it already exists",
                module_name.c_str());
    return false;
  }
}

bool RobotModel::removeExtraModule(const std::string& module_name) {
  const auto it = extra_module_map_.find(module_name);
  if (it == extra_module_map_.end()) {
    RCLCPP_WARN(LOGGER, "[extra module]: fail to remove the extra module %s, because it does not exists",
                module_name.c_str());
    return false;
  } else {
    extra_module_map_.erase(module_name);
    RCLCPP_INFO(LOGGER, "[extra module]: succeed to remove the extra module %s", module_name.c_str());

    if (fixed_model_) {
      // update robot model instantly
      updateRobotModel();
    }

    return true;
  }
}

void RobotModel::updateRobotModel() {
  KDL::JntArray dummy_joint_positions(tree_.getNrOfJoints());
  KDL::SetToZero(dummy_joint_positions);
  updateRobotModelImpl(dummy_joint_positions);
}

void RobotModel::updateRobotModel(const KDL::JntArray& joint_positions) { updateRobotModelImpl(joint_positions); }

void RobotModel::updateRobotModel(const sensor_msgs::msg::JointState& state) { updateRobotModel(jointMsgToKdl(state)); }

void RobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions) {
  joint_positions_ = joint_positions;

  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  const auto seg_tf_map = fullForwardKinematics(joint_positions);
  setSegmentsTf(seg_tf_map);

  for (const auto& inertia : inertia_map_) {
    KDL::Frame f = seg_tf_map.at(inertia.first);
    link_inertia = link_inertia + f * inertia.second;

    /* process for the extra module */
    for (const auto& extra : extra_module_map_) {
      if (extra.second.getName() == inertia.first) {
        link_inertia = link_inertia + f * (extra.second.getFrameToTip() * extra.second.getInertia());
      }
    }
  }

  /* CoG */
  KDL::Frame f_baselink = seg_tf_map.at(baselink_);
  KDL::Frame cog;
  cog.M = f_baselink.M * cog_desire_orientation_.Inverse();
  cog.p = link_inertia.getCOG();
  setCog(cog);
  mass_ = link_inertia.getMass();
  RCLCPP_INFO_STREAM_ONCE(LOGGER, "[aerial_robot_model] robot mass is " << mass_);

  setInertia((cog.Inverse() * link_inertia).getRotationalInertia());
  setCog2Baselink(cog.Inverse() * f_baselink);

  /* thrust point based on COG */
  std::vector<KDL::Vector> rotors_origin_from_cog, rotors_normal_from_cog;
  for (int i = 0; i < rotor_num_; ++i) {
    std::string rotor = thrust_link_ + std::to_string(i + 1);
    KDL::Frame f = seg_tf_map.at(rotor);
    if (verbose_) RCLCPP_WARN(LOGGER, " %s : [%f, %f, %f]", rotor.c_str(), f.p.x(), f.p.y(), f.p.z());
    rotors_origin_from_cog.push_back((cog.Inverse() * f).p);
    rotors_normal_from_cog.push_back((cog.Inverse() * f).M * KDL::Vector(0, 0, 1));
  }
  setRotorsNormalFromCog(rotors_normal_from_cog);
  setRotorsOriginFromCog(rotors_origin_from_cog);

  /* statics */
  calcStaticThrust();
  calcFeasibleControlFDists();
  calcFeasibleControlTDists();

  if (!initialized_) initialized_ = true;
}

KDL::Frame RobotModel::forwardKinematicsImpl(const std::string& link, const KDL::JntArray& joint_positions) const {
  if (joint_positions.rows() != tree_.getNrOfJoints()) throw std::runtime_error("joint num is invalid");

  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  KDL::Frame f;
  int status = fk_solver.JntToCart(joint_positions, f, link);
  if (status < 0) RCLCPP_ERROR(LOGGER, "can not solve FK to link: %s", link.c_str());

  return f;
}

std::map<std::string, KDL::Frame> RobotModel::fullForwardKinematicsImpl(const KDL::JntArray& joint_positions) {
  if (joint_positions.rows() != tree_.getNrOfJoints()) throw std::runtime_error("joint num is invalid");

  std::map<std::string, KDL::Frame> seg_tf_map;
  std::function<void(const KDL::TreeElement&, const KDL::Frame&)> recursiveFullFk =
      [&recursiveFullFk, &seg_tf_map, &joint_positions](const KDL::TreeElement& tree_element,
                                                        const KDL::Frame& parrent_f) {
        for (const auto& elem : GetTreeElementChildren(tree_element)) {
          const KDL::TreeElement& curr_element = elem->second;
          KDL::Frame curr_f =
              parrent_f * GetTreeElementSegment(curr_element).pose(joint_positions(GetTreeElementQNr(curr_element)));
          seg_tf_map.insert(std::make_pair(GetTreeElementSegment(curr_element).getName(), curr_f));
          recursiveFullFk(curr_element, curr_f);
        }
      };

  recursiveFullFk(tree_.getRootSegment()->second, KDL::Frame::Identity());

  return seg_tf_map;
}

Eigen::VectorXd RobotModel::calcGravityWrenchOnRoot() {
  const auto seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();

  Eigen::MatrixXd root_rot =
      aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());
  Eigen::VectorXd wrench_g = Eigen::VectorXd::Zero(6);
  for (const auto& inertia : inertia_map) {
    Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(3, 6);
    Eigen::Vector3d p =
        root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(inertia.first).p +
                                                  seg_frames.at(inertia.first).M * inertia.second.getCOG());
    jacobi_root.rightCols(3) = -aerial_robot_model::skew(p);
    wrench_g += jacobi_root.transpose() * inertia.second.getMass() * (-gravity_3d_);
  }
  return wrench_g;
}

Eigen::MatrixXd RobotModel::calcWrenchMatrixOnCoG() {
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  // Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = u.at(i);
    Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
  }
  return Q;
}

void RobotModel::calcWrenchMatrixOnRoot() {
  const auto seg_frames = getSegmentsTf();
  const std::vector<Eigen::Vector3d>& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  Eigen::MatrixXd root_rot =
      aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());

  q_mat_ = Eigen::MatrixXd::Zero(6, rotor_num);
  for (unsigned int i = 0; i < rotor_num; ++i) {
    std::string rotor = "thrust" + std::to_string(i + 1);
    Eigen::MatrixXd q_i = Eigen::MatrixXd::Identity(6, 6);
    Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(rotor).p);
    q_i.bottomLeftCorner(3, 3) = aerial_robot_model::skew(p);

    Eigen::VectorXd wrench_unit = Eigen::VectorXd::Zero(6);
    wrench_unit.head(3) = u.at(i);
    wrench_unit.tail(3) = m_f_rate * sigma.at(i + 1) * u.at(i);

    thrust_wrench_units_.at(i) = wrench_unit;
    thrust_wrench_allocations_.at(i) = q_i;
    q_mat_.col(i) = q_i * wrench_unit;
  }
}

void RobotModel::calcStaticThrust() {
  calcWrenchMatrixOnRoot();  // update Q matrix
  Eigen::VectorXd wrench_g = calcGravityWrenchOnRoot();
  static_thrust_ = aerial_robot_model::pseudoinverse(q_mat_) * (-wrench_g);
}

bool RobotModel::stabilityCheck(bool verbose) {
  if (fc_f_min_ < fc_f_min_thre_) {
    if (verbose)
      RCLCPP_ERROR_STREAM(LOGGER, "the min distance to the plane of feasible control force convex "
                                      << fc_f_min_ << " is lower than the threshold " << fc_f_min_thre_);
    return false;
  }

  if (fc_t_min_ < fc_t_min_thre_) {
    if (verbose)
      RCLCPP_ERROR_STREAM(LOGGER, "the min distance to the plane of feasible control torque convex "
                                      << fc_t_min_ << " is lower than the threshold " << fc_t_min_thre_);
    return false;
  }

  if (static_thrust_.maxCoeff() > thrust_max_ || static_thrust_.minCoeff() < thrust_min_) {
    if (verbose)
      RCLCPP_ERROR(LOGGER, "Invalid static thrust, max: %f, min: %f", static_thrust_.maxCoeff(),
                   static_thrust_.minCoeff());
    return false;
  }

  return true;
}

double RobotModel::calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk) {
  Eigen::Vector3d uixuj = ui.cross(uj);
  if (uixuj.norm() < 10e-5) {
    return 0.0;
  }
  return uixuj.dot(uk) / uixuj.norm();
}

std::vector<Eigen::Vector3d> RobotModel::calcV() {
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();
  std::vector<Eigen::Vector3d> v(rotor_num);

  for (int i = 0; i < rotor_num; ++i) v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
  return v;
}

void RobotModel::calcFeasibleControlFDists() {
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();

  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  Eigen::Vector3d gravity_force = getMass() * gravity_3d_;

  int index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    const Eigen::Vector3d& u_i = u.at(i);
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      const Eigen::Vector3d& u_j = u.at(j);

      double dist_ij = 0.0;
      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        const Eigen::Vector3d& u_k = u.at(k);
        double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
        dist_ij += std::max(0.0, u_triple_product * thrust_max);
      }

      Eigen::Vector3d uixuj = u_i.cross(u_j);
      if (uixuj.norm() < 10e-5) {
        fc_f_dists_(index) = 0;
      } else {
        fc_f_dists_(index) = fabs(dist_ij - (uixuj.dot(gravity_force) / uixuj.norm()));
      }

      index++;
    }
  }
  fc_f_min_ = fc_f_dists_.minCoeff();
}

void RobotModel::calcFeasibleControlTDists() {
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();

  const auto v = calcV();
  int index = 0;

  for (int i = 0; i < rotor_num; ++i) {
    const Eigen::Vector3d& v_i = v.at(i);
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      const Eigen::Vector3d& v_j = v.at(j);
      double dist_ij = 0.0;
      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        const Eigen::Vector3d& v_k = v.at(k);
        double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
        dist_ij += std::max(0.0, v_triple_product * thrust_max);
      }
      fc_t_dists_(index) = dist_ij;
      index++;
    }
  }

  fc_t_min_ = fc_t_dists_.minCoeff();
}

std::unique_ptr<tinyxml2::XMLDocument> RobotModel::getRobotModelXml(const std::string& param,
                                                                    rclcpp::Node::SharedPtr node) {
  std::string xml_string;
  if (!node->get_parameter(param, xml_string)) {
    RCLCPP_ERROR(LOGGER, "parameter '%s' not set", param.c_str());
    return nullptr;
  }
  auto doc = std::make_unique<tinyxml2::XMLDocument>();
  doc->Parse(xml_string.c_str());
  return doc;
}

KDL::JntArray RobotModel::jointMsgToKdl(const sensor_msgs::msg::JointState& state) const {
  KDL::JntArray joint_positions(tree_.getNrOfJoints());
  for (unsigned int i = 0; i < state.position.size(); ++i) {
    auto itr = joint_index_map_.find(state.name[i]);
    if (itr != joint_index_map_.end()) joint_positions(itr->second) = state.position[i];
  }
  return joint_positions;
}

sensor_msgs::msg::JointState RobotModel::kdlJointToMsg(const KDL::JntArray& joint_positions) const {
  sensor_msgs::msg::JointState state;
  state.name.reserve(joint_index_map_.size());
  state.position.reserve(joint_index_map_.size());
  for (const auto& actuator : joint_index_map_) {
    state.name.push_back(actuator.first);
    state.position.push_back(joint_positions(actuator.second));
  }
  return state;
}

KDL::JntArray RobotModel::convertEigenToKDL(const Eigen::VectorXd& joint_vector) {
  const auto& joint_indices = getJointIndices();
  KDL::JntArray joint_positions(getTree().getNrOfJoints());
  for (unsigned int i = 0; i < joint_indices.size(); ++i) {
    joint_positions(joint_indices.at(i)) = joint_vector(i);
  }
  return joint_positions;
}

}  // namespace aerial_robot_model
