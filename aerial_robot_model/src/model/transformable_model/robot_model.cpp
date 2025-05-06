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
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model::transformable;

RobotModel::RobotModel() : aerial_robot_model::RobotModel() {}

void RobotModel::initialize(rclcpp::Node::SharedPtr node, bool init_with_rosparam, bool verbose, bool fixed_model,
                            double fc_f_min_thre, double fc_t_min_thre, double epsilon) {
  aerial_robot_model::RobotModel::initialize(node, init_with_rosparam, verbose, fixed_model, fc_f_min_thre,
                                             fc_t_min_thre, epsilon);

  std::function<void(const KDL::TreeElement&)> recursiveSegmentCheck = [&recursiveSegmentCheck,
                                                                        this](const KDL::TreeElement& tree_element) {
    const KDL::Segment current_seg = GetTreeElementSegment(tree_element);

    if (current_seg.getJoint().getType() != KDL::Joint::None && current_seg.getJoint().getName().find("joint") == 0) {
      link_joint_names_.push_back(current_seg.getJoint().getName());
      link_joint_indices_.push_back(tree_element.q_nr);
    }

    // recursive process
    for (const auto& elem : GetTreeElementChildren(tree_element)) {
      recursiveSegmentCheck(elem->second);
    }
  };
  recursiveSegmentCheck(getTree().getRootSegment()->second);

  for (auto itr : link_joint_names_) {
    auto joint_ptr = getUrdfModel().getJoint(itr);
    link_joint_lower_limits_.push_back(joint_ptr->limits->lower);
    link_joint_upper_limits_.push_back(joint_ptr->limits->upper);
  }

  resolveLinkLength();

  // jacobian
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int full_body_dof = 6 + joint_num;
  u_jacobians_.resize(rotor_num);
  p_jacobians_.resize(rotor_num);
  thrust_coord_jacobians_.resize(rotor_num);
  cog_coord_jacobians_.resize(getInertiaMap().size());
  cog_jacobian_.resize(3, full_body_dof);
  l_momentum_jacobian_.resize(3, full_body_dof);
  approx_fc_f_dists_.resize(rotor_num * (rotor_num - 1));
  approx_fc_t_dists_.resize(rotor_num * (rotor_num - 1));
  fc_f_dists_jacobian_.resize(rotor_num * (rotor_num - 1), full_body_dof);
  fc_t_dists_jacobian_.resize(rotor_num * (rotor_num - 1), full_body_dof);
  lambda_jacobian_.resize(rotor_num, full_body_dof);
  joint_torque_.resize(joint_num);
  joint_torque_jacobian_.resize(joint_num, full_body_dof);
}

void RobotModel::updateJacobians() { updateJacobians(getJointPositions(), false); }

void RobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model) {
  if (update_model) updateRobotModel(joint_positions);

  calcCoGMomentumJacobian();  // should be processed first

  calcBasicKinematicsJacobian();  // need cog_jacobian_

  calcLambdaJacobian();

  calcJointTorque(false);

  calcJointTorqueJacobian();

  calcFeasibleControlJacobian();
}
