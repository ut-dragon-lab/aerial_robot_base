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
#include <gimbalrotor/model/gimbalrotor_robot_model.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <pluginlib/class_list_macros.hpp>

void GimbalrotorRobotModel::initialize(rclcpp::Node::SharedPtr node, bool init_with_rosparam, bool verbose,
                                       bool fixed_model, double fc_f_min_thre, double fc_t_min_thre, double epsilon)
{
  (void)fixed_model;
  node->get_parameter_or("thrust_frame_prefix", thrust_frame_prefix_, std::string("rotor_arm"));
  aerial_robot_model::RobotModel::initialize(std::move(node), init_with_rosparam, verbose, false, fc_f_min_thre,
                                             fc_t_min_thre, epsilon);

  const int rotor_num = getRotorNum();
  {
    std::lock_guard<std::mutex> lock(links_rotation_mutex_);
    links_rotation_from_cog_.resize(rotor_num);
  }
  {
    std::lock_guard<std::mutex> lock(thrust_rotation_mutex_);
    thrust_coords_rot_.resize(rotor_num);
  }
}

void GimbalrotorRobotModel::updateRobotModelImpl(const KDL::JntArray &joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();

  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  std::vector<KDL::Rotation> thrust_coords_rot(getRotorNum());
  for (int i = 0; i < getRotorNum(); ++i)
  {
    const std::string thrust = thrust_frame_prefix_ + std::to_string(i + 1);
    KDL::Frame f;
    fk_solver.JntToCart(joint_positions, f, thrust);
    thrust_coords_rot[i] = cog_frame.Inverse() * f.M;
  }

  std::lock_guard<std::mutex> lock(thrust_rotation_mutex_);
  thrust_coords_rot_ = thrust_coords_rot;
}

PLUGINLIB_EXPORT_CLASS(GimbalrotorRobotModel, aerial_robot_model::RobotModel)
