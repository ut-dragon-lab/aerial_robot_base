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

#pragma once

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>  // ROS2 version
// eigen_conversions is not available in ROS2; use tf2_kdl and tf2_eigen for conversions

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/rotationalinertia.hpp>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace aerial_robot_model {

namespace {
template <class T1, class T2, class Callback>
std::vector<T1> convertVector(const std::vector<T2>& in, Callback callback) {
  std::vector<T1> out;
  out.reserve(in.size());
  for (const auto& elem : in) {
    out.push_back(callback(elem));
  }
  return out;
}
}  // namespace

inline bool isValidRotation(const KDL::Rotation& m) {
  double x, y, z, w;
  m.GetQuaternion(x, y, z, w);
  return (std::fabs(1.0 - Eigen::Quaterniond(w, x, y, z).squaredNorm()) < 1e-6);
}

inline geometry_msgs::msg::TransformStamped kdlToMsg(const KDL::Frame& in) { return tf2::kdlToTransform(in); }

inline geometry_msgs::msg::PointStamped kdlToMsg(const KDL::Vector& in) {
  tf2::Stamped<KDL::Vector> tmp;
  tmp.setData(in);
  geometry_msgs::msg::PointStamped out;
  tf2::convert(tmp, out);
  return out;
}

inline std::vector<geometry_msgs::msg::PointStamped> kdlToMsg(const std::vector<KDL::Vector>& in) {
  return convertVector<geometry_msgs::msg::PointStamped, KDL::Vector>(
      in, [](const KDL::Vector& v) -> geometry_msgs::msg::PointStamped { return kdlToMsg(v); });
}

inline Eigen::Affine3d kdlToEigen(const KDL::Frame& in) {
  auto tf_stamped = tf2::kdlToTransform(in);
  return tf2::transformToEigen(tf_stamped);
}

inline Eigen::Vector3d kdlToEigen(const KDL::Vector& in) {
  // convert KDL::Vector to geometry_msgs::msg::Point then to Eigen
  auto p = kdlToMsg(in);
  Eigen::Vector3d out;
  tf2::fromMsg(p.point, out);
  return out;
}

inline Eigen::Matrix3d kdlToEigen(const KDL::RotationalInertia& in) {
  return Eigen::Map<const Eigen::Matrix3d>(in.data);
}

inline Eigen::Matrix3d kdlToEigen(const KDL::Rotation& in) {
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(in.data);
}

inline std::vector<Eigen::Vector3d> kdlToEigen(const std::vector<KDL::Vector>& in) {
  return convertVector<Eigen::Vector3d, KDL::Vector>(
      in, [](const KDL::Vector& v) -> Eigen::Vector3d { return kdlToEigen(v); });
}

inline std::vector<Eigen::Matrix3d> kdlToEigen(const std::vector<KDL::Rotation>& in) {
  return convertVector<Eigen::Matrix3d, KDL::Rotation>(
      in, [](const KDL::Rotation& r) -> Eigen::Matrix3d { return kdlToEigen(r); });
}

inline tf2::Transform kdlToTf2(const KDL::Frame& in) {
  geometry_msgs::msg::TransformStamped tmp = kdlToMsg(in);
  tf2::Transform out;
  tf2::convert(tmp.transform, out);
  return out;
}

inline tf2::Vector3 kdlToTf2(const KDL::Vector& in) {
  auto p = kdlToMsg(in);
  tf2::Vector3 out;
  tf2::convert(p.point, out);
  return out;
}

inline std::vector<tf2::Vector3> kdlToTf2(const std::vector<KDL::Vector>& in) {
  return convertVector<tf2::Vector3, KDL::Vector>(in, [](const KDL::Vector& v) -> tf2::Vector3 { return kdlToTf2(v); });
}

}  // namespace aerial_robot_model
