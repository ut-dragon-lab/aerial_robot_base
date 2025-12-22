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

void RobotModel::calcFeasibleControlJacobian() {
  // reference:
  // https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8967725

  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = 6 + joint_num;
  const auto& p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const double thrust_max = getThrustUpperLimit();
  Eigen::Vector3d fg = getMass() * getGravity3d();
  const double m_f_rate = getMFRate();
  const double epsilon = getEpsilon();

  const auto v = calcV();
  std::vector<Eigen::MatrixXd> v_jacobians;
  for (int i = 0; i < rotor_num; ++i) {
    v_jacobians.push_back(-skew(u.at(i)) * p_jacobians_.at(i) + skew(p.at(i)) * u_jacobians_.at(i) +
                          m_f_rate * sigma.at(i + 1) * u_jacobians_.at(i));
  }

  // calc jacobian of f_min_ij, t_min_ij
  int index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;

      approx_fc_f_dists_(index) = 0;
      fc_f_dists_jacobian_.row(index) = Eigen::MatrixXd::Zero(1, ndof);
      approx_fc_t_dists_(index) = 0;
      fc_t_dists_jacobian_.row(index) = Eigen::MatrixXd::Zero(1, ndof);

      double approx_f_dist = 0.0;
      double approx_t_dist = 0.0;
      Eigen::MatrixXd d_f_min = Eigen::MatrixXd::Zero(1, ndof);
      Eigen::MatrixXd d_t_min = Eigen::MatrixXd::Zero(1, ndof);

      const Eigen::Vector3d& u_i = u.at(i);
      const Eigen::Vector3d& u_j = u.at(j);
      const Eigen::Vector3d uixuj = u_i.cross(u_j);
      const Eigen::MatrixXd& d_u_i = u_jacobians_.at(i);
      const Eigen::MatrixXd& d_u_j = u_jacobians_.at(j);
      const Eigen::MatrixXd d_uixuj = -skew(u_j) * d_u_i + skew(u_i) * d_u_j;

      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d& v_j = v.at(j);
      const Eigen::Vector3d vixvj = v_i.cross(v_j);
      const Eigen::MatrixXd& d_v_i = v_jacobians.at(i);
      const Eigen::MatrixXd& d_v_j = v_jacobians.at(j);
      const Eigen::MatrixXd d_vixvj = -skew(v_j) * d_v_i + skew(v_i) * d_v_j;

      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;

        // u
        const Eigen::Vector3d& u_k = u.at(k);
        const double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
        const Eigen::MatrixXd& d_u_k = u_jacobians_.at(k);
        Eigen::MatrixXd d_u_triple_product =
            (uixuj / uixuj.norm()).transpose() * d_u_k +
            u_k.transpose() *
                (d_uixuj / uixuj.norm() - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);
        d_f_min += sigmoid(u_triple_product * thrust_max, epsilon) * d_u_triple_product * thrust_max;
        approx_f_dist += reluApprox(u_triple_product * thrust_max, epsilon);

        // v
        const Eigen::Vector3d& v_k = v.at(k);
        const double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
        const Eigen::MatrixXd& d_v_k = v_jacobians.at(k);
        Eigen::MatrixXd d_v_triple_product =
            (vixvj / vixvj.norm()).transpose() * d_v_k +
            v_k.transpose() *
                (d_vixvj / vixvj.norm() - vixvj / (vixvj.norm() * vixvj.squaredNorm()) * vixvj.transpose() * d_vixvj);
        d_t_min += sigmoid(v_triple_product * thrust_max, epsilon) * d_v_triple_product * thrust_max;
        approx_t_dist += reluApprox(v_triple_product * thrust_max, epsilon);
      }

      if (uixuj.norm() > 10e-5) {
        double uixuj_fg = uixuj.dot(fg) / uixuj.norm();
        Eigen::MatrixXd d_uixuj_fg = Eigen::MatrixXd::Zero(1, ndof);
        d_uixuj_fg = fg.transpose() * (d_uixuj / uixuj.norm() -
                                       uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);

        approx_fc_f_dists_(index) = absApprox(approx_f_dist - uixuj_fg, epsilon);
        fc_f_dists_jacobian_.row(index) = tanh(approx_f_dist - uixuj_fg, epsilon) * (d_f_min - d_uixuj_fg);
      }

      if (vixvj.norm() > 10e-5) {
        approx_fc_t_dists_(index) = approx_t_dist;
        fc_t_dists_jacobian_.row(index) = d_t_min;
      }

      index++;

    }  // j
  }    // i
}
