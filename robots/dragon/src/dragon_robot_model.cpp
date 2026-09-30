// SPDX-License-Identifier: BSD-3-Clause
// DRAGON's articulated geometry uses the shared ROS 2 vectoring allocator.
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/SVD>

class DragonRobotModel : public GimbalrotorRobotModel
{
public:
  // A fixed-axis rotor feasibility test is inappropriate for two-axis gimbals.
  // Check the full vectoring wrench map and static force limits instead.
  bool stabilityCheck(bool verbose = false) override
  {
    const auto origins = getRotorsOriginFromCog<Eigen::Vector3d>();
    if (origins.size() != 4 || !std::isfinite(getMass()) || getMass() <= 0.0) return false;
    Eigen::MatrixXd map = Eigen::MatrixXd::Zero(6, 12);
    for (int i = 0; i < 4; ++i)
    {
      map.block<3, 3>(0, 3 * i).setIdentity();
      map.block<3, 3>(3, 3 * i) = aerial_robot_model::skew(origins[i]);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(map, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues().minCoeff() < 1e-4) return false;
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench(2) = getMass() * 9.798015;
    const Eigen::VectorXd force = svd.solve(wrench);
    for (int i = 0; i < 4; ++i)
      if (!force.segment<3>(3 * i).allFinite() || force.segment<3>(3 * i).norm() > getThrustUpperLimit())
        return false;
    (void)verbose;
    return true;
  }
};
PLUGINLIB_EXPORT_CLASS(DragonRobotModel, aerial_robot_model::RobotModel)
