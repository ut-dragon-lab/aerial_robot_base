#include "aerial_robot_model/numerical_jacobians.h"

namespace aerial_robot_model {

NumericalJacobian::NumericalJacobian(const rclcpp::NodeOptions& options,
                                     std::unique_ptr<transformable::RobotModel> robot_model)
    : Node("numerical_jacobian", options), robot_model_(std::move(robot_model)), initialized_(false) {
  //--- declare & fetch parameters ---
  this->declare_parameter<bool>("rostest", true);
  this->declare_parameter<double>("delta", 1e-5);
  this->declare_parameter<bool>("check_thrust_force", true);
  this->declare_parameter<bool>("check_joint_torque", true);
  this->declare_parameter<bool>("check_cog_motion", true);
  this->declare_parameter<bool>("check_feasible_control", true);
  this->declare_parameter<double>("thrust_force_diff_thre", 1e-3);
  this->declare_parameter<double>("joint_torque_diff_thre", 1e-3);
  this->declare_parameter<double>("cog_vel_diff_thre", 1e-3);
  this->declare_parameter<double>("l_momentum_diff_thre", 1e-3);
  this->declare_parameter<double>("feasible_control_force_diff_thre", 1e-3);
  this->declare_parameter<double>("feasible_control_torque_diff_thre", 1e-3);

  this->get_parameter("rostest", rostest_);
  this->get_parameter("delta", delta_);
  this->get_parameter("check_thrust_force", check_thrust_force_);
  this->get_parameter("check_joint_torque", check_joint_torque_);
  this->get_parameter("check_cog_motion", check_cog_motion_);
  this->get_parameter("check_feasible_control", check_feasible_control_);
  this->get_parameter("thrust_force_diff_thre", thrust_force_diff_thre_);
  this->get_parameter("joint_torque_diff_thre", joint_torque_diff_thre_);
  this->get_parameter("cog_vel_diff_thre", cog_vel_diff_thre_);
  this->get_parameter("l_momentum_diff_thre", l_momentum_diff_thre_);
  this->get_parameter("feasible_control_force_diff_thre", feasible_control_force_diff_thre_);
  this->get_parameter("feasible_control_torque_diff_thre", feasible_control_torque_diff_thre_);

  //--- subscriptions ---
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SystemDefaultsQoS(),
      std::bind(&NumericalJacobian::jointStateCallback, this, std::placeholders::_1));

  desire_coordinate_sub_ = this->create_subscription<spinal::msg::DesireCoord>(
      "desire_coordinate", rclcpp::SystemDefaultsQoS(),
      std::bind(&NumericalJacobian::desireCoordinateCallback, this, std::placeholders::_1));
}

void NumericalJacobian::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr state) {
  robot_model_->updateRobotModel(*state);
  robot_model_->updateJacobians();
  initialized_ = true;
  if (!rostest_) {
    checkJacobians();
  }
}

void NumericalJacobian::desireCoordinateCallback(const spinal::msg::DesireCoord::ConstSharedPtr msg) {
  robot_model_->setCogDesireOrientation(msg->roll, msg->pitch, msg->yaw);
}

bool NumericalJacobian::checkJacobians() {
  bool ok = true;
  if (check_thrust_force_) ok &= checkThrustForceJacobian();
  if (check_joint_torque_) ok &= checkJointTorqueJacobian();
  if (check_cog_motion_) ok &= checkCoGMomentumJacobian();
  if (check_feasible_control_) ok &= checkFeasibleControlJacobian();
  return ok;
}
Eigen::MatrixXd NumericalJacobian::thrustForceNumericalJacobian(std::vector<int> joint_indices) {
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const auto& u = getRobotModel().getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRobotModel().getRotorDirection();
  const double m_f_rate = getRobotModel().getMFRate();
  const std::string baselink = getRobotModel().getBaselinkName();
  const int rotor_num = getRobotModel().getRotorNum();
  const int full_body_dof = 6 + joint_indices.size();

  Eigen::MatrixXd q_mat = getRobotModel().getThrustWrenchMatrix();

  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot =
      getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(rotor_num, full_body_dof);
  Eigen::VectorXd nominal_static_thrust = getRobotModel().getStaticThrust();

  int col_index = 6;

  // intermediate results
  {
    Eigen::MatrixXd J_g = Eigen::MatrixXd::Zero(6, full_body_dof);
    Eigen::MatrixXd J_thrust = Eigen::MatrixXd::Zero(q_mat.rows(), full_body_dof);
    Eigen::VectorXd nominal_wrench_g = getRobotModel().calcGravityWrenchOnRoot();
    Eigen::VectorXd nominal_wrench_thrust = q_mat * nominal_static_thrust;
    auto perturbationSeparateForce = [&](int col, KDL::JntArray joint_angles) {
      getRobotModel().updateRobotModel(joint_angles);
      Eigen::VectorXd perturbated_wrench_g = getRobotModel().calcGravityWrenchOnRoot();
      J_g.col(col) = (perturbated_wrench_g - nominal_wrench_g) / delta_;
      getRobotModel().calcWrenchMatrixOnRoot();
      q_mat = getRobotModel().getThrustWrenchMatrix();
      Eigen::VectorXd perturbated_wrench_thrust = q_mat * nominal_static_thrust;
      J_thrust.col(col) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_;
    };

    // joint part
    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions(joint_positions);
      perturbation_joint_positions(joint_index) += delta_;
      getRobotModel().updateRobotModel(perturbation_joint_positions);
      getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
      perturbationSeparateForce(col_index, perturbation_joint_positions);
      col_index++;
    }
    // virtual 6dof root
    // roll
    getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
    perturbationSeparateForce(3, joint_positions);

    // pitch
    getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
    perturbationSeparateForce(4, joint_positions);

    // yaw
    getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
    perturbationSeparateForce(5, joint_positions);

    // reset
    getRobotModel().setCogDesireOrientation(baselink_rot);
    getRobotModel().updateRobotModel(joint_positions);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of wrench_gravity_jacobian: \n" << J_g);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of wrench_thrust_jacobian: \n" << J_thrust);
  }

  auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles) {
    getRobotModel().updateRobotModel(joint_angles);
    const Eigen::VectorXd static_thrust = getRobotModel().getStaticThrust();
    J_lambda.col(col) = (static_thrust - nominal_static_thrust) / delta_;
  };

  col_index = 6;
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);  // necessary
    perturbationStaticThrust(col_index, perturbation_joint_positions);
    col_index++;
  }

  // virtual 6dof root
  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbationStaticThrust(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbationStaticThrust(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbationStaticThrust(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot);
  getRobotModel().updateRobotModel(joint_positions);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical lambda_jacobian: \n" << J_lambda);

  return J_lambda;
}

Eigen::MatrixXd NumericalJacobian::jointTorqueNumericalJacobian(std::vector<int> joint_indices) {
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int full_body_dof = 6 + joint_indices.size();
  Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(getRobotModel().getJointNum(), full_body_dof);
  const std::string baselink = getRobotModel().getBaselinkName();
  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot =
      getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  getRobotModel().calcJointTorque();

  int col_index = 6;
  const Eigen::VectorXd nominal_joint_torque = getRobotModel().getJointTorque();

  auto perturbationJointTorque = [&](int col, KDL::JntArray joint_angles) {
    getRobotModel().updateRobotModel(joint_angles);
    getRobotModel().calcJointTorque();
    const Eigen::VectorXd joint_torque = getRobotModel().getJointTorque();
    J_t.col(col) = (joint_torque - nominal_joint_torque) / delta_;
  };

  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
    perturbationJointTorque(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbationJointTorque(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbationJointTorque(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbationJointTorque(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot);  // set the orientation of root
  getRobotModel().updateRobotModel(joint_positions);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of joint_torque_jacobian: \n" << J_t);

  // intermediate results
  {
    col_index = 6;
    const auto& inertia_map = getRobotModel().getInertiaMap();
    std::vector<Eigen::MatrixXd> nominal_thrust_coord_jacobians = getRobotModel().getThrustCoordJacobians();
    std::vector<Eigen::MatrixXd> nominal_cog_coord_jacobians = getRobotModel().getCOGCoordJacobians();
    Eigen::MatrixXd J_t_j = Eigen::MatrixXd::Zero(joint_indices.size(), full_body_dof);

    auto perturbationJointTorqueSeparate = [&](int col, KDL::JntArray joint_angles) {
      getRobotModel().updateRobotModel(joint_angles);
      getRobotModel().calcBasicKinematicsJacobian();  // update thrust_coord_jacobians
      getRobotModel().calcJointTorque();

      std::vector<Eigen::MatrixXd> thrust_coord_jacobians = getRobotModel().getThrustCoordJacobians();
      std::vector<Eigen::MatrixXd> cog_coord_jacobians = getRobotModel().getCOGCoordJacobians();
      const auto& thrust_wrench_units = getRobotModel().getThrustWrenchUnits();
      const auto& static_thrust = getRobotModel().getStaticThrust();
#if 1
      for (int i = 0; i < getRobotModel().getRotorNum(); ++i) {
        J_t_j.col(col) -= (thrust_coord_jacobians.at(i) - nominal_thrust_coord_jacobians.at(i))
                              .rightCols(joint_indices.size())
                              .transpose() *
                          thrust_wrench_units.at(i) * static_thrust(i) / delta_;
      }
#else
      int seg_index = 0;
      for (const auto& inertia : inertia_map) {
        J_t_j.col(col) -= (cog_coord_jacobians.at(seg_index) - nominal_cog_coord_jacobians.at(seg_index))
                              .rightCols(joint_indices.size())
                              .transpose() *
                          inertia.second.getMass() * (-getRobotModel().getGravity()) / delta_;
        seg_index++;
      }
#endif
    };

    for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_positions = joint_positions;
      perturbation_joint_positions(joint_index) += delta_;
      getRobotModel().updateRobotModel(perturbation_joint_positions);
      getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
      perturbationJointTorqueSeparate(col_index, perturbation_joint_positions);
      col_index++;
    }

    // roll
    getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
    perturbationJointTorqueSeparate(3, joint_positions);

    // pitch
    getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
    perturbationJointTorqueSeparate(4, joint_positions);

    // yaw
    getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
    perturbationJointTorqueSeparate(5, joint_positions);

    // reset
    getRobotModel().setCogDesireOrientation(baselink_rot);  // set the orientation of root
    getRobotModel().updateRobotModel(joint_positions);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of intermediate result of joint torque jacobian: \n"
                                                << J_t_j);
  }

  return J_t;
}

std::vector<Eigen::MatrixXd> NumericalJacobian::cogMomentumNumericalJacobian(std::vector<int> joint_indices) {
  const auto& inertia_map = getRobotModel().getInertiaMap();
  const auto nominal_seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int full_body_dof = 6 + joint_indices.size();
  double mass_all = getRobotModel().getMass();
  KDL::Rotation nominal_root_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>() *
                                   nominal_seg_frames.at(getRobotModel().getBaselinkName()).M.Inverse();

  Eigen::MatrixXd J_cog = Eigen::MatrixXd::Zero(3, full_body_dof);
  Eigen::MatrixXd J_L = Eigen::MatrixXd::Zero(3, full_body_dof);

  KDL::Vector nominal_cog = getRobotModel().getCog<KDL::Frame>().p;

  auto perturbation = [&](int col, KDL::JntArray joint_angles, KDL::Rotation root_rot) {
    getRobotModel().updateRobotModel(joint_angles);
    const auto seg_frames = getRobotModel().getSegmentsTf();

    for (const auto& seg : inertia_map) {
      Eigen::Vector3d p_momentum_jacobian =
          aerial_robot_model::kdlToEigen(root_rot * (seg_frames.at(seg.first) * seg.second.getCOG()) -
                                         nominal_root_rot * (nominal_seg_frames.at(seg.first) * seg.second.getCOG())) *
          seg.second.getMass() / delta_;

      J_cog.col(col) += p_momentum_jacobian / mass_all;

      J_L.col(col) += aerial_robot_model::kdlToEigen(
                          nominal_root_rot * (nominal_seg_frames.at(seg.first) * seg.second.getCOG() - nominal_cog))
                          .cross(p_momentum_jacobian);

      KDL::Rotation inertia_rot = nominal_root_rot * nominal_seg_frames.at(seg.first).M;
      KDL::RigidBodyInertia seg_inertia = seg.second;
      Eigen::MatrixXd rotional_inertia = aerial_robot_model::kdlToEigen(
          (inertia_rot * seg_inertia.RefPoint(seg.second.getCOG())).getRotationalInertia());

      KDL::Rotation pertuabated_inertia_rot = root_rot * seg_frames.at(seg.first).M;
      Eigen::MatrixXd omega_skew =
          (aerial_robot_model::kdlToEigen(pertuabated_inertia_rot) - aerial_robot_model::kdlToEigen(inertia_rot)) /
          delta_ * aerial_robot_model::kdlToEigen(inertia_rot.Inverse());

      Eigen::Vector3d omega(omega_skew(2, 1), omega_skew(0, 2), omega_skew(1, 0));
      J_L.col(col) += rotional_inertia * omega;
    }

    // simple way to get cog velocity jacobian
    // J_cog.col(col) = aerial_robot_model::kdlToEigen(root_rot * getCog<KDL::Frame>().p - nominal_root_rot *
    // nominal_cog) / delta_;
  };

  /* joint */
  int col_index = 6;
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions(joint_positions);
    perturbation_joint_positions(joint_index) += delta_;
    perturbation(col_index, perturbation_joint_positions, nominal_root_rot);
    col_index++;
  }

  // virtual 6dof root
  J_cog.leftCols(3) = aerial_robot_model::kdlToEigen(nominal_root_rot);
  // roll
  perturbation(3, joint_positions, nominal_root_rot * KDL::Rotation::RPY(delta_, 0, 0));

  // pitch
  perturbation(4, joint_positions, nominal_root_rot * KDL::Rotation::RPY(0, delta_, 0));

  // yaw
  perturbation(5, joint_positions, nominal_root_rot * KDL::Rotation::RPY(0, 0, delta_));

  // reset
  getRobotModel().updateRobotModel(joint_positions);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical cog_jacobian: \n" << J_cog);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical angular momentum_jacobian: \n" << J_L);

  std::vector<Eigen::MatrixXd> out;
  out.push_back(J_cog);
  out.push_back(J_L);

  return out;
}

std::vector<Eigen::MatrixXd> NumericalJacobian::feasibleControlNumericalJacobian(std::vector<int> joint_indices) {
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int rotor_num = getRobotModel().getRotorNum();
  const int full_body_dof = 6 + joint_indices.size();
  const std::string baselink = getRobotModel().getBaselinkName();
  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot =
      getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  getRobotModel().updateRobotModel(joint_positions);
  getRobotModel().calcFeasibleControlJacobian();  // update approx_fc_f_dists_, approx_fc_t_dists_

  int col_index = 6;

  Eigen::VectorXd nominal_fc_f_dists = getRobotModel().getFeasibleControlFDists();
  Eigen::VectorXd nominal_approx_fc_f_dists = getRobotModel().getApproxFeasibleControlFDists();
  Eigen::VectorXd nominal_fc_t_dists = getRobotModel().getFeasibleControlTDists();
  Eigen::VectorXd nominal_approx_fc_t_dists = getRobotModel().getApproxFeasibleControlTDists();

  RCLCPP_DEBUG_STREAM(this->get_logger(), "nominal_fc_f_dists :" << nominal_fc_f_dists.transpose());
  RCLCPP_DEBUG_STREAM(this->get_logger(), "nominal_approx_fc_f_dists :" << nominal_approx_fc_f_dists.transpose());
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of approx and nominal_fc_f_dists :"
                                              << (nominal_approx_fc_f_dists - nominal_fc_f_dists).transpose());

  RCLCPP_DEBUG_STREAM(this->get_logger(), "nominal_fc_t_dists :" << nominal_fc_t_dists.transpose());
  RCLCPP_DEBUG_STREAM(this->get_logger(), "nominal_approx_fc_t_dists :" << nominal_approx_fc_t_dists.transpose());
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of approx and nominal_fc_t_dists :"
                                              << (nominal_approx_fc_t_dists - nominal_fc_t_dists).transpose());

  Eigen::MatrixXd J_fc_f_dists = Eigen::MatrixXd::Zero(nominal_fc_f_dists.size(), full_body_dof);
  Eigen::MatrixXd J_approx_fc_f_dists = Eigen::MatrixXd::Zero(nominal_fc_f_dists.size(), full_body_dof);
  Eigen::MatrixXd J_fc_t_dists = Eigen::MatrixXd::Zero(nominal_fc_t_dists.size(), full_body_dof);
  Eigen::MatrixXd J_approx_fc_t_dists = Eigen::MatrixXd::Zero(nominal_fc_t_dists.size(), full_body_dof);

  auto perturbation = [&](int col, KDL::JntArray joint_angles) {
    getRobotModel().updateRobotModel(joint_angles);
    getRobotModel().calcFeasibleControlJacobian();  // update approx_fc_f_dists_, approx_fc_t_dists_

    Eigen::VectorXd fc_f_dists = getRobotModel().getFeasibleControlFDists();
    Eigen::VectorXd approx_fc_f_dists = getRobotModel().getApproxFeasibleControlFDists();
    Eigen::VectorXd fc_t_dists = getRobotModel().getFeasibleControlTDists();
    Eigen::VectorXd approx_fc_t_dists = getRobotModel().getApproxFeasibleControlTDists();

    J_fc_f_dists.col(col) = (fc_f_dists - nominal_fc_f_dists) / delta_;
    J_approx_fc_f_dists.col(col) = (approx_fc_f_dists - nominal_approx_fc_f_dists) / delta_;
    J_fc_t_dists.col(col) = (fc_t_dists - nominal_fc_t_dists) / delta_;
    J_approx_fc_t_dists.col(col) = (approx_fc_t_dists - nominal_approx_fc_t_dists) / delta_;

    for (int i = 0; i < nominal_fc_f_dists.size(); i++) {
      if (std::isnan(J_fc_f_dists.col(col)(i))) J_fc_f_dists.col(col)(i) = 0;
      if (std::isnan(J_approx_fc_f_dists.col(col)(i))) J_approx_fc_f_dists.col(col)(i) = 0;
      if (std::isnan(J_fc_t_dists.col(col)(i))) J_fc_t_dists.col(col)(i) = 0;
      if (std::isnan(J_approx_fc_t_dists.col(col)(i))) J_approx_fc_t_dists.col(col)(i) = 0;
    }
  };

  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbation(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbation(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbation(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot);
  getRobotModel().updateRobotModel(joint_positions);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of J_fc_f_dists: \n" << J_fc_f_dists);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of J_approx_fc_f_dists: \n" << J_approx_fc_f_dists);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of J_fc_t_dists: \n" << J_fc_t_dists);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "numerical result of J_approx_fc_t_dists: \n" << J_approx_fc_t_dists);

  std::vector<Eigen::MatrixXd> out;
  out.push_back(J_approx_fc_f_dists);
  out.push_back(J_approx_fc_t_dists);
  out.push_back(J_fc_f_dists);
  out.push_back(J_fc_t_dists);

  return out;
}

bool NumericalJacobian::checkThrustForceJacobian(std::vector<int> joint_indices) {
  if (joint_indices.empty()) joint_indices = getRobotModel().getJointIndices();

  const Eigen::MatrixXd analytical_jacobi = getRobotModel().getLambdaJacobian();
  const Eigen::MatrixXd numerical_jacobi = thrustForceNumericalJacobian(joint_indices);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "analytical lambda_jacobian: \n" << analytical_jacobi);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of lambda jacobian: \n" << numerical_jacobi - analytical_jacobi);

  double max_diff = (numerical_jacobi - analytical_jacobi).maxCoeff();
  double min_diff = (numerical_jacobi - analytical_jacobi).minCoeff();
  if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if (max_diff < thrust_force_diff_thre_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "max diff of lambda jacobian: " << max_diff);
    return true;
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "max diff of lambda jacobian: " << max_diff << ", exceed!");
    return false;
  }
}

bool NumericalJacobian::checkJointTorqueJacobian(std::vector<int> joint_indices) {
  if (joint_indices.empty()) joint_indices = getRobotModel().getJointIndices();

  const Eigen::MatrixXd analytical_jacobi = getRobotModel().getJointTorqueJacobian();
  const Eigen::MatrixXd numerical_jacobi = jointTorqueNumericalJacobian(joint_indices);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "analytical joint torque jacobian: \n" << analytical_jacobi);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of joint torque jacobian: \n" << numerical_jacobi - analytical_jacobi);

  double max_diff = (numerical_jacobi - analytical_jacobi).maxCoeff();
  double min_diff = (numerical_jacobi - analytical_jacobi).minCoeff();
  if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if (max_diff < joint_torque_diff_thre_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "max diff of joint torque jacobian: " << max_diff);
    return true;
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "max diff of joint torque jacobian: " << max_diff << ", exceed!");
    return false;
  }
}

bool NumericalJacobian::checkCoGMomentumJacobian(std::vector<int> joint_indices) {
  if (joint_indices.empty()) joint_indices = getRobotModel().getJointIndices();

  bool flag = true;

  const Eigen::MatrixXd analytical_cog_jacobi = getRobotModel().getCOGJacobian();
  const Eigen::MatrixXd analytical_L_jacobi = getRobotModel().getLMomentumJacobian();
  const std::vector<Eigen::MatrixXd> numerical_jacobis = cogMomentumNumericalJacobian(joint_indices);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "analytical cog vel jacobian: \n" << analytical_cog_jacobi);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of cog vel jacobian: \n"
                                              << numerical_jacobis.at(0) - analytical_cog_jacobi);

  double max_diff = (numerical_jacobis.at(0) - analytical_cog_jacobi).maxCoeff();
  double min_diff = (numerical_jacobis.at(0) - analytical_cog_jacobi).minCoeff();

  if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if (max_diff < cog_vel_diff_thre_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "max diff of cog vel jacobian: " << max_diff);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "max diff of cog vel jacobian: " << max_diff << ", exceed!");
    flag = false;
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "analytical L momentum jacobian: \n" << analytical_L_jacobi);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of L momentum jacobian: \n"
                                              << numerical_jacobis.at(1) - analytical_L_jacobi);

  max_diff = (numerical_jacobis.at(1) - analytical_L_jacobi).maxCoeff();
  min_diff = (numerical_jacobis.at(1) - analytical_L_jacobi).minCoeff();

  if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if (max_diff < l_momentum_diff_thre_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "max diff of L momentum jacobian: " << max_diff);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "max diff of L momentum jacobian: " << max_diff << ", exceed!");
    flag = false;
  }

  return flag;
}

bool NumericalJacobian::checkFeasibleControlJacobian(std::vector<int> joint_indices) {
  if (joint_indices.empty()) joint_indices = getRobotModel().getJointIndices();

  bool flag = true;

  const Eigen::MatrixXd analytical_fc_f_jacobi = getRobotModel().getFeasibleControlFDistsJacobian();
  const Eigen::MatrixXd analytical_fc_t_jacobi = getRobotModel().getFeasibleControlTDistsJacobian();
  const std::vector<Eigen::MatrixXd> numerical_jacobis = feasibleControlNumericalJacobian(joint_indices);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "analytical appprox feasible control force distances jacobian: \n"
                                              << analytical_fc_f_jacobi);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of appprox fc_f_dists_jacobian with: \n"
                                              << numerical_jacobis.at(0) - analytical_fc_f_jacobi);

  double max_diff = (numerical_jacobis.at(0) - analytical_fc_f_jacobi).maxCoeff();
  double min_diff = (numerical_jacobis.at(0) - analytical_fc_f_jacobi).minCoeff();

  if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if (max_diff < feasible_control_force_diff_thre_) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "max diff of appprox feasible control force distances jacobian: " << max_diff);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "max diff of appprox feasible control force distances jacobian: " << max_diff << ", exceed!");
    flag = false;
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "analytical appprox feasible control torque distances jacobian: \n"
                                              << analytical_fc_t_jacobi);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of appprox fc_t_dists_jacobian with: \n"
                                              << numerical_jacobis.at(1) - analytical_fc_t_jacobi);

  max_diff = (numerical_jacobis.at(1) - analytical_fc_t_jacobi).maxCoeff();
  min_diff = (numerical_jacobis.at(1) - analytical_fc_t_jacobi).minCoeff();

  if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if (max_diff < feasible_control_torque_diff_thre_) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "max diff of appprox feasible control torque distances jacobian: " << max_diff);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "max diff of appprox feasible control torque distances jacobian: " << max_diff << ", exceed!");
    flag = false;
  }

  // not critical, compare with non-approximated feasible control (i.e. relu)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of non-approx fc_t_dists_jacobian: \n"
                                                << numerical_jacobis.at(2) - analytical_fc_f_jacobi);

    max_diff = (numerical_jacobis.at(2) - analytical_fc_f_jacobi).maxCoeff();
    min_diff = (numerical_jacobis.at(2) - analytical_fc_f_jacobi).minCoeff();

    if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "not ciritcal: max diff of non-approx feasible control force distances jacobian: " << max_diff);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "diff of non-approx fc_t_dists_jacobian: \n"
                                                << numerical_jacobis.at(3) - analytical_fc_t_jacobi);

    max_diff = (numerical_jacobis.at(3) - analytical_fc_t_jacobi).maxCoeff();
    min_diff = (numerical_jacobis.at(3) - analytical_fc_t_jacobi).minCoeff();

    if (fabs(min_diff) > max_diff) max_diff = fabs(min_diff);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "not critical: max diff of non-approx feasible control torque distances jacobian: " << max_diff);
  }

  return flag;
}

};  // namespace aerial_robot_model
