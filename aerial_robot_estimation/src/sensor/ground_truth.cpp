// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, DRAGON Lab
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
 *   * Neither the name of the DRAGON Lab nor the names of its
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

#include <aerial_robot_estimation/sensor/ground_truth.h>


using namespace aerial_robot_estimation;

namespace sensor_plugin {
  GroundTruth::GroundTruth():
    sensor_plugin::SensorBase() { }

  void GroundTruth::initialize(rclcpp::Node::SharedPtr node,
                       RobotModelPtr robot_model,
                       EstimatorPtr estimator,
                       string sensor_name, int index) {

    SensorBase::initialize(node, robot_model, estimator, sensor_name, index);

    std::string topic_name;
    getParam<string>("topic_name", topic_name, "ground_truth");
    msg_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>
      (topic_name, rclcpp::SystemDefaultsQoS(),
       std::bind(&GroundTruth::odomCallback, this, std::placeholders::_1));

    rosParamInit();

    // This LPF simulates the smoothing of gyro in spinal for simulator
    lpf_omega_ = IirFilter(sample_freq_, cutoff_freq_, 3);
  }

  void GroundTruth::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    tf2::fromMsg(msg->pose.pose, pose_);
    tf2::fromMsg(msg->twist.twist, twist_);

    Eigen::Vector3d ang_vec;
    tf2::vectorKDLToEigen(twist_.rot, ang_vec);

    if(!estimator_->hasGroundTruthOdom()) {
      estimator_->receiveGroundTruthOdom(true);
      setStateStatus();

      /* set initial value for low pass filter */
      lpf_omega_.setInitValues(ang_vec);
    }

    /* do low pass filter for angular velocity */
    Eigen::Vector3d out_vec;
    out_vec = lpf_omega_.filterFunction(ang_vec);
    tf2::vectorEigenToKDL(out_vec, twist_.rot);

    /* set ground truth pose and twist */
    estimateProcess();
  }

  void GroundTruth::setStateStatus()
  {
    /* set ground truth */
    estimator_->setBasePosStateStatus(State::X, GROUND_TRUTH, true);
    estimator_->setBasePosStateStatus(State::Y, GROUND_TRUTH, true);
    estimator_->setBasePosStateStatus(State::Z, GROUND_TRUTH, true);
    estimator_->setBaseRotStateStatus(GROUND_TRUTH, true);
    estimator_->setCogRotStateStatus(GROUND_TRUTH, true);
  }

  void GroundTruth::estimateProcess() {

    /* base link */
    estimator_->setBasePose(GROUND_TRUTH, pose_);
    estimator_->setBaseTwist(GROUND_TRUTH, twist_);

    /* cog */
    KDL::Frame c2b_tf = robot_model_->getCog2Baselink<KDL::Frame>();
    KDL::Vector b2c_pos = c2b_tf.Inverse().p;

    /* rot_cog = rot_b * rot_base2cog */
    KDL::Rotation rot_c = pose_.M * c2b_tf.M.Inverse();
    estimator_->setCogOrientation(GROUND_TRUTH, rot_c);
    /* omega_cog = rot_cog2base * omega_b */
    KDL::Vector omega_c = c2b_tf.M * twist_.rot;
    estimator_->setCogAngularVel(GROUND_TRUTH, omega_c);

    /* pos_cog = pos_base + rot_b * pos_base2cog */
    KDL::Vector pos_c = pose_.p + pose_.M * b2c_pos;
    estimator_->setCogPos(GROUND_TRUTH, pos_c);
    /* vel_cog = vel_base + rob_b * (w x pos_base2cog) */
    KDL::Vector vel_c = twist_.vel + pose_.M * (twist_.rot * b2c_pos);
    estimator_->setCogVel(GROUND_TRUTH, vel_c);
  }

  void GroundTruth::rosParamInit() {
    getParam<double>("angular.sample_freq", sample_freq_, 100.0);
    getParam<double>("angular.cutoff_freq", cutoff_freq_, 20.0);
  }

};

/* plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::GroundTruth, sensor_plugin::SensorBase);


