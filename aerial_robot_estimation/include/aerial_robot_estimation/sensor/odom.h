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

#pragma once

/* basic plugin */
#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

/* ros */
#include <tf2_ros/static_transform_broadcaster.h>

/* ros messages */
#include <nav_msgs/msg/odometry.h>

namespace sensor_plugin {
  enum {ONLY_POS_MODE = 0, ONLY_VEL_MODE = 1, POS_VEL_MODE = 2,};

  class Odometry :public sensor_plugin::SensorBase {
  public:

    Odometry();
    ~Odometry(){}

    void initialize(rclcpp::Node::SharedPtr node,
                    RobotModelPtr robot_model,
                    EstimatorPtr estimator,
                    string sensor_name, int index) override;


    const bool odomPosMode();

    const KDL::Frame& getBasePose() const { return base_pose_; }

  private:
    /* ros */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;

    /* reconfigurable varaible */
    double throttle_rate_;
    double level_pos_noise_sigma_;
    double z_pos_noise_sigma_;
    double vel_noise_sigma_;
    double vel_outlier_thresh_;
    int fusion_mode_;
    bool local_vel_mode_;

    KDL::Frame origin_offset_; // ^{w}H_{o}: transform between two origins
    KDL::Frame base_pose_; // ^{w}H_{b} estimated by odometry sensor
    KDL::Twist base_twist_;
    KDL::Frame sensor_pose_, prev_sensor_pose_;
    KDL::Twist sensor_twist_, prev_sensor_twist_;

    std::string odom_origin_frame_;

    double ref_time_stamp_;
    aerial_robot_msgs::msg::States states_; /* for debug */

    bool checkStatus();
    bool calcuateOriginOffset();
    void calculateBasePose();
    void calculateBaseVelocity();
    bool isMsgNan(nav_msgs::msg::Odometry msg);
    void activateFuser() override;
    void estimateProcess() override;

    void preProcessState() override;
    void fuse() override;
    void print();

    void publish() override;
    void tfBroadcast();
    void rosParamInit() override;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr vo_msg);
  };
};



