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

/* ros messages */
#include <aerial_robot_msgs/msg/states.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <spinal/msg/imu.hpp>


namespace sensor_plugin {
  class Imu :public sensor_plugin::SensorBase{
  public:
    virtual void initialize(rclcpp::Node::SharedPtr node,
                            RobotModelPtr robot_model,
                            EstimatorPtr estimator,
                            string sensor_name, int index) override;

    ~Imu() {}
    Imu();

  protected:
    rclcpp::Subscription<spinal::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  ros_imu_pub_;
    rclcpp::Publisher<aerial_robot_msgs::msg::States>::SharedPtr  state_pub_;

    int calib_count_, calib_max_count_;
    double calib_time_;
    double dt_; /* sensor internal */


    /* reconfigurable varaible */
    double level_acc_noise_sigma_, z_acc_noise_sigma_;
    double level_acc_bias_noise_sigma_, z_acc_bias_noise_sigma_; /* sigma for kf */

    KDL::Vector omega_; /* the omega both of body frame */
    KDL::Vector mag_; /* the magnetometer of body frame */
    KDL::Vector acc_b_; /* the acceleration in baselink frame */
    KDL::Rotation raw_rot_; /* the raw rotation matrix from IMU */
    /* acc */
    std::array<KDL::Vector, 2> acc_w_; /* the acceleration in world frame, for estimate_mode and expriment_mode */
    std::array<KDL::Vector, 2> acc_non_bias_w_; /* the acceleration without bias in world frame for estimate_mode and expriment_mode */
    /* acc bias */
    KDL::Vector acc_bias_b_; /* the acceleration bias in baselink frame, only use z axis  */
    std::array<KDL::Vector, 2> acc_bias_w_; /* the acceleration bias in world frame for estimate_mode and expriment_mode*/

    aerial_robot_msgs::msg::States states_; /* for debug */

    virtual void imuCallback(const spinal::msg::Imu::SharedPtr msg);
    virtual void estimateProcess() override;

    void updateAcc();
    bool calibrateAcc();
    void setRotationalStates();
    void setTranslationalStates();

    void activateFuser() override;
    void fuse() override;

    void publish() override;
    void rosParamInit() override;
  };
};





