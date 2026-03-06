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

#include <aerial_robot_estimation/sensor/imu.h>

using namespace aerial_robot_estimation;

namespace sensor_plugin {
  Imu::Imu ():
    sensor_plugin::SensorBase(),
    calib_count_(200), dt_(0) {

    states_.states.resize(3);
    states_.states[0].id = "x";
    states_.states[0].state.resize(2);
    states_.states[1].id = "y";
    states_.states[1].state.resize(2);
    states_.states[2].id = "z";
    states_.states[2].state.resize(2);

    omega_ = KDL::Vector::Zero();
    mag_   = KDL::Vector::Zero();
    acc_b_ = KDL::Vector::Zero();
    acc_bias_b_ = KDL::Vector::Zero();

    for (int i = 0; i++; i < 2) {
      acc_w_.at(0) = KDL::Vector::Zero();
      acc_non_bias_w_.at(0) = KDL::Vector::Zero();
      acc_bias_w_.at(0) = KDL::Vector::Zero();
    }

    raw_rot_ = KDL::Rotation::Identity();
  }

  void Imu::initialize(rclcpp::Node::SharedPtr node,
                       RobotModelPtr robot_model,
                       EstimatorPtr estimator,
                       string sensor_name, int index) {
    SensorBase::initialize(node, robot_model, estimator, sensor_name, index);
    rosParamInit();

    std::string topic_name;
    getParam<string>("imu_topic_name", topic_name, "imu");
    imu_sub_ = node_->create_subscription<spinal::msg::Imu>
      (topic_name, rclcpp::SystemDefaultsQoS(),
       std::bind(&Imu::imuCallback, this, std::placeholders::_1));

    topic_name = sensor_name + "/" + std::to_string(index) + "/ros/imu";
    ros_imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>
      (topic_name, rclcpp::SystemDefaultsQoS());

    topic_name = sensor_name + "/" + std::to_string(index) + "/states";
    state_pub_ = node_->create_publisher<aerial_robot_msgs::msg::States>
      (topic_name, rclcpp::SystemDefaultsQoS());
  }

  void Imu::imuCallback(const spinal::msg::Imu::SharedPtr msg) {
    // check the validity first
    for(int i = 0; i < 3; i++) {
      if(isnan(msg->acc[i]) || isnan(msg->gyro[i]) || isnan(msg->mag[i])) {
        RCLCPP_ERROR_THROTTLE
          (logger_, *node_->get_clock(), 1.0,
           "IMU plugin receives Nan value in IMU sensors !");
        return;
      }
    }

    geometry_msgs::msg::Quaternion q;
    q.x = msg->quaternion[0];
    q.y = msg->quaternion[1];
    q.z = msg->quaternion[2];
    q.w = msg->quaternion[3];

    if(isnan(q.x) || isnan(q.y) || isnan(q.z) || isnan(q.w)) {
      RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1.0,
                            "IMU plugin receives Nan value in Quaternion!");
      return;
    }

    /* substitude */
    tf2::fromMsg(msg->acc,  acc_b_);
    tf2::fromMsg(msg->gyro, omega_);
    tf2::fromMsg(msg->mag,  mag_);
    tf2::fromMsg(q, raw_rot_);

    /* main process */
    time_stamp_ = msg->stamp;
    estimateProcess();
    prev_time_stamp_ = time_stamp_;
  }

  void Imu::estimateProcess() {

    /* skip if the time stamp is ill */
    if(time_stamp_.seconds() <= prev_time_stamp_.seconds()) {
      RCLCPP_WARN(logger_,
                  "IMU: bad timestamp. curr time stamp: %f, prev time stamp: %f",
                  time_stamp_.seconds(), prev_time_stamp_.seconds());
      return;
    }

    /* set the time internal */
    dt_ = time_stamp_.seconds() - prev_time_stamp_.seconds();


    /* set the rotational states */
    /* only base on raw IMU value */
    setRotationalStates();

    /* update accleration */
    updateAcc();

    /* do fusion */
    fuse();

    /* set the states */
    setTranslationalStates();

    /* publish topics */
    publish();

    /* update health state */
    updateHealthStamp();
  }

  void Imu::updateAcc() {

    /* acc calibration */
    if (!calibrateAcc()) return;


    KDL::Vector g = KDL::Vector(0, 0, aerial_robot_estimation::G);

    for (int i = 0; i < 2; i++) {

      KDL::Rotation rot = estimator_->getBaseOrientation(i);

      acc_w_.at(i) = rot * acc_b_ - g;
      acc_non_bias_w_.at(i) = acc_w_.at(i) - acc_bias_w_.at(i);
    }
  }

  bool Imu::calibrateAcc() {

    if(calib_count_ == calib_max_count_) return true;

    calib_count_ ++;

    if(calib_count_ == 100) {
      // warm up for callback to be stable subscribe
      calib_max_count_ = calib_time_ / dt_;
      RCLCPP_INFO_STREAM(logger_,
                         string("\033[32m IMU calib max count is : ") <<
                         calib_max_count_ << string("\033[0m"));

      setStatus(Status::INIT); // start init
    }

    /* accmumate acc bias */
    for (int i = 0; i < 2; i++) {
      acc_bias_w_.at(i) += acc_w_.at(i);
    }

    if(calib_count_ < calib_max_count_) return false;


    /* calib_count_ == calib_max_count_ */
    for (int i = 0; i < 2; i++) {
      acc_bias_w_.at(i) = acc_bias_w_.at(i) / calib_max_count_;
    }

    KDL::Rotation rot_inv =
      estimator_->getBaseOrientation(EGOMOTION_ESTIMATE).Inverse();
    KDL::Vector acc_bias_b = rot_inv * acc_bias_w_.at(EGOMOTION_ESTIMATE);
    RCLCPP_INFO_STREAM
      (logger_,
       string("\033[32m IMU acc bias w.r.t body frame: [") <<
       acc_bias_b.x() << ", " <<
       acc_bias_b.y() << ", " <<
       acc_bias_b.z() << ", dt: " << dt_ <<
       string("\033[0m"));

    estimator_->setBaseQueueSize(1 / dt_);

    activateFuser();

    return true;
  }


  void Imu::activateFuser() {

    setStatus(Status::ACTIVE);

    /* fuser for 0: egomotion, 1: experiment */
    for(int mode = 0; mode < 2; mode++) {
      if(!isModeActivate(mode)) continue;

      for(auto& fuser : estimator_->getFuserMap(mode)) {
        std::string plugin_name = fuser.first;
        FuserPtr kf = fuser.second;
        int id = kf->getId();

        if(plugin_name == "kalman_filter/kf_pos_vel_acc") {

          Eigen::VectorXd input_noise_sigma(2);
          if((id & (1 << State::X)) || (id & (1 << State::Y))) {
            input_noise_sigma
              << level_acc_noise_sigma_, level_acc_bias_noise_sigma_;

            kf->setPredictionNoiseCovariance(input_noise_sigma);

            if(level_acc_bias_noise_sigma_ > 0) {
              if(id & (1 << State::X)) {
                kf->setInitState(acc_bias_w_.at(mode).x(),2);
              } else if(id & (1 << State::Y)) {
                kf->setInitState(acc_bias_w_.at(mode).y(),2);
              } else { }
            }
          }

          if(id & (1 << State::Z)) {
            input_noise_sigma << z_acc_noise_sigma_, z_acc_bias_noise_sigma_;
            kf->setPredictionNoiseCovariance(input_noise_sigma);

            if(z_acc_bias_noise_sigma_ > 0) {
              kf->setInitState(acc_bias_w_.at(mode).z(), 2);
            }
          }
        }

        //set the prediction handler buffer size
        kf->setPredictBufSize(1 / dt_);
        kf->setInputFlag();
      }
    }
  }

  void Imu::fuse() {

    /* fuser for 0: egomotion, 1: experiment */
    for(int mode = 0; mode < 2; mode++)
      {
        if(!isModeActivate(mode)) continue;

        for(auto& fuser : estimator_->getFuserMap(mode))
          {
            string plugin_name = fuser.first;
            FuserPtr kf = fuser.second;
            int id = kf->getId();
            vector<double> params = {dt_};

            int axis;
            bool predict = false;

            if(plugin_name == "kalman_filter/kf_pos_vel_acc") {
              VectorXd input_val(1);

              if(id & (1 << State::X)) {

                input_val << ((level_acc_bias_noise_sigma_ > 0)?
                              acc_w_.at(mode).x():
                              acc_non_bias_w_.at(mode).x());

                axis = State::X;

              } else if(id & (1 << State::Y)) {

                input_val << ((level_acc_bias_noise_sigma_ > 0)?
                              acc_w_.at(mode).y():
                              acc_non_bias_w_.at(mode).y());

                axis = State::Y;
              } else if(id & (1 << State::Z)) {

                input_val << ((z_acc_bias_noise_sigma_ > 0)?
                              acc_w_.at(mode).z():
                              acc_non_bias_w_.at(mode).z());

                axis = State::Z;

                /* considering the undescend mode,
                   such as the phase of takeoff,
                   the velocity should not below than 0 */
                if(estimator_->getUnDescendMode()
                   && (kf->getEstimateState())(1) < 0) {
                  kf->resetState();
                }
              } else {}

              /* do kalman filter prediction */
              kf->prediction(input_val, time_stamp_.seconds(), params);
            }
          }
      }
  }

  void Imu::setRotationalStates() {

    KDL::Vector wx_b = raw_rot_.Inverse().UnitX();
    KDL::Vector wy_b = raw_rot_.Inverse().UnitY();
    KDL::Vector wz_b = raw_rot_.Inverse().UnitZ();

    KDL::Rotation c2b_rot = robot_model_->getCog2Baselink<KDL::Frame>().M;
    KDL::Vector wz_c = c2b_rot * wz_b;
    KDL::Vector omega_c = c2b_rot * omega_;

    for (int i = 0; i < 2; i++) {
      /*
        check if there is a refined (better)
        yaw estimation handler (e.g. VO, RTK-GPS)
      */
      if (estimator_->hasRefinedYawEstimate(i)) {
        // only update wz_b vector (the vector only related to gravity)
        estimator_->setBaseOrientationWzB(i, wz_b);
        estimator_->setCogOrientationWzB(i, wz_c);
      } else {
        estimator_->setBaseOrientation(i, raw_rot_);
        KDL::Rotation rot_c = raw_rot_ * c2b_rot.Inverse();
        estimator_->setCogOrientation(i, rot_c);
      }

      // set angular velocity
      estimator_->setBaseAngularVel(i, omega_);
      estimator_->setCogAngularVel(i, omega_c);
    }

    // mode for GROUND_TRUTH
    if (!estimator_->hasGroundTruthOdom()) {
      // the orientation is set from ground truth plugin (mocap)

      /* set baselink angular velocity for all axes using imu omega */
      estimator_->setBaseAngularVel(GROUND_TRUTH, omega_);
      /* set cog angular velocity for all axes using imu omega */
      estimator_->setCogAngularVel(GROUND_TRUTH, omega_c);
    }

    /*
      set the rotation and angular velocity
      for the temporal queue for other sensor with time delay
    */
    estimator_->updateBaseQueue(time_stamp_.seconds(),
                                estimator_->getBaseOrientation(EGOMOTION_ESTIMATE),
                                estimator_->getBaseOrientation(EXPERIMENT_ESTIMATE),
                                omega_);

  }

  void Imu::setTranslationalStates() {

    /* fuser for 0: egomotion, 1: experiment */
    for(int mode = 0; mode < 2; mode++) {
      if(!isModeActivate(mode)) continue;

      KDL::Vector pos, vel, acc;
      for(auto& fuser : estimator_->getFuserMap(mode))
        {
          string plugin_name = fuser.first;
          FuserPtr kf = fuser.second;
          int id = kf->getId();

          if(plugin_name == "kalman_filter/kf_pos_vel_acc") {

            /* set the state */
            VectorXd estimate_state = kf->getEstimateState();

            if(id & (1 << State::X)) {
              pos.x(estimate_state(0));
              vel.x(estimate_state(1));

              if(level_acc_bias_noise_sigma_ > 0) {
                acc_bias_w_.at(mode).x(kf->getEstimateState()(2));
                acc.x(acc_w_.at(mode).x() - acc_bias_w_.at(mode).x());
              } else {
                acc.x(acc_non_bias_w_.at(mode).x());
              }

            } else if(id & (1 << State::Y)) {
              pos.y(estimate_state(0));
              vel.y(estimate_state(1));

              /* get the estiamted offset(bias) */
              if(level_acc_bias_noise_sigma_ > 0) {
                acc_bias_w_.at(mode).y(kf->getEstimateState()(2));
                acc.y(acc_w_.at(mode).y() - acc_bias_w_.at(mode).y());
              } else {
                acc.y(acc_non_bias_w_.at(mode).y());
              }

            } else if(id & (1 << State::Z)) {

              pos.z(estimate_state(0));
              vel.z(estimate_state(1));

              /* get the estiamted offset(bias) */
              if(z_acc_bias_noise_sigma_ > 0) {
                acc_bias_w_.at(mode).z(kf->getEstimateState()(2));
                acc.z(acc_w_.at(mode).z() - acc_bias_w_.at(mode).z());
              } else {
                acc.z(acc_non_bias_w_.at(mode).z());
              }

            } else {}
          }
        }

      /* set Baselink States */
      estimator_->setBasePos(mode, pos);
      estimator_->setBaseVel(mode, vel);
      estimator_->setBaseAcc(mode, acc);


      /* set CoG States based on Baselink States */
      /* TODO: the joint velocity */
      KDL::Rotation base_rot = estimator_->getBaseOrientation(mode);
      KDL::Vector base_omega = estimator_->getBaseAngularVel(mode);
      KDL::Vector pos_b2c
        = robot_model_->getCog2Baselink<KDL::Frame>().Inverse().p;

      KDL::Vector cog_pos = pos + base_rot * pos_b2c;
      KDL::Vector cog_vel = vel + base_rot * (base_omega * pos_b2c);
      KDL::Vector cog_acc = acc;

      estimator_->setCogPos(mode, pos);
      estimator_->setCogPos(mode, vel);
      estimator_->setCogPos(mode, acc);
    }
  }

  void Imu::publish() {
    /* ros IMU message */
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = time_stamp_;
    imu_msg.header.frame_id
      = estimator_->getTFPrefix() + "/" + robot_model_->getBaselinkName();
    auto& q = imu_msg.orientation;
    raw_rot_.GetQuaternion(q.x, q.y, q.z, q.w);
    imu_msg.angular_velocity.x = omega_.x();
    imu_msg.angular_velocity.y = omega_.y();
    imu_msg.angular_velocity.z = omega_.z();
    imu_msg.linear_acceleration.x = acc_b_.x();
    imu_msg.linear_acceleration.y = acc_b_.y();
    imu_msg.linear_acceleration.z = acc_b_.z();
    ros_imu_pub_->publish(imu_msg);


    /* publish state date */
    states_.header.stamp = time_stamp_;
    for (int i = 0; i < 2; i++) {
      KDL::Vector pos = estimator_->getBasePos(i);
      states_.states[0].state[i].pos = pos.x();
      states_.states[1].state[i].pos = pos.y();
      states_.states[2].state[i].pos = pos.z();

      KDL::Vector vel = estimator_->getBaseVel(i);
      states_.states[0].state[i].vel = vel.x();
      states_.states[1].state[i].vel = vel.y();
      states_.states[2].state[i].vel = vel.z();

      KDL::Vector acc = estimator_->getBaseAcc(i);
      states_.states[0].state[i].acc = acc.x();
      states_.states[1].state[i].acc = acc.y();
      states_.states[2].state[i].acc = acc.z();
    }
    state_pub_->publish(states_);

  }

  void Imu::rosParamInit()
  {
    getParam<double>("level_acc_noise_sigma", level_acc_noise_sigma_, 0.01);
    getParam<double>("z_acc_noise_sigma", z_acc_noise_sigma_, 0.01 );
    getParam<double>("level_acc_bias_noise_sigma", level_acc_bias_noise_sigma_, 0.01 );
    getParam<double>("z_acc_bias_noise_sigma", z_acc_bias_noise_sigma_, 0.0);
    getParam<double>("calib_time", calib_time_, 2.0);
  }
};

/* plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu, sensor_plugin::SensorBase);



