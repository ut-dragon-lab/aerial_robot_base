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

/* base class */
#include <aerial_robot_estimation/sensor/odom.h>

using namespace aerial_robot_estimation;

namespace sensor_plugin
{
  Odometry::Odometry():
    sensor_plugin::SensorBase() {
    states_.states.resize(3);
    states_.states[0].id = "x";
    states_.states[0].state.resize(2);
    states_.states[1].id = "y";
    states_.states[1].state.resize(2);
    states_.states[2].id = "z";
    states_.states[2].state.resize(2);


    sensor_pose_ = KDL::Frame::Identity();
    prev_sensor_pose_ = KDL::Frame::Identity();
    sensor_twist_ = KDL::Twist::Zero();
    prev_sensor_twist_ = KDL::Twist::Zero();

    origin_offset_ = KDL::Frame::Identity();
    base_pose_ = KDL::Frame::Identity();
  }

  void Odometry::initialize(rclcpp::Node::SharedPtr node,
                            RobotModelPtr robot_model,
                            EstimatorPtr estimator,
                            string sensor_name, int index) {

    SensorBase::initialize(node, robot_model, estimator, sensor_name, index);

    string topic_name;
    getParam<std::string>("odom_sub", topic_name, "raw_odom");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>
      (topic_name, rclcpp::SystemDefaultsQoS(),
       std::bind(&Odometry::odomCallback, this, std::placeholders::_1));
    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    rosParamInit();
  }

  void Odometry::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    /* check the sensor value whether valid */
    if (isMsgNan(*msg)) {
      RCLCPP_ERROR(logger_, "odometry sensor publishes NaN value!");
      return;
    }

    tf2::fromMsg(msg->pose.pose, sensor_pose_);
    tf2::fromMsg(msg->twist.twist, sensor_twist_);

    time_stamp_ = msg->header.stamp;
    if (odom_origin_frame_.empty()) odom_origin_frame_ = msg->header.frame_id;

    estimateProcess();

    prev_time_stamp_ = time_stamp_;
    prev_sensor_pose_ = sensor_pose_;
    prev_sensor_twist_ = sensor_twist_;
  }

  void Odometry::estimateProcess()
  {

    /* update and check status */
    if (!checkStatus()) return;

    /* update the sensor tf w.r.t baselink */
    if(!updateBase2SensorTF()) return;

    /* preprocess */
    preProcessState();

    /* do fusion */
    fuse();

    /* publish */
    publish();

    /* update health state */
    updateHealthStamp();
  }

  bool Odometry::checkStatus() {
    /* only do egmotion estimate mode */
    if(!isModeActivate(EGOMOTION_ESTIMATE)) {
      RCLCPP_WARN_THROTTLE
        (logger_, *node_->get_clock(), 1.0,
         "Odometry Plugin: no egmotion estimate mode !");

      return false;
    }

    /* check whether is force att control mode */
    if(estimator_->getForceAttControlFlag() && getStatus() == Status::ACTIVE) {
      setStatus(Status::INVALID);
    }

    if(getStatus() == Status::INVALID || getStatus() == Status::RESET) {
      return false;
    }

    return true;
  }

  void Odometry::preProcessState() {

    /* throttle message */
    if(throttle_rate_ > 0) {
      if (time_stamp_ - prev_time_stamp_
          < rclcpp::Duration::from_seconds(1 / throttle_rate_)) {
          return;
        }
    }

    /* timestamp for fusion (consider the delay for timestamp) */
    ref_time_stamp_ = time_stamp_.seconds() + delay_;


    if(getStatus() == Status::INACTIVE) {

      // YAW (wx_b) update
      if (!estimator_->hasRefinedYawEstimate(EGOMOTION_ESTIMATE)) {
        RCLCPP_INFO(logger_, " refined yaw estimate becomes true");
        estimator_->SetRefinedYawEstimate(EGOMOTION_ESTIMATE, true);
      }

      if(!calcuateOriginOffset()) return;

      activateFuser();
      setStatus(Status::ACTIVE);
    }

    // get baselink pose
    calculateBasePose();

    // get baselink velocity
    calculateBaseVelocity();

    // debug print
    print();
  }

  bool Odometry::calcuateOriginOffset() {

    bool imu_initialized = false;
    for(const auto& handler: estimator_->getImuHandlers()) {
      if(handler->getStatus() == Status::ACTIVE) {
        imu_initialized = true;
        break;
      }
    }

    if(!imu_initialized) {

      RCLCPP_WARN_THROTTLE
        (logger_, *node_->get_clock(), 1.0,
         "Odometry Plugin: no imu is initialized, wait !");

      return false;
    }

    /** step1: ^{w}H_{b} **/
    KDL::Frame base_pose_w;
    base_pose_w.M = estimator_->getBaseOrientation(EGOMOTION_ESTIMATE);

    KDL::Vector pos = estimator_->getBasePos(EGOMOTION_ESTIMATE);
    if(estimator_->getBasePosStateStatus(State::X, EGOMOTION_ESTIMATE)) {
      base_pose_w.p.x(pos.x());
    }
    if(estimator_->getBasePosStateStatus(State::Y, EGOMOTION_ESTIMATE)) {
      base_pose_w.p.y(pos.x());
    }
    if(estimator_->getBasePosStateStatus(State::Z, EGOMOTION_ESTIMATE)) {
      base_pose_w.p.z(pos.x());
    }

    /* set the offset if we know the ground truth */
    if(estimator_->getBaseRotStateStatus(GROUND_TRUTH)) {
      base_pose_w = estimator_->getBasePose(GROUND_TRUTH);
    }

    /** step2: ^{o}H_{b} **/
    KDL::Frame base_pose_o = sensor_pose_ * sensor_rel_pose_.Inverse(); 

    /** step3: ^{w}H_{o} = ^{w}H_{b} * ^{b}H_{o} **/
    origin_offset_ = base_pose_w * base_pose_o.Inverse();

    tfBroadcast();

    return true;
  }

  void Odometry::calculateBasePose() {

    base_pose_ = origin_offset_ * sensor_pose_ * sensor_rel_pose_.Inverse();

    // set orientation state
    // only update the wx_b vector (the vector only related to yaw)
    KDL::Frame c2b_tf = robot_model_->getCog2Baselink<KDL::Frame>();

    KDL::Vector wx_b = base_pose_.M.Inverse().UnitX();
    KDL::Vector wx_c = c2b_tf.M * wx_b;

    estimator_->setBaseOrientationWxB(EXPERIMENT_ESTIMATE, wx_b);
    estimator_->setCogOrientationWxB(EXPERIMENT_ESTIMATE, wx_c);
  }

  void Odometry::calculateBaseVelocity() {

    /* get the latest orientation and omega */
    KDL::Rotation rot_b = estimator_->getBaseOrientation(EGOMOTION_ESTIMATE);
    KDL::Vector omega_b = estimator_->getBaseAngularVel(EGOMOTION_ESTIMATE);

    if (sensor_twist_ == KDL::Twist::Zero()) {

      if (fusion_mode_ != ONLY_POS_MODE) {

        RCLCPP_WARN(logger_, "force change fuse mode to ONLY_POS_MODE since velocity is invalid");
        fusion_mode_ = ONLY_POS_MODE;
      }
    }

    if (time_sync_) {

      int mode = EGOMOTION_ESTIMATE;

      // update rot_b, omega_b
      if(!estimator_->findBaseRotOmega
         (ref_time_stamp_, mode, rot_b, omega_b)) {
        ref_time_stamp_ = estimator_->getImuLatestTimeStamp();
      }
    }

    base_twist_.vel = origin_offset_.M * sensor_twist_.vel;
    if (local_vel_mode_) {
      // if the velocity is described in local frame (i.e., the sensor frame),
      // we need to convert to global one
      base_twist_.vel = rot_b * sensor_rel_pose_.M * sensor_twist_.vel;
    }

    // consider the offset between baselink and sensor frames
    base_twist_.vel -= rot_b * (omega_b * sensor_rel_pose_.p);
  }

  void Odometry::activateFuser() {

    KDL::Vector init_pos = base_pose_.p;

    for(auto& fuser : estimator_->getFuserList(EGOMOTION_ESTIMATE)) {

      string plugin_name = fuser.first;
      FuserPtr kf = fuser.second;
      int id = kf->getId();

      if(plugin_name == "kalman_filter/kf_pos_vel_acc") {

        int index;
        if(id & (1 << State::X)) index = 0;
        else if(id & (1 << State::Y)) index = 1;
        else if(id & (1 << State::Z)) index = 2;
        else index = -1;

        if (index < 0) continue; // only consider x,y,z

        /* not need to initialize */
        if(estimator_->getBasePosStateStatus(index, EGOMOTION_ESTIMATE)) {
          continue;
        }

        if(fusion_mode_ != ONLY_VEL_MODE){
          kf->setInitState(init_pos[index], 0);
        }

        kf->setMeasureFlag();

        estimator_->setBasePosStateStatus(index, EGOMOTION_ESTIMATE, true);
      }
    }
  }

  void Odometry::fuse() {

    if (getStatus() != Status::ACTIVE) return;

    for(auto& fuser : estimator_->getFuserList(EGOMOTION_ESTIMATE)) {

      string plugin_name = fuser.first;
      FuserPtr kf = fuser.second;

      if(!kf->getFilteringFlag()) continue;

      int id = kf->getId();
      double timestamp = ref_time_stamp_;
      double outlier_thresh = (fusion_mode_ == ONLY_VEL_MODE)?
        (vel_outlier_thresh_ / vel_noise_sigma_ / vel_noise_sigma_):0;

      if(plugin_name == "kalman_filter/kf_pos_vel_acc") {

        int index;
        if(id & (1 << State::X)) index = 0;
        else if(id & (1 << State::Y)) index = 1;
        else if(id & (1 << State::Z)) index = 2;
        else index = -1;

        if (index < 0) continue; // only consider x,y,z

        vector<double> params;

        /* correction */
        if (fusion_mode_ == ONLY_POS_MODE) {
          VectorXd meas(1);
          VectorXd sigma(1);
          sigma << level_pos_noise_sigma_; // xy by default
          meas << base_pose_.p[index];
          params = {kf_plugin::POS};

          if(index == State::Z) {
            sigma << z_pos_noise_sigma_;
          }

          kf->correction(meas, sigma,
                         time_sync_?(timestamp):-1,
                         params, outlier_thresh);
        } else if(fusion_mode_ == ONLY_VEL_MODE) {
          VectorXd sigma(1);
          sigma << vel_noise_sigma_;

          VectorXd meas(1);
          meas << base_twist_.vel[index];
          params = {kf_plugin::VEL};

          kf->correction(meas, sigma,
                         time_sync_?(timestamp):-1,
                         params, outlier_thresh);
        } else if(fusion_mode_ == POS_VEL_MODE) {

          VectorXd meas(2);
          meas << base_pose_.p[index], base_twist_.vel[index];
          params = {kf_plugin::POS_VEL};

          VectorXd sigma(2);
          if(index == State::Z) {
            sigma << z_pos_noise_sigma_, vel_noise_sigma_;
          } else {
            sigma << level_pos_noise_sigma_, vel_noise_sigma_;
          }

          kf->correction(meas, sigma,
                         time_sync_?(timestamp):-1,
                         params, outlier_thresh);
        } else {
          RCLCPP_WARN(logger_, "wrong fusion model %d", fusion_mode_);
        }

      }

    }

  }

  const bool Odometry::odomPosMode() {
    if(fusion_mode_ != ONLY_VEL_MODE) {
      return true;
    } else {
      return false;
    }
  }

  bool Odometry::isMsgNan(nav_msgs::msg::Odometry msg) {

    if(std::isnan(msg.pose.pose.position.x) ||
       std::isnan(msg.pose.pose.position.y) ||
       std::isnan(msg.pose.pose.position.z) ||
       std::isnan(msg.pose.pose.orientation.x) ||
       std::isnan(msg.pose.pose.orientation.y) ||
       std::isnan(msg.pose.pose.orientation.z) ||
       std::isnan(msg.pose.pose.orientation.w) ||
       std::isnan(msg.twist.twist.linear.x) ||
       std::isnan(msg.twist.twist.linear.y) ||
       std::isnan(msg.twist.twist.linear.z) ||
       std::isnan(msg.twist.twist.angular.x) ||
       std::isnan(msg.twist.twist.angular.y) ||
       std::isnan(msg.twist.twist.angular.z)) {

      return true;
    }

    return false;
  }


  void Odometry::print() {

    if(!debug_verbose_) return;

    double y, p, r;
    sensor_pose_.M.GetRPY(r, p, y);
    KDL::Vector pos = sensor_pose_.p;
    RCLCPP_INFO(logger_, "odometry sensor pos: [%f, %f, %f], raw rot: [%f, %f, %f]",
                pos.x(), pos.y(), pos.z(), r, p, y);
    KDL::Vector gt_pos = estimator_->getBasePos(GROUND_TRUTH);
    KDL::Vector base_pos = base_pose_.p;
    RCLCPP_INFO(logger_,
                "ground truth base pos: [%f, %f, %f], estimate pos: [%f, %f, %f]",
                gt_pos.x(), gt_pos.y(), gt_pos.z(),
                base_pos.x(), base_pos.y(), base_pos.z());


    double gt_r, gt_p, gt_y;
    estimator_->getBaseOrientation(GROUND_TRUTH).GetRPY(gt_r, gt_p, gt_y);
    base_pose_.M.GetRPY(r, p, y);
    RCLCPP_INFO(logger_,
                "ground truth base yaw: %f, estimate rpy: [%f, %f, %f]",
                gt_y, r, p, y);
  }

  void Odometry::publish() {

    states_.header.stamp = time_stamp_;

    for(int axis = 0; axis < 3; axis++) {
      states_.states[axis].state[0].pos = base_pose_.p[axis]; // raw base pos
      states_.states[axis].state[0].vel = base_twist_.vel[axis]; // raw base vel
      states_.states[axis].state[1].pos = sensor_pose_.p[axis]; // raw sensor pos
      states_.states[axis].state[1].vel = sensor_twist_.vel[axis]; // raw base vel
    }

    state_pub_->publish(states_);
  }

  void Odometry::tfBroadcast() {
    /* publish the offset between origins of the world frame and the odom frame */
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg = aerial_robot_model::kdlToMsg(origin_offset_);
    tf_msg.header.stamp = time_stamp_;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = odom_origin_frame_;
    br_->sendTransform(tf_msg);
  }

  void Odometry::rosParamInit() {
    getParam<int>("fusion_mode", fusion_mode_, (int)ONLY_POS_MODE);
    getParam<bool>("local_vel_mode", local_vel_mode_, true);
    getParam<double>("throttle_rate", throttle_rate_, 0.0);
    getParam<double>("level_pos_noise_sigma", level_pos_noise_sigma_, 0.01 );
    getParam<double>("z_pos_noise_sigma", z_pos_noise_sigma_, 0.01 );
    getParam<double>("vel_noise_sigma", vel_noise_sigma_, 0.05 );
    getParam<double>("vel_outlier_thresh", vel_outlier_thresh_, 1.0);
  }

};

/* plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Odometry, sensor_plugin::SensorBase);



