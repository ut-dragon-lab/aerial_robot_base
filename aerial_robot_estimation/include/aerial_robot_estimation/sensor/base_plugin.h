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

/* common utility */
#include <iostream>

/* aerial_robot_core API */
#include <rclcpp/rclcpp.hpp>
#include <aerial_robot_estimation/state_estimation.h>
#include <kalman_filter/kf_base_plugin.h>
#include <kalman_filter/lpf_filter.h>

/* ros API */
#include <tf2_kdl/tf2_kdl.hpp>

/* ros messages */
#include <aerial_robot_msgs/msg/states.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>


using RobotModelPtr = std::shared_ptr<aerial_robot_model::RobotModel>;
using EstimatorPtr = std::shared_ptr<aerial_robot_estimation::StateEstimator>;

namespace Status {
  enum {INACTIVE = 0, INIT = 1, ACTIVE = 2, INVALID = 3, RESET = 4};
}

namespace sensor_plugin {
  class SensorBase {
  public:
    SensorBase(): sensor_hz_(0),
                  prev_time_stamp_(-1),
                  get_sensor_rel_pose_(false),
                  logger_(rclcpp::get_logger("")) {
      sensor_status_ = Status::INACTIVE;
      sensor_rel_pose_ == KDL::Frame::Identity();
    }

    virtual void initialize(rclcpp::Node::SharedPtr node,
                            RobotModelPtr robot_model,
                            EstimatorPtr estimator,
                            string sensor_name, int index) {
      estimator_ = estimator;
      robot_model_ = robot_model;

      node_ = node;

      sensor_name_ = sensor_name.substr(sensor_name.rfind("/") + 1);
      logger_ = rclcpp::get_logger(sensor_name_ + std::to_string(index));
      sensor_index_ = index;

      health_.resize(1, false);
      health_stamp_.resize(1, node_->get_clock()->now().seconds());

      state_pub_ = node_->create_publisher<aerial_robot_msgs::msg::States>("kf/" + sensor_name_ + std::to_string(sensor_index_) + "/data", rclcpp::SystemDefaultsQoS());

      set_status_service_
        = node_->create_service<std_srvs::srv::SetBool>
        (sensor_name_ + std::to_string(sensor_index_) +"/estimate_flag",
         std::bind(&SensorBase::setStatusCb, this,
                   std::placeholders::_1,
                   std::placeholders::_2,
                   std::placeholders::_3));
      reset_service_
        = node_->create_service<std_srvs::srv::Empty>
        (sensor_name_ + std::to_string(sensor_index_) +"/reset",
         std::bind(&SensorBase::resetCb, this,
                   std::placeholders::_1,
                   std::placeholders::_2,
                   std::placeholders::_3));

      RCLCPP_INFO_STREAM(logger_, "load sensor plugin: "
                         << sensor_name_ + std::to_string(sensor_index_));

      std::string prefix = "estimation.fusion.sensor_plugin." + sensor_name_ ;
      if (!node_->get_parameter(prefix + ".estimate_mode", estimate_mode_) &&
          !node_->get_parameter(prefix + std::to_string(sensor_index_)
                                + ".estimate_mode",
                                estimate_mode_)) {
            RCLCPP_ERROR_STREAM(logger_,
                                "Can not get param about estimate mode");
      }

      /* general parameters for the same sensor type */
      getParam<bool>("param_verbose", param_verbose_, false);
      getParam<bool>("debug_verbose", debug_verbose_, false);
      getParam<std::string>("sensor_frame", sensor_frame_, "sensor_frame");
      getParam<bool>("sensor_pose_vary_flag", sensor_pose_vary_flag_, false);
      getParam<double>("reset_duration", reset_duration_, 1.0);
      getParam<double>("health_timeout", health_timeout_, 0.5);
      getParam<int>("unhealth_level", unhealth_level_, 0);
      getParam<bool>("time_sync", time_sync_, false);
      getParam<double>("delay", delay_, 0.0);
    }

    virtual ~SensorBase() {}

    inline const std::string& getSensorName() const {
      return sensor_name_;
    }

    const rclcpp::Time getTimeStamp() {
      return time_stamp_;
    }

    const int getStatus() {
      std::lock_guard<std::mutex> lock(status_mutex_);
      return sensor_status_;
    }

    void setStatus(const int status) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      prev_status_ = sensor_status_;
      sensor_status_ = status;
    }

    virtual bool reset(){
      return true;
    }

    virtual void changeStatus(bool flag) {
      if(sensor_status_ == Status::INVALID && flag) {
          sensor_status_ = Status::INACTIVE;
          RCLCPP_INFO_STREAM(logger_, "Set to inactive");
      }
      if(!flag) {
          sensor_status_ = Status::INVALID;
          RCLCPP_INFO_STREAM(logger_, "Set to invalid");
      }
    }

    /* check whether we get sensor data */
    void healthCheck() {
      std::lock_guard<std::mutex> lock(health_check_mutex_);

      /* this will call only once, no recovery */
      for(int i = 0; i < health_.size(); i++)
        {
          double st = node_->get_clock()->now().seconds();
          if(st - health_stamp_.at(i) > health_timeout_ && health_.at(i)) //  && !simulation_
            {
              RCLCPP_ERROR(logger_, "[chan%d]: can not get fresh sensor data for %f[sec]", i, st - health_stamp_.at(i));
              /* TODO: the solution to unhealth should be more clever */
              estimator_->setUnhealthLevel(unhealth_level_);

              health_.at(i) = false;
            }
        }
    }

  protected:

    /* node handle */
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;

    /* publisher */
    rclcpp::Publisher<aerial_robot_msgs::msg::States>::SharedPtr state_pub_;

    /* service */
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_status_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

    /* instance */
    RobotModelPtr robot_model_;
    EstimatorPtr estimator_;

    /* reconfigurable varaible */
    int estimate_mode_;

    bool param_verbose_;
    bool debug_verbose_;

    string sensor_name_;
    int sensor_index_;

    bool get_sensor_rel_pose_;
    bool sensor_pose_vary_flag_;

    string sensor_frame_;

    rclcpp::Time time_stamp_, prev_time_stamp_;
    bool time_sync_;
    double delay_;
    double curr_timestamp_;
    double prev_timestamp_;
    double sensor_hz_; // hz  of the sensor
    std::vector<int> estimate_indices_; // the fuser_egomation index
    std::vector<int> experiment_indices_; // the fuser_experiment indices

    /* the transformation between sensor frame and baselink frame */
    KDL::Frame sensor_rel_pose_;

    /* status */
    int sensor_status_;
    int prev_status_;
    std::mutex status_mutex_;
    std::mutex health_check_mutex_;

    /* health check */
    double reset_stamp_;
    double reset_duration_;
    std::vector<bool> health_;
    std::vector<double> health_stamp_;
    double health_timeout_;
    int unhealth_level_;


    inline const bool isModeActivate(uint8_t mode) const {
      return (estimate_mode_ & (1 << mode));
    }

    virtual void estimateProcess() {}
    virtual void activateFuser() {}
    virtual void fuse() {}
    virtual void preProcessState() {}
    virtual void setState() {}

    virtual void publish() {}
    virtual void rosParamInit() {}


    bool resetCb(const std::shared_ptr<rmw_request_id_t> /*req_id*/,
                 const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                 std::shared_ptr<std_srvs::srv::Empty::Response> res) {
      RCLCPP_INFO_STREAM(logger_, "reset sensor plugin " << sensor_name_ <<
                         " from rosservice server");
      reset();
      return true;
    }

    bool setStatusCb(const std::shared_ptr<rmw_request_id_t> /*req_id*/,
                     const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      changeStatus(req->data);

      return true;
    }

    void setHealthChanNum(const uint8_t& chan_num) {
      assert(chan_num > 0);

      health_.resize(chan_num, false);
      health_stamp_.resize(chan_num, node_->get_clock()->now().seconds());
    }

    void updateHealthStamp(uint8_t chan = 0) {
      std::lock_guard<std::mutex> lock(health_check_mutex_);

      double st = node_->get_clock()->now().seconds();
      if(!health_[chan])
        {
          health_[chan] = true;
          RCLCPP_INFO(logger_, "Get sensor data, du: %f", st - health_stamp_[chan]);
        }
      health_stamp_[chan] = st;
    }

    inline const KDL::Frame& getBase2SensorTF() const {
      return sensor_rel_pose_;
    }

    bool updateBase2SensorTF() {
      /* get transform from baselink to sensor frame */
      if(!sensor_pose_vary_flag_ && get_sensor_rel_pose_) return true;

      /*
        for joint or servo system, this should be processed every time,
        therefore kinematics based on kinematics is better, since the tf need 0.x[sec].
      */
      const auto segments_tf =  robot_model_->getSegmentsTf();

      if(segments_tf.empty()) {
        if(get_sensor_rel_pose_) {
          RCLCPP_ERROR(logger_, "The segment tf is empty after init phase");
        }

          return false;
        }

      if(segments_tf.find(sensor_frame_) == segments_tf.end()) {
        RCLCPP_ERROR_STREAM_THROTTLE(logger_, *node_->get_clock(), 0.5,
                                     "Can not find " << sensor_frame_
                                     << " in kinematics model");
        return false;
      }

      try {
        sensor_rel_pose_
          = segments_tf.at(robot_model_->getBaselinkName()).Inverse()
          * segments_tf.at(sensor_frame_);
      } catch (...) {
        RCLCPP_ERROR_STREAM(logger_, "Can not find " << sensor_frame_
                            << " in spite of segments_tf.find is true");
      }

      double y, p, r; sensor_rel_pose_.M.GetRPY(r, p, y);
      if(!sensor_pose_vary_flag_) {
        RCLCPP_INFO_STREAM(logger_, "Get TF from" <<
                           robot_model_->getBaselinkName() << " to " <<
                           sensor_frame_ << " pos: [" <<
                           sensor_rel_pose_.p.x() << ", " <<
                           sensor_rel_pose_.p.y() << ", " <<
                           sensor_rel_pose_.p.z() << "], rot: [" <<
                           r << ", " << p << ", " << y << "]");

      }

      get_sensor_rel_pose_ = true;
      return true;
    }

    template<class T> void getParam(std::string param_name, T& param, T default_value) {

      std::string prefix = "estimation.fusion.sensor_plugin." + sensor_name_ ;
      node_->get_parameter_or<T>(prefix + "." + param_name, param, default_value);
      node_->get_parameter<T>(prefix + std::to_string(sensor_index_) + "." + param_name,
                              param);


      if(param_verbose_) {
        RCLCPP_INFO_STREAM(logger_, param_name << ": " << param);
      }
    }
  };

};
