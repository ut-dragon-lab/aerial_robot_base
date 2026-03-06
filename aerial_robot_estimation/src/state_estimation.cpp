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

#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <aerial_robot_estimation/state_estimation.h>

using namespace aerial_robot_estimation;
static const rclcpp::Logger LOGGER
= rclcpp::get_logger("state_estimation");

StateEstimator::StateEstimator()
  : sensor_fusion_flag_(false),
    qu_size_(0),
    flying_flag_(false),
    un_descend_flag_(false),
    force_att_control_flag_(false),
    has_groundtruth_odom_(false),
    imu_handlers_(0), alt_handlers_(0), vo_handlers_(0), gps_handlers_(0),
    fusion_loader_("kalman_filter", "kf_plugin::KalmanFilter"),
    sensor_loader_("aerial_robot_estimation", "sensor_plugin::SensorBase")
{
  has_refined_yaw_estimate_[EGOMOTION_ESTIMATE] = false;
  has_refined_yaw_estimate_[EXPERIMENT_ESTIMATE] = false;

  for(int i = 0; i < 3; i++) {

    for(int j = 0; j < 3; j++) {
      base_pos_status_matrix_.at(i).at(j) = 0;
      cog_pos_status_matrix_.at(i).at(j) = 0;
    }
    base_rot_status_.at(i) = 0;
    cog_rot_status_.at(i) = 0;

    base_pose_.at(i) = KDL::Frame::Identity();
    cog_pose_.at(i) = KDL::Frame::Identity();
  }

  /* TODO: represented sensors unhealth level */
  unhealth_level_ = 0;

  prev_pub_stamp_ = node_->get_clock()->now();
}

void StateEstimator::initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model) {
  node_ = node;
  robot_model_ = robot_model;

  load();

  baselink_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>
    ("uav/baselink/odom", rclcpp::SystemDefaultsQoS());
  cog_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>
    ("uav/cog/odom", rclcpp::SystemDefaultsQoS());

  node_->get_parameter_or("tf_prefix", tf_prefix_, std::string(""));
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  double rate;
  node_->get_parameter_or("state_pub_rate", rate, 100.0);
  auto period = std::chrono::duration<double>(1.0 / rate);
  state_pub_timer_ = rclcpp::create_timer(node_, node_->get_clock(), period,
                                          std::bind(&StateEstimator::publish, this));
}

void StateEstimator::load() {
  auto pattern_match = [](std::string &pl, std::string &pl_candidate) {
    int cmp = fnmatch(pl.c_str(), pl_candidate.c_str(), FNM_CASEFOLD);
    if (cmp == 0)
      return true;

    if (cmp != FNM_NOMATCH) {
      RCLCPP_ERROR(LOGGER,
                "Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
                pl.c_str(), pl_candidate.c_str(), cmp);
    }

    return false;
  };

  node_->get_parameter_or("estimation/mode", estimate_mode_, 0); //EGOMOTION_ESTIMATE: 0
  if (estimate_mode_ > GROUND_TRUTH) {
      RCLCPP_ERROR(LOGGER,"the estimate mode is not correct: %d. \
                   It should be [0, 1, 2].",
                  estimate_mode_);
      return;
  }

  std::string estimate_mode_str
    = (estimate_mode_ == EGOMOTION_ESTIMATE)? std::string("EGOMOTION_ESTIMATE"):
    ((estimate_mode_ == EXPERIMENT_ESTIMATE)?
     std::string("EXPERIMENT_ESTIMATE"): std::string("GROUND_TRUTH"));
  RCLCPP_INFO_STREAM(LOGGER, std::string("\033[32m estimate mode: ") <<
                     estimate_mode_str << std::string("\033[0m"));

  /* kalman filter egomotion plugin initialization for 0: egomotion, 1: experiment */
  string fuse_prefix = "estimation.fusion.";
  for (int i = 0; i < 2; i++) {
    /* kalman filter egomotion plugin list */
    string mode_prefix;
    if(i == EGOMOTION_ESTIMATE) mode_prefix = string("egomotion");
    else if(i == EXPERIMENT_ESTIMATE) mode_prefix = string("experiment");

    std::vector<std::string> fuser_list;
    if(!node_->get_parameter<std::vector<std::string>>
       (fuse_prefix + mode_prefix + "_list", fuser_list)) {
      RCLCPP_ERROR_STREAM(LOGGER,
                          fuse_prefix << mode_prefix << "_list" << " is not set");
      return;
    }

    int cnt = 0;
    for (auto &fuser_name : fuser_list) {
      for (auto &name : fusion_loader_.getDeclaredClasses()) {
        if(!pattern_match(fuser_name, name)) continue;

        std::stringstream fuser_no;
        fuser_no << cnt + 1;

        int fuser_id;

        std::string fuser_id_param
          = fuse_prefix + "fuser_" + mode_prefix + "_id" + fuser_no.str();
        if (!node_->get_parameter(fuser_id_param, fuser_id)) {
          RCLCPP_ERROR(LOGGER, "%s, no param in fuser %s id",
                       mode_prefix.c_str(), fuser_no.str().c_str());
          continue;
        }

        std::string fuser_name_param
          = fuse_prefix + "fuser_" + mode_prefix + "_label" + fuser_no.str();
        if (!node_->get_parameter(fuser_name_param, fuser_name)) {
          RCLCPP_ERROR(LOGGER, "%s, no param in fuser %s name",
                       mode_prefix.c_str(), fuser_no.str().c_str());
          continue;
        }

        try {
          std::shared_ptr<kf_plugin::KalmanFilter> plugin_ptr
            = fusion_loader_.createSharedInstance(name);
          plugin_ptr->initialize(fuser_name, fuser_id);
          fuser_maps_.at(i).insert(make_pair(name, plugin_ptr));

          cnt ++;
        } catch (const pluginlib::PluginlibException& ex) {
          RCLCPP_ERROR(LOGGER,
                       "Failed to load kf plugin '%s': %s",
                       name.c_str(), ex.what());
        }
      }
    }
  }

  std::vector<std::string> sensor_list{};
  if(!node_->get_parameter<std::vector<std::string>>(fuse_prefix + "sensor_list", sensor_list)) {
    RCLCPP_ERROR_STREAM(LOGGER, fuse_prefix + "sensor_list" << " is not set");
    return;
  }

  vector<int> sensor_index(0);

  for (auto &plugin_name : sensor_list) {
    for (auto &name : sensor_loader_.getDeclaredClasses()) {
      if(!pattern_match(plugin_name, name)) continue;

      sensors_.push_back(sensor_loader_.createSharedInstance(name));
      sensor_index.push_back(1);

      if(name.find("imu") != std::string::npos) {
        imu_handlers_.push_back(sensors_.back());
        sensor_index.back() = imu_handlers_.size();
      }

      if(name.find("gps") != std::string::npos) {
        gps_handlers_.push_back(sensors_.back());
        sensor_index.back() = gps_handlers_.size();
      }

      if(name.find("alt") != std::string::npos) {
        alt_handlers_.push_back(sensors_.back());
        sensor_index.back() = alt_handlers_.size();
      }

      if(name.find("vo") != std::string::npos) {
        vo_handlers_.push_back(sensors_.back());
        sensor_index.back() = vo_handlers_.size();
      }

      sensors_.back()->initialize(node_, robot_model_, shared_from_this(), name, sensor_index.back());
      break;
    }
  }
}

int StateEstimator::getBasePosStateStatus(uint8_t axis, uint8_t estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  return base_pos_status_matrix_.at(estimate_mode).at(axis);
}

void StateEstimator::setBasePosStateStatus(uint8_t axis, uint8_t estimate_mode, bool status) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  int& curr_status = base_pos_status_matrix_.at(estimate_mode).at(axis);

  if(status) {
    curr_status++;
  } else {
    if(curr_status > 0) {
      curr_status --;
    } else {
      RCLCPP_WARN(LOGGER,
                  "wrong pos status update for axis: %d, estimate mode: %d",
                  axis, estimate_mode);
    }
  }
}

int StateEstimator::getBaseRotStateStatus(uint8_t estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  return base_rot_status_.at(estimate_mode);
}

void StateEstimator::setBaseRotStateStatus(uint8_t estimate_mode, bool status) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  int& curr_status = base_rot_status_.at(estimate_mode);

  if(status) {
    curr_status++;
  } else {
    if(curr_status > 0) {
      curr_status --;
    } else {
      RCLCPP_WARN(LOGGER,
                  "wrong rot status update for estimate mode: %d", estimate_mode);
    }
  }
}

int StateEstimator::getCogPosStateStatus(uint8_t axis, uint8_t estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  return cog_pos_status_matrix_.at(estimate_mode).at(axis);
}

void StateEstimator::setCogPosStateStatus(uint8_t axis, uint8_t estimate_mode, bool status) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  int& curr_status = cog_pos_status_matrix_.at(estimate_mode).at(axis);

  if(status) {
    curr_status++;
  } else {
    if(curr_status > 0) {
      curr_status --;
    } else {
      RCLCPP_WARN(LOGGER,
                  "wrong pos status update for axis: %d, estimate mode: %d",
                  axis, estimate_mode);
    }
  }
}

int StateEstimator::getCogRotStateStatus(uint8_t estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  return cog_rot_status_.at(estimate_mode);
}

void StateEstimator::setCogRotStateStatus(uint8_t estimate_mode, bool status) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  int& curr_status = cog_rot_status_.at(estimate_mode);

  if(status) {
    curr_status++;
  } else {
    if(curr_status > 0) {
      curr_status --;
    } else {
      RCLCPP_WARN(LOGGER,
                  "wrong rot status update for estimate mode: %d", estimate_mode);
    }
  }
}

const KDL::Frame StateEstimator::getBasePose(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_pose_.at(estimate_mode);
}

void StateEstimator::setBasePose(int estimate_mode, KDL::Frame pose) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode) = pose;
}

const KDL::Twist StateEstimator::getBaseTwist(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_twist_.at(estimate_mode);
}

void StateEstimator::setBaseTwist(int estimate_mode, KDL::Twist twist) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode) = twist;
}

const KDL::Vector StateEstimator::getBasePos(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_pose_.at(estimate_mode).p;
}

void StateEstimator::setBasePos(int estimate_mode, KDL::Vector pos) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p = pos;
}

void StateEstimator::setBasePosX(int estimate_mode, double pos) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p.x(pos);
}

void StateEstimator::setBasePosY(int estimate_mode, double pos) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p.y(pos);
}

void StateEstimator::setBasePosZ(int estimate_mode, double pos) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).p.z(pos);
}

const KDL::Vector StateEstimator::getBaseVel(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_twist_.at(estimate_mode).vel;
}

void StateEstimator::setBaseVel(int estimate_mode, KDL::Vector vel) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel = vel;
}

void StateEstimator::setBaseVelX(int estimate_mode, double vel) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel.x(vel);
}

void StateEstimator::setBaseVelY(int estimate_mode, double vel) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel.y(vel);
}

void StateEstimator::setBaseVelZ(int estimate_mode, double vel) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).vel.z(vel);
}

const KDL::Vector StateEstimator::getBaseAcc(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_acc_.at(estimate_mode);
}

void StateEstimator::setBaseAcc(int estimate_mode, KDL::Vector acc) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_acc_.at(estimate_mode) = acc;
}

const KDL::Rotation StateEstimator::getBaseOrientation(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return base_pose_.at(estimate_mode).M;
}

void StateEstimator::setBaseOrientation(int estimate_mode, KDL::Rotation rot) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_pose_.at(estimate_mode).M = rot;
}

const KDL::Vector StateEstimator::getBaseEuler(int estimate_mode) {
  KDL::Rotation rot = getBaseOrientation(estimate_mode);
  double r, p, y;
  rot.GetRPY(r, p, y);
  return KDL::Vector(r, p, y);
}

const KDL::Vector StateEstimator::getBaseAngularVel(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  return base_twist_.at(estimate_mode).rot;
}

void StateEstimator::setBaseAngularVel(int estimate_mode, KDL::Vector omega) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  base_twist_.at(estimate_mode).rot = omega;
}

const KDL::Frame StateEstimator::getCogPose(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_pose_.at(estimate_mode);
}

void StateEstimator::setCogPose(int estimate_mode, KDL::Frame pose) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(estimate_mode) = pose;
}

const KDL::Twist StateEstimator::getCogTwist(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_twist_.at(estimate_mode);
}

void StateEstimator::setCogTwist(int estimate_mode, KDL::Twist twist) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_twist_.at(estimate_mode) = twist;
}

const KDL::Vector StateEstimator::getCogPos(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_pose_.at(estimate_mode).p;
}

void StateEstimator::setCogPos(int estimate_mode, KDL::Vector pos) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(estimate_mode).p = pos;
}

const KDL::Vector StateEstimator::getCogVel(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_twist_.at(estimate_mode).vel;
}

void StateEstimator::setCogVel(int estimate_mode, KDL::Vector vel) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_twist_.at(estimate_mode).vel = vel;
}

const KDL::Vector StateEstimator::getCogAcc(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_acc_.at(estimate_mode);
}

void StateEstimator::setCogAcc(int estimate_mode, KDL::Vector acc) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_acc_.at(estimate_mode) = acc;
}

const KDL::Rotation StateEstimator::getCogOrientation(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cog_pose_.at(estimate_mode).M;
}

void StateEstimator::setCogOrientation(int estimate_mode, KDL::Rotation rot) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_pose_.at(estimate_mode).M = rot;
}

const KDL::Vector StateEstimator::getCogEuler(int estimate_mode) {
  KDL::Rotation rot = getCogOrientation(estimate_mode);
  double r, p, y;
  rot.GetRPY(r, p, y);
  return KDL::Vector(r, p, y);
}

const KDL::Vector StateEstimator::getCogAngularVel(int estimate_mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  return cog_twist_.at(estimate_mode).rot;
}

void StateEstimator::setCogAngularVel(int estimate_mode, KDL::Vector omega) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cog_twist_.at(estimate_mode).rot = omega;
}

void StateEstimator::setBaseOrientationWxB(int estimate_mode, KDL::Vector v) {
  KDL::Vector wx_b = v;
  wx_b.Normalize();

  KDL::Rotation rot_inv = getBaseOrientation(estimate_mode).Inverse();
  KDL::Vector wz_b = rot_inv.UnitZ();
  KDL::Vector wy_b = wz_b * wx_b;
  wy_b.Normalize();

  rot_inv.UnitX(wx_b);
  rot_inv.UnitY(wy_b);
  rot_inv.UnitZ(wz_b);

  setBaseOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::setBaseOrientationWzB(int estimate_mode, KDL::Vector v) {
  KDL::Vector wz_b = v;
  wz_b.Normalize();

  KDL::Rotation rot_inv = getBaseOrientation(estimate_mode).Inverse();
  KDL::Vector wx_b = rot_inv.UnitX();
  KDL::Vector wy_b = wz_b * wx_b;
  wy_b.Normalize();

  rot_inv.UnitX(wx_b);
  rot_inv.UnitY(wy_b);
  rot_inv.UnitZ(wz_b);

  setBaseOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::setCogOrientationWxB(int estimate_mode, KDL::Vector v) {
  KDL::Vector wx_c = v;
  wx_c.Normalize();

  KDL::Rotation rot_inv = getCogOrientation(estimate_mode).Inverse();
  KDL::Vector wz_c = rot_inv.UnitZ();
  KDL::Vector wy_c = wz_c * wx_c;
  wy_c.Normalize();

  rot_inv.UnitX(wx_c);
  rot_inv.UnitY(wy_c);
  rot_inv.UnitZ(wz_c);

  setCogOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::setCogOrientationWzB(int estimate_mode, KDL::Vector v) {
  KDL::Vector wz_c = v;
  wz_c.Normalize();

  KDL::Rotation rot_inv = getCogOrientation(estimate_mode).Inverse();
  KDL::Vector wx_c = rot_inv.UnitX();
  KDL::Vector wy_c = wz_c * wx_c;
  wy_c.Normalize();

  rot_inv.UnitX(wx_c);
  rot_inv.UnitY(wy_c);
  rot_inv.UnitZ(wz_c);

  setCogOrientation(estimate_mode, rot_inv.Inverse());
}

void StateEstimator::updateBaseQueue(const double timestamp, const KDL::Rotation r_ee, const KDL::Rotation r_ex, const KDL::Vector omega) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  timestamp_qu_.push_back(timestamp);
  base_rot_ee_qu_.push_back(r_ee);
  base_rot_ex_qu_.push_back(r_ex);
  base_omega_qu_.push_back(omega);

  if(timestamp_qu_.size() > qu_size_) {
    timestamp_qu_.pop_front();
    base_rot_ee_qu_.pop_front();
    base_rot_ex_qu_.pop_front();
    base_omega_qu_.pop_front();
  }
}

bool StateEstimator::findBaseRotOmega(const double timestamp, const int mode, KDL::Rotation& r, KDL::Vector& omega, bool verbose) {
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if(timestamp_qu_.size() == 0) {

    if (verbose) {
      RCLCPP_WARN(LOGGER,
                  "estimation: no valid queue for timestamp to find proper r and omega");
    }

    return false;
  }

  if(timestamp < timestamp_qu_.front()) {
    if (verbose) {
      RCLCPP_WARN_STREAM(LOGGER,
                         "estimation: sensor timestamp "
                         << timestamp << " is earlier than the oldest timestamp "
                         << timestamp_qu_.front() << " in queue");
    }
    return false;
  }

  if(timestamp > timestamp_qu_.back()) {
    if (verbose) {
      RCLCPP_WARN_STREAM(LOGGER,
                        "estimation: sensor timestamp "
                        << timestamp << " is later than the latest timestamp "
                        << timestamp_qu_.back() << " in queue");
    }
    return false;
  }

  size_t candidate_index = (timestamp_qu_.size() - 1) * (timestamp - timestamp_qu_.front()) / (timestamp_qu_.back() - timestamp_qu_.front());

  if(timestamp > timestamp_qu_.at(candidate_index)) {
    for(auto it = timestamp_qu_.begin() + candidate_index; it != timestamp_qu_.end(); ++it) {
      /* future timestamp, escape */
      if(*it > timestamp) {
        if (fabs(*it - timestamp) < fabs(*(it - 1) - timestamp))
          candidate_index = std::distance(timestamp_qu_.begin(), it);
        else
          candidate_index = std::distance(timestamp_qu_.begin(), it-1);

        //RCLCPP_INFO(LOGGER, "find timestamp sensor vs imu: [%f, %f], candidate: %d", timestamp, timestamp_qu_.at(candidate_index), candidate_index);
        break;
      }
    }
  } else {
    for(auto it = timestamp_qu_.rbegin() + (timestamp_qu_.size() - 1 - candidate_index); it != timestamp_qu_.rend(); ++it) {
      /* future timestamp, escape */
      if(*it < timestamp) {
        if (fabs(*it - timestamp) < fabs(*(it - 1) - timestamp)) {
          candidate_index = timestamp_qu_.size() - 1 - std::distance(timestamp_qu_.rbegin(), it);
        } else {
          candidate_index = timestamp_qu_.size() - 1 - std::distance(timestamp_qu_.rbegin(), it-1);
        }

        //RCLCPP_INFO(LOGGER, "reverse find timestamp sensor vs imu: [%f, %f], %d", timestamp, timestamp_qu_.at(candidate_index) , candidate_index);
        break;
      }
    }
  }

  omega = base_omega_qu_.at(candidate_index);
  switch(mode) {
  case EGOMOTION_ESTIMATE:
    r = base_rot_ee_qu_.at(candidate_index);
    break;
  case EXPERIMENT_ESTIMATE:
    r = base_rot_ex_qu_.at(candidate_index);
    break;
  default:
    RCLCPP_ERROR(LOGGER,
                 "estimation search state with timestamp: wrong mode %d", mode);
    return false;
  }

  return true;
}

const double StateEstimator::getImuLatestTimeStamp() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return timestamp_qu_.back();
}

const FuserMap& StateEstimator::getFuserMap(int mode) {
  return fuser_maps_.at(mode);
}

/* set unhealth level */
void StateEstimator::setUnhealthLevel(uint8_t unhealth_level) {
  if(unhealth_level > unhealth_level_) unhealth_level_ = unhealth_level;

  /* TODO: should write the solution for the unhealth sensor  */
}

void StateEstimator::publish() {
  rclcpp::Time imu_stamp = imu_handlers_.at(0)->getTimeStamp();

  odomPublish(imu_stamp);
  tfBroadcast(imu_stamp);

  prev_pub_stamp_ = imu_stamp;
}

void StateEstimator::odomPublish(rclcpp::Time stamp) {
  nav_msgs::msg::Odometry odom_state;
  odom_state.header.stamp = stamp;
  odom_state.header.frame_id = "world";

  /* publish Baselink odometry */
  std::string base_name = robot_model_->getBaselinkName();
  odom_state.child_frame_id
    = tf_prefix_.empty() ? base_name : tf_prefix_ + "/" + base_name;
  odom_state.pose.pose = tf2::toMsg(getBasePose(estimate_mode_));
  odom_state.twist.twist =
    aerial_robot_model::kdlToMsg(getBaseTwist(estimate_mode_));
  baselink_odom_pub_->publish(odom_state);

  /* publish CoG odometry */
  odom_state.child_frame_id = tf_prefix_.empty() ? "cog" : tf_prefix_ + "/cog";
  odom_state.pose.pose = tf2::toMsg(getCogPose(estimate_mode_));
  odom_state.twist.twist =
    aerial_robot_model::kdlToMsg(getCogTwist(estimate_mode_));
  cog_odom_pub_->publish(odom_state);
}

void StateEstimator::tfBroadcast(rclcpp::Time stamp) {
  /* avoid the redundant timestamp which induces annoying logs from TF server */
  if (stamp == prev_pub_stamp_) return;

  const auto segments_tf = robot_model_->getSegmentsTf();
  /* skip if kinemtiacs is not initialized */
  if(segments_tf.size() == 0) return;

  auto tf_base2root = segments_tf.at(robot_model_->getBaselinkName()).Inverse();
  auto tf_world2base = getBasePose(estimate_mode_);
  geometry_msgs::msg::TransformStamped tf_msg
    = aerial_robot_model::kdlToMsg(tf_world2base * tf_base2root);

  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = tf_prefix_.empty() ? "root" : tf_prefix_ + "/root";

  br_->sendTransform(tf_msg);
}

