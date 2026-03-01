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

#pragma once

/* common utility */
#include <deque>
#include <fnmatch.h>
#include <kdl/frames.hpp>

/* aerial_robot_core API */
#include <kalman_filter/kf_base_plugin.h>
#include <aerial_robot_model/model/aerial_robot_model.h>

/* ros API */
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/* ros messages */
#include <aerial_robot_msgs/msg/states.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <geographic_msgs/msg/geo_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>

using StatusVector = std::array<int, 3>; // x, y, z
using StatusMatrix = std::array<StatusVector, 3>; // egomotion, experiment, ground truth
using SensorFuser = std::vector< std::pair<std::string, std::shared_ptr<kf_plugin::KalmanFilter> > >;

namespace Sensor
{
  enum
    {
      UNHEALTH_LEVEL1 = 1, // do nothing
      UNHEALTH_LEVEL2, // change estimation mode
      UNHEALTH_LEVEL3, // force landing
    };
};

/* pre-definition */
namespace sensor_plugin
{
  class  SensorBase;
};


namespace aerial_robot_estimation
{
  //mode
  static constexpr int NONE = -1;
  static constexpr int EGOMOTION_ESTIMATE = 0;
  static constexpr int EXPERIMENT_ESTIMATE = 1;
  static constexpr int GROUND_TRUTH = 2;

  static constexpr float G = 9.797;

  class StateEstimator: public std::enable_shared_from_this<StateEstimator>
  {

  public:
    StateEstimator();
    virtual ~StateEstimator() = default;

    void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model);

    int getBasePosStateStatus(uint8_t axis, uint8_t estimate_mode);
    int getCogPosStateStatus(uint8_t axis, uint8_t estimate_mode);
    int getBaseRotStateStatus(uint8_t estimate_mode);
    int getCogRotStateStatus(uint8_t estimate_mode);
    void setBasePosStateStatus(uint8_t axis, uint8_t estimate_mode, bool status);
    void setCogPosStateStatus(uint8_t axis, uint8_t estimate_mode, bool status);
    void setBaseRotStateStatus(uint8_t estimate_mode, bool status);
    void setCogRotStateStatus(uint8_t estimate_mode, bool status);

    const KDL::Frame getBasePose(int estimate_mode);
    void setBasePose(int estimate_mode, KDL::Frame pose);
    const KDL::Twist getBaseTwist(int estimate_mode);
    void setBaseTwist(int estimate_mode, KDL::Twist twist);
    const KDL::Vector getBasePos(int estimate_mode);
    void setBasePos(int estimate_mode, KDL::Vector pos);
    const KDL::Vector getBaseVel(int estimate_mode);
    void setBaseVel(int estimate_mode, KDL::Vector vel);
    const KDL::Rotation getBaseOrientation(int estimate_mode);
    void setBaseOrientation(int estimate_mode, KDL::Rotation rot);
    const KDL::Vector getBaseEuler(int estimate_mode);
    const KDL::Vector getBaseAngularVel(int estimate_mode);
    void setBaseAngularVel(int estimate_mode, KDL::Vector omega);

    const KDL::Frame getCogPose(int estimate_mode);
    void setCogPose(int estimate_mode, KDL::Frame pose);
    const KDL::Twist getCogTwist(int estimate_mode);
    void setCogTwist(int estimate_mode, KDL::Twist twist);
    const KDL::Vector getCogPos(int estimate_mode);
    void setCogPos(int estimate_mode, KDL::Vector pos);
    const KDL::Vector getCogVel(int estimate_mode);
    void setCogVel(int estimate_mode, KDL::Vector vel);
    const KDL::Rotation getCogOrientation(int estimate_mode);
    void setCogOrientation(int estimate_mode, KDL::Rotation rot);
    const KDL::Vector getCogEuler(int estimate_mode);
    const KDL::Vector getCogAngularVel(int estimate_mode);
    void setCogAngularVel(int estimate_mode, KDL::Vector omega);

    void setBaseOrientationWxB(int estimate_mode, KDL::Vector v);
    void setBaseOrientationWzB(int estimate_mode, KDL::Vector v);


    inline void setBaseQueueSize(const int& qu_size) {qu_size_ = qu_size;}
    void updateBaseQueue(const double timestamp, const KDL::Rotation r_ee, const KDL::Rotation r_ex, const KDL::Vector omega);
    bool findBaseRotOmega(const double timestamp, const int mode, KDL::Rotation& r, KDL::Vector& omega, bool verbose = true);

    const double getImuLatestTimeStamp();

    inline void setSensorFusionFlag(bool flag){sensor_fusion_flag_ = flag;  }
    inline bool getSensorFusionFlag(){return sensor_fusion_flag_; }

    //start flying flag (~takeoff)
    virtual bool getFlyingFlag() {  return  flying_flag_;}
    virtual void setFlyingFlag(bool flag){  flying_flag_ = flag;}
    /* when takeoff, should use the undescend mode be true */
    inline void setUnDescendMode(bool flag){un_descend_flag_ = flag;  }
    inline bool getUnDescendMode(){return un_descend_flag_; }
    /* att control mode is for user to manually control x and y motion by attitude control */
    inline void setForceAttControlFlag (bool flag) {force_att_control_flag_ = flag; }
    inline bool getForceAttControlFlag () {return force_att_control_flag_;}
    /* latitude & longitude value for GPS based navigation */
    // inline void setCurrGpsPoint(const geographic_msgs::GeoPoint point) {curr_wgs84_poiont_ = point;}
    // inline const geographic_msgs::GeoPoint& getCurrGpsPoint() const {return curr_wgs84_poiont_;}
    inline const bool hasGroundTruthOdom() const {return has_groundtruth_odom_; }
    inline void receiveGroundTruthOdom(bool flag) {has_groundtruth_odom_ = flag; }
    inline const bool hasRefinedYawEstimate(int i) const {return has_refined_yaw_estimate_.at(i); }
    inline void SetRefinedYawEstimate(int i, bool flag) {has_refined_yaw_estimate_.at(i) = flag; }

    const SensorFuser& getFuser(int mode);

    inline int getEstimateMode() {return estimate_mode_;}
    inline void setEstimateMode(int estimate_mode) {estimate_mode_ = estimate_mode;}

    /* set unhealth level */
    void setUnhealthLevel(uint8_t unhealth_level);

    inline uint8_t getUnhealthLevel() { return unhealth_level_; }
    std::string getTFPrefix() {return tf_prefix_;}

    const vector<std::shared_ptr<sensor_plugin::SensorBase> >& getImuHandlers() const { return imu_handlers_;}
    const vector<std::shared_ptr<sensor_plugin::SensorBase> >& getAltHandlers() const { return alt_handlers_;}
    const vector<std::shared_ptr<sensor_plugin::SensorBase> >& getVoHandlers() const { return vo_handlers_;}
    const vector<std::shared_ptr<sensor_plugin::SensorBase> >& getGpsHandlers() const { return gps_handlers_;}

    const std::shared_ptr<sensor_plugin::SensorBase> getImuHandler(int i) const { return imu_handlers_.at(i);}
    const std::shared_ptr<sensor_plugin::SensorBase> getAltHandlers(int i) const { return alt_handlers_.at(i);}
    const std::shared_ptr<sensor_plugin::SensorBase> getVoHandlers(int i) const { return vo_handlers_.at(i);}
    const std::shared_ptr<sensor_plugin::SensorBase> getGpsHandlers(int i) const { return gps_handlers_.at(i);}

  protected:

    /* node handle */
    rclcpp::Node::SharedPtr node_;

    /* publisher */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr baselink_odom_pub_, cog_odom_pub_;

    /* timer */
    rclcpp::TimerBase::SharedPtr state_pub_timer_;

    /* tf broadcaster */
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;


    /* sensor handlers */
    pluginlib::ClassLoader<sensor_plugin::SensorBase> sensor_loader_;
    vector<std::shared_ptr<sensor_plugin::SensorBase> > sensors_;
    vector<std::shared_ptr<sensor_plugin::SensorBase> > imu_handlers_;
    vector<std::shared_ptr<sensor_plugin::SensorBase> > alt_handlers_;
    vector<std::shared_ptr<sensor_plugin::SensorBase> > vo_handlers_;
    vector<std::shared_ptr<sensor_plugin::SensorBase> > gps_handlers_;

    /* mutex */
    std::mutex state_mutex_;
    std::mutex queue_mutex_;
    /* ros param */
    int estimate_mode_; /* main estimte mode */

    /* robot model (kinematics)  */
    std::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
    std::string tf_prefix_;

    /* states */
    StatusMatrix base_pos_status_matrix_, cog_pos_status_matrix_;
    std::array<int, 3> base_rot_status_, cog_rot_status_;
    std::array<KDL::Frame, 3> base_pose_, cog_pose_;
    std::array<KDL::Twist, 3> base_twist_, cog_twist_;
    std::array<KDL::Vector, 3> base_acc_, cog_acc_;

    bool has_groundtruth_odom_; // whether receive entire groundthtruth odometry (e.g., for simulation mode)

    std::map<int, bool> has_refined_yaw_estimate_; // whether receive refined yaw estimation data (e.g., vio) for each estimate mode

    /* for calculate the sensor to baselink with the consideration of time delay */
    int qu_size_;
    deque<double> timestamp_qu_;
    deque<KDL::Rotation> base_rot_ee_qu_, base_rot_ex_qu_;
    deque<KDL::Vector> base_omega_qu_;

    /* sensor fusion */
    pluginlib::ClassLoader<kf_plugin::KalmanFilter> fusion_loader_;
    bool sensor_fusion_flag_;
    std::array<SensorFuser, 2> fuser_; //0: egomotion; 1: experiment

    /* sensor (un)health level */
    uint8_t unhealth_level_;

    /* height related var */
    bool flying_flag_;
    bool un_descend_flag_;
    bool force_att_control_flag_;

    /* latitude & longitude point */
    //geographic_msgs::GeoPoint curr_wgs84_poiont_;

    /* time */
    rclcpp::Time prev_pub_stamp_;

    void publish();
    void odomPublish(rclcpp::Time stamp);
    void tfBroadcast(rclcpp::Time stamp);
    void load();
  };
};
