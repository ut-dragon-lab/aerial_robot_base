#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Software License Agreement (BSD License)

Copyright (c) 2024, DRAGON Laboratory, The University of Tokyo
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import math
import numpy as np
import time
import rclpy
from rclpy.node import Node

import ros2_numpy as ros_np
import tf2_ros
from tf_transformations import * 

from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Empty, Int8, UInt8, String
from std_srvs.srv import SetBool

def nsecToSec(nsec):
    return nsec / 1e9

class RobotInterface(Node):
    def __init__(self, robot_ns="", debug_view=False):
        super().__init__('rotor_interface')
        
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6

        self.debug_view = debug_view
        self.joint_state = JointState()
        self.cog_odom = None
        self.base_odom = None
        self.flight_state = None
        self.target_pos = np.array([0, 0, 0])
        
        self.robot_ns = robot_ns
        
        self.start_pub = self.create_publisher(Empty, self.robot_ns + '/teleop_command/start', 1)
        self.takeoff_pub = self.create_publisher(Empty, self.robot_ns + '/teleop_command/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, self.robot_ns + '/teleop_command/land', 1)
        self.force_landing_pub = self.create_publisher(Empty, self.robot_ns + '/teleop_command/force_landing', 1)
        self.halt_pub = self.create_publisher(Empty, self.robot_ns + '/teleop_command/halt', 1)
        
        # Odometry & Control
        self.cog_odom_sub = self.create_subscription(Odometry, self.robot_ns + '/uav/cog/odom', self.cogOdomCallback, 10)
        self.base_odom_sub = self.create_subscription(Odometry, self.robot_ns + '/uav/baselink/odom', self.baseOdomCallback, 10)
        self.control_pid_sub = self.create_subscription(PoseControlPid, self.robot_ns + '/debug/pose/pid', self.controlPidCallback, 10)
        
        # Navigation
        self.flight_state_sub = self.create_subscription(UInt8, self.robot_ns + '/flight_state', self.flightStateCallback, 10)
        self.traj_nav_pub = self.create_publisher(PoseStamped, self.robot_ns + '/target_pose', 1)
        self.direct_nav_pub = self.create_publisher(FlightNav, self.robot_ns + '/uav/nav', 1)
        self.final_rot_pub = self.create_publisher(Vector3Stamped, self.robot_ns + '/final_target_baselink_rpy', 1)
        
        # Joint
        self.joint_state_sub = self.create_subscription(JointState, self.robot_ns + '/joint_states', self.jointStateCallback, 10)
        self.joint_ctrl_pub = self.create_publisher(JointState, self.robot_ns + '/joints_ctrl', 1)
        self.set_joint_torque_client = self.create_client(SetBool, self.robot_ns + '/joints/torque_enable')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        start_time = nsecToSec(self.get_clock().now().nanoseconds)
        while rclpy.ok():
            current_time = nsecToSec(self.get_clock().now().nanoseconds)
            if current_time - start_time > 5.0:
                self.get_logger().error("cannot connect to {}".format(self.robot_ns))
                break
            if self.base_odom is not None:
                self.get_logger().info("Connected to {}!".format(self.robot_ns))
                break
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def cogOdomCallback(self, msg):
        self.cog_odom = msg

    def baseOdomCallback(self, msg):
        self.base_odom = msg

    def flightStateCallback(self, msg):
        self.flight_state = msg.data

    def controlPidCallback(self, msg):
        self.control_pid = msg

    def jointStateCallback(self, msg):
        js = JointState()
        js.name = []
        js.position = []
        for n, j in zip(msg.name, msg.position):
            if 'joint' in n:
                js.name.append(n)
                js.position.append(j)
        self.joint_state = js

    def getControlPid(self):
        return self.control_pid

    def getBaseOdom(self):
        return self.base_odom

    def getCogOdom(self):
        return self.cog_odom

    def getBasePos(self):
        return ros_np.numpify(self.base_odom.pose.pose.position)

    def getBaseRot(self):
        return ros_np.numpify(self.base_odom.pose.pose.orientation)

    def getBaseRPY(self):
        return euler_from_quaternion(self.getBaseRot())

    def getBaseLinearVel(self):
        return ros_np.numpify(self.base_odom.twist.twist.linear)

    def getBaseAngularVel(self):
        return ros_np.numpify(self.base_odom.twist.twist.angular)

    def getCogPos(self):
        return ros_np.numpify(self.cog_odom.pose.pose.position)

    def getCogRot(self):
        return ros_np.numpify(self.cog_odom.pose.pose.orientation)

    def getCogRPY(self):
        return euler_from_quaternion(self.getCogRot())

    def getCogLinVel(self):
        return ros_np.numpify(self.cog_odom.twist.twist.linear)

    def getCogAngVel(self):
        return ros_np.numpify(self.cog_odom.twist.twist.angular)

    def getFlightState(self):
        return self.flight_state

    def getTargetPos(self):
        return self.target_pos

    def start(self, sleep=1.0):
        self.start_pub.publish(Empty())
        time.sleep(sleep)

    def takeoff(self):
        self.takeoff_pub.publish(Empty())

    def land(self):
        self.land_pub.publish(Empty())

    def forceLanding(self):
        self.force_landing_pub.publish(Empty())

    def halt(self):
        self.halt_pub.publish(Empty())

    def goPos(self, pos, pos_thresh=0.1, vel_thresh=0.05, timeout=30):
        return self.navigate(pos=pos, pos_thresh=pos_thresh, vel_thresh=vel_thresh, timeout=timeout)

    def rotateYaw(self, yaw, yaw_thresh=0.1, timeout=30):
        rot = quaternion_from_euler(0, 0, yaw)
        return self.rotate(rot, yaw_thresh, timeout)

    def rotate(self, rot, rot_thresh=0.1, timeout=30):
        return self.navigate(rot=rot, rot_thresh=rot_thresh, timeout=timeout)

    def goPosYaw(self, pos, yaw, pos_thresh=0.1, vel_thresh=0.05, yaw_thresh=0.1, timeout=30):
        rot = quaternion_from_euler(0, 0, yaw)
        return self.goPose(pos, rot, pos_thresh, vel_thresh, yaw_thresh, timeout)

    def goPose(self, pos, rot, pos_thresh=0.1, vel_thresh=0.05, rot_thresh=0.1, timeout=30):
        return self.navigate(pos=pos, rot=rot, pos_thresh=pos_thresh, vel_thresh=vel_thresh, rot_thresh=rot_thresh, timeout=timeout)

    def goVel(self, vel):
        return self.navigate(lin_vel=vel)

    def rotateVel(self, vel):
        return self.navigate(ang_vel=vel)

    def trajectoryNavigate(self, pos, rot):
        if self.flight_state != self.HOVER_STATE:
            self.get_logger().error("[Navigate] flight state ({}) disallows navigation".format(self.flight_state))
            return
        if pos is None:
            pos = self.getCogPos()
        if rot is None:
            rot = self.getCogRot()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]
        self.traj_nav_pub.publish(msg)

    def directNavigate(self, pos, rot, lin_vel, ang_vel):
        if self.flight_state != self.HOVER_STATE:
            self.get_logger().error("[Navigate] flight state ({}) disallows navigation".format(self.flight_state))
            return
        msg = FlightNav()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        if pos is None:
            pos_mode = FlightNav.VEL_MODE if (lin_vel is not None) else FlightNav.NO_NAVIGATION
        else:
            pos_mode = FlightNav.POS_MODE if (lin_vel is None) else FlightNav.POS_VEL_MODE
        if rot is None:
            rot_mode = FlightNav.VEL_MODE if (ang_vel is not None) else FlightNav.NO_NAVIGATION
        else:
            rot_mode = FlightNav.POS_MODE if (ang_vel is None) else FlightNav.POS_VEL_MODE
        if pos is None:
            pos = self.getCogPos()
        if rot is None:
            rot = self.getCogRot()
        _, _, yaw = euler_from_quaternion(rot)
        if lin_vel is None:
            lin_vel = np.array([0, 0, 0])
        if ang_vel is None:
            ang_vel = np.array([0, 0, 0])
        msg.pos_xy_nav_mode = pos_mode
        msg.target_pos_x = pos[0]
        msg.target_pos_y = pos[1]
        msg.target_vel_x = lin_vel[0]
        msg.target_vel_y = lin_vel[1]
        msg.pos_z_nav_mode = pos_mode
        msg.target_pos_z = pos[2]
        msg.target_vel_z = lin_vel[2]
        msg.yaw_nav_mode = rot_mode
        msg.target_yaw = yaw
        msg.target_omega_z = ang_vel[2]
        self.direct_nav_pub.publish(msg)

    def navigate(self, pos=None, rot=None, lin_vel=None, ang_vel=None, pos_thresh=0.1, vel_thresh=0, rot_thresh=0, timeout=-1):
        if self.flight_state != self.HOVER_STATE:
            self.get_logger().error("[Navigate] flight state ({}) disallows navigation".format(self.flight_state))
            return False
        if rot is not None:
            if len(rot) == 3:
                rot = quaternion_from_euler(*rot)
        if lin_vel is None and ang_vel is None:
            self.trajectoryNavigate(pos, rot)
        else:
            self.directNavigate(pos, rot, lin_vel, ang_vel)
        return self.poseConvergenceCheck(timeout, target_pos=pos, target_rot=rot, pos_thresh=pos_thresh, vel_thresh=vel_thresh, rot_thresh=rot_thresh)

    def poseConvergenceCheck(self, timeout, target_pos=None, target_rot=None, pos_thresh=None, vel_thresh=None, rot_thresh=None):
        return self.convergenceCheck(timeout, self.poseThresholdCheck, target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh)

    def poseThresholdCheck(self, target_pos=None, target_rot=None, pos_thresh=None, vel_thresh=None, rot_thresh=None):
        if pos_thresh is None:
            pos_thresh = self.default_pos_thresh
        if isinstance(pos_thresh, float):
            pos_thresh = [pos_thresh] * 3
        if rot_thresh is None:
            rot_thresh = self.default_rot_thresh
        if isinstance(rot_thresh, float):
            rot_thresh = [rot_thresh] * 3
        if vel_thresh is None:
            vel_thresh = self.default_vel_thresh
        if isinstance(vel_thresh, float):
            vel_thresh = [vel_thresh] * 3
        if target_pos is None:
            target_pos = self.getCogPos()
            pos_thresh = np.array([1e6] * 3)
            vel_thresh = np.array([1e6] * 3)
        if target_rot is None:
            target_rot = self.getBaseRot()
            rot_thresh = np.array([1e6] * 3)
        if len(target_rot) == 3:
            target_rot = quaternion_from_euler(*target_rot)
        current_pos = self.getCogPos()
        current_vel = self.getCogLinVel()
        current_rot = self.getBaseRot()
        delta_pos = target_pos - current_pos
        delta_vel = current_vel
        delta_rot = quaternion_multiply(quaternion_inverse(current_rot), target_rot)
        delta_rot = euler_from_quaternion(delta_rot)
        self.get_logger().info_throttle(1.0, "pose diff: {}, rot: {}, vel: {}; target: pos: {}, rot: {}; current: pos: {}, rot: {}, vel: {}"
                                         .format(delta_pos, delta_rot, delta_vel, target_pos, target_rot, current_pos, current_rot, current_vel))
        if np.all(np.abs(delta_pos) < pos_thresh) and np.all(np.abs(delta_rot) < rot_thresh) and np.all(np.abs(delta_vel) < vel_thresh):
            return True
        else:
            return False

    def rotateCog(self, roll, pitch):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = roll
        msg.vector.y = pitch
        self.final_rot_pub.publish(msg)

    def getTF(self, frame_id, wait=0.5, parent_frame_id='world'):
        try:
            trans = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, self.get_clock().now().to_msg(), rclpy.duration.Duration(seconds=wait))
            return trans
        except Exception as e:
            self.get_logger().error("TF lookup error: {}".format(e))
            return None

    def getJointState(self):
        return self.joint_state

    def setJointAngle(self, target_joint_names, target_joint_angles, thresh=0.05, timeout=-1):
        if self.flight_state != self.HOVER_STATE and self.flight_state != self.ARM_OFF_STATE:
            self.get_logger().error("[Send Joint] flight state ({}) disallows joint motion".format(self.flight_state))
            return False
        if len(target_joint_names) != len(target_joint_angles):
            self.get_logger().error("[Send Joint] sizes of joint names ({}) and angles ({}) do not match"
                                    .format(len(target_joint_names), len(target_joint_angles)))
            return False
        for name in target_joint_names:
            if name not in self.joint_state.name:
                self.get_logger().error("set joint angle: cannot find {}".format(name))
                return False
        target_joint_state = JointState()
        target_joint_state.name = target_joint_names
        target_joint_state.position = target_joint_angles
        self.joint_ctrl_pub.publish(target_joint_state)
        return self.jointConvergenceCheck(timeout, target_joint_names, target_joint_angles, thresh)

    def jointConvergenceCheck(self, timeout, target_joint_names, target_joint_angles, thresh):
        return self.convergenceCheck(timeout, self.jointThresholdCheck, target_joint_names, target_joint_angles, thresh)

    def jointThresholdCheck(self, target_joint_names, target_joint_angles, thresh):
        if len(target_joint_names) != len(target_joint_angles):
            self.get_logger().error("[Send Joint] sizes of joint names ({}) and angles ({}) do not match"
                                    .format(len(target_joint_names), len(target_joint_angles)))
            return False
        index_map = []
        for name in target_joint_names:
            try:
                j = self.joint_state.name.index(name)
            except ValueError:
                self.get_logger().error("set joint angle: cannot find {}".format(name))
                return False
            index_map.append(j)
        delta_ang = []
        for index, target_ang in zip(index_map, target_joint_angles):
            current_ang = self.joint_state.position[index]
            delta_ang.append(target_ang - current_ang)
        self.get_logger().info_throttle(1.0, "delta angle: {}".format(delta_ang))
        if np.all(np.abs(delta_ang) < thresh):
            return True
        else:
            return False

    def setJointTorque(self, state):
        req = SetBool.Request()
        req.data = state
        future = self.set_joint_torque_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed")
            return None

if __name__ == "__main__":
    from IPython import embed
    rclpy.init()
    ri = RobotInterface(robot_ns="")
    embed()
    rclpy.shutdown()
