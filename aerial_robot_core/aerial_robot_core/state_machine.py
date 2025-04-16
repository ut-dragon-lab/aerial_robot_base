#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Software License Agreement (BSD License)

Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
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

import time
import smach
import smach_ros
import numpy as np
import json
import rclpy

from aerial_robot_core.robot_interface import RobotInterface
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

def nsecToSec(nsec):
    return nsec / 1e9

class BaseState(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=['flags', 'extra']):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.robot = robot

    def hold(self, t, flags):
        if 'interfere_mode' not in flags:
            time.sleep(t)
        else:
            if not flags['interfere_mode']:
                time.sleep(t)
            else:
                message = '\n\n Please press "L1" and "R1" simultaneously to proceed the state \n'
                self.robot.get_logger().info(message)
                while rclpy.ok():
                    if flags['interfering']:
                        flags['interfering'] = False
                        break
                    time.sleep(0.1)

# Start
class Start(BaseState):
    def __init__(self, robot):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.flags = {'interfere_mode': False, 'interfering': False}
        self.task_start = False
        self.task_start_sub = robot.create_subscription(Empty, '/task_start', self.taskStartCallback, 10)
        self.joy_sub = robot.create_subscription(Joy, robot.robot_ns + '/joy', self.joyCallback, 10)

    def taskStartCallback(self, msg):
        self.task_start = True

    def joyCallback(self, msg):
        interfere_flag = False
        if len(msg.axes) == 14 and len(msg.buttons) == 14:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:
                interfere_flag = True
        if len(msg.axes) == 8 and len(msg.buttons) == 11:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:
                interfere_flag = True
        if interfere_flag and not self.flags['interfere_mode']:
            self.robot.get_logger().info("Enter interfere mode")
            self.flags['interfere_mode'] = True
        self.flags['interfering'] = interfere_flag

    def execute(self, userdata):
        message = (
            '\n\n  Please run the following command to start the state machine: \n'
            '  "$ ros2 topic pub -1 /task_start std_msgs/msg/Empty \'{}\'" \n \n'
            '  Or press "L1" and "R1" simultaneously to enter interfere mode, \n'
            '  which enables manual progression via a controller (e.g., PS4).\n'
        )
        self.robot.get_logger().info(message)
        userdata.flags = self.flags
        while not self.task_start:
            rclpy.spin_once(self.robot, timeout_sec=0.01)
            time.sleep(0.1)
            self.robot.get_logger().debug("wait to start task")
            if userdata.flags['interfere_mode']:
                self.robot.get_logger().info(self.__class__.__name__ + ': enter interference mode')
                userdata.flags['interfering'] = False
                break
            if not rclpy.ok():
                self.robot.get_logger().error("ROS shutdown")
                return 'preempted'
        self.robot.get_logger().info("start task")
        return 'succeeded'

# SingleCommandState
class SingleCommandState(BaseState):
    def __init__(self, robot, prefix, func, start_flight_state, target_flight_state, timeout, hold_time):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.prefix = prefix
        self.func = func
        self.start_flight_state = start_flight_state
        self.target_flight_state = target_flight_state
        self.hold_time = hold_time
        self.timeout = timeout

    def execute(self, userdata):
        if self.robot.getFlightState() != self.start_flight_state:
            self.robot.get_logger().warning(
                "{}: robot state ({}) is not the required start state ({}). preempted!"
                .format(self.__class__.__name__, self.robot.getFlightState(), self.start_flight_state)
            )
            return 'preempted'
        self.robot.get_logger().info("{}: start {}".format(self.__class__.__name__, self.prefix))
        self.func()
        start_t = nsecToSec(self.robot.get_clock().now().nanoseconds)
        while nsecToSec(self.robot.get_clock().now().nanoseconds) < start_t + self.timeout:
            if self.robot.getFlightState() == self.target_flight_state:
                self.robot.get_logger().info("{}: robot succeeded to {}!".format(self.__class__.__name__, self.prefix))
                self.hold(self.hold_time, userdata.flags)
                return 'succeeded'
            if not rclpy.ok():
                self.robot.get_logger().error("ROS shutdown during state execution")
                return 'preempted'
            time.sleep(0.1)
        self.robot.get_logger().warning("{}: timeout ({} sec). preempted!"
                                          .format(self.__class__.__name__, self.timeout))
        return 'preempted'

class Arm(SingleCommandState):
    def __init__(self, robot):
        super().__init__(robot, 'arm', robot.start, robot.ARM_OFF_STATE, robot.ARM_ON_STATE, 2.0, 2.0)
    def execute(self, userdata):
        if self.robot.getFlightState() == self.robot.HOVER_STATE:
            self.robot.get_logger().info(self.__class__.__name__ + ': robot already hovers, skip')
            return 'succeeded'
        return super().execute(userdata)

class Takeoff(SingleCommandState):
    def __init__(self, robot):
        super().__init__(robot, 'takeoff', robot.takeoff, robot.ARM_ON_STATE, robot.HOVER_STATE, 30.0, 2.0)
    def execute(self, userdata):
        if self.robot.getFlightState() == self.robot.HOVER_STATE:
            self.robot.get_logger().info(self.__class__.__name__ + ': robot already hovers, skip')
            return 'succeeded'
        return super().execute(userdata)

class Land(SingleCommandState):
    def __init__(self, robot):
        super().__init__(robot, 'land', robot.land, robot.HOVER_STATE, robot.ARM_OFF_STATE, 20.0, 0)

# WayPoint
class WayPoint(BaseState):
    def __init__(self, robot, prefix='waypoint', waypoints=[], timeout=30.0, hold_time=1.0):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.waypoints = waypoints
        self.timeout = timeout
        self.hold_time = hold_time
        self.pos_thresh = 0.1
        self.vel_thresh = 0.05
        self.yaw_thresh = 0.1

    def execute(self, userdata):
        if self.robot.getFlightState() != self.robot.HOVER_STATE:
            self.robot.get_logger().warning("{}: robot state ({}) is not HOVER_STATE. preempted!"
                                              .format(self.__class__.__name__, self.robot.getFlightState()))
            return 'preempted'
        if len(self.waypoints) == 0:
            self.robot.get_logger().warning("{}: waypoints are empty. preempted"
                                              .format(self.__class__.__name__))
            return 'preempted'
        for i, waypoint in enumerate(self.waypoints):
            if len(waypoint) == 3:
                ret = self.robot.goPos(pos=waypoint, pos_thresh=self.pos_thresh,
                                       vel_thresh=self.vel_thresh, timeout=self.timeout)
            elif len(waypoint) == 4:
                ret = self.robot.goPosYaw(pos=waypoint[:3], yaw=waypoint[-1],
                                          pos_thresh=self.pos_thresh, vel_thresh=self.vel_thresh,
                                          yaw_thresh=self.yaw_thresh, timeout=self.timeout)
            else:
                self.robot.get_logger().warning("{}: waypoint {} format unsupported (length must be 3 or 4). preempted!"
                                                  .format(self.__class__.__name__, waypoint))
                return 'preempted'
            if ret:
                self.robot.get_logger().info("{}: reached {}th waypoint [{}]".format(self.__class__.__name__, i+1, waypoint))
                self.hold(self.hold_time, userdata.flags)
            else:
                self.robot.get_logger().warning("{}: failed to reach waypoint {}. preempted".format(self.__class__.__name__, waypoint))
                return 'preempted'
        return 'succeeded'

class CircleTrajectory(BaseState):
    def __init__(self, robot, period=20.0, radius=1.0, init_theta=0.0, yaw=True, loop=1, timeout=30.0, hold_time=2.0):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.period = period
        self.radius = radius
        self.init_theta = init_theta
        self.yaw = yaw
        self.loop = loop
        self.hold_time = hold_time
        self.omega = 2 * np.pi / self.period
        self.velocity = self.omega * self.radius
        nav_rate_hz = 20.0
        self.nav_period = 1.0 / nav_rate_hz

    def execute(self, userdata):
        current_pos = self.robot.getCogPos()
        center_pos_x = current_pos[0] - np.cos(self.init_theta) * self.radius
        center_pos_y = current_pos[1] - np.sin(self.init_theta) * self.radius
        center_pos_z = current_pos[2]
        init_yaw = self.robot.getCogRPY()[2]
        self.robot.get_logger().info("Center position is: [{:.2f}, {:.2f}]".format(center_pos_x, center_pos_y))
        loop = 0
        cnt = 0
        while loop < self.loop:
            if self.robot.getFlightState() != self.robot.HOVER_STATE:
                self.robot.get_logger().error("[CircleTrajectory] robot not hovering, preempted!")
                return 'preempted'
            theta = self.init_theta + cnt * self.nav_period * self.omega
            pos_x = center_pos_x + np.cos(theta) * self.radius
            pos_y = center_pos_y + np.sin(theta) * self.radius
            pos = [pos_x, pos_y, center_pos_z]
            vel_x = -np.sin(theta) * self.velocity
            vel_y = np.cos(theta) * self.velocity
            vel = [vel_x, vel_y, 0]
            if self.yaw:
                yaw_val = init_yaw + cnt * self.nav_period * self.omega
                rot = [0, 0, yaw_val]
                ang_vel = [0, 0, self.omega]
                self.robot.navigate(pos=pos, rot=rot, lin_vel=vel, ang_vel=ang_vel)
            else:
                self.robot.navigate(pos=pos, lin_vel=vel)
            cnt += 1
            if cnt >= (self.period / self.nav_period):
                loop += 1
                cnt = 0
            time.sleep(self.nav_period)
        self.robot.navigate(lin_vel=[0,0,0], ang_vel=[0,0,0])
        self.hold(self.hold_time, userdata.flags)
        return 'succeeded'

class FormCheck(BaseState):
    def __init__(self, robot, prefix='form_check', joint_names=[], joint_angles=[], thresh=0.02, timeout=10.0):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.target_joint_names = joint_names
        self.target_joint_angles = joint_angles
        self.timeout = timeout
        self.thresh = thresh

    def execute(self, userdata):
        ret = self.robot.jointConvergenceCheck(self.timeout, self.target_joint_names, self.target_joint_angles, self.thresh)
        if ret:
            self.robot.get_logger().info("{}: converged to target joints {}: {} successfully!"
                                           .format(self.__class__.__name__, self.target_joint_names, self.target_joint_angles))
            return 'succeeded'
        else:
            self.robot.get_logger().warning("{}: timeout ({} sec). preempted!"
                                              .format(self.__class__.__name__, self.timeout))
            return 'preempted'

class Transform(BaseState):
    def __init__(self, robot, prefix='transform', joint_names=[], joint_trajectory=[], thresh=0.05, timeout=10.0, hold_time=2.0):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.target_joint_names = joint_names
        self.target_joint_trajectory = joint_trajectory
        self.timeout = timeout
        self.hold_time = hold_time
        self.thresh = thresh

    def execute(self, userdata):
        if self.robot.getFlightState() != self.robot.HOVER_STATE:
            self.robot.get_logger().warning("{}: robot state ({}) is not HOVER_STATE. preempted!"
                                              .format(self.__class__.__name__, self.robot.getFlightState()))
            return 'preempted'
        if len(self.target_joint_trajectory) == 0:
            self.robot.get_logger().warning("{}: joint trajectory empty. preempted".format(self.__class__.__name__))
            return 'preempted'
        for i, target_angles in enumerate(self.target_joint_trajectory):
            ret = self.robot.setJointAngle(self.target_joint_names, target_angles, self.thresh, self.timeout)
            if ret:
                self.robot.get_logger().info("{}: converged to {}th target joints {}: {} successfully!"
                                               .format(self.__class__.__name__, i+1, self.target_joint_names, target_angles))
                self.hold(self.hold_time, userdata.flags)
            else:
                self.robot.get_logger().warning("{}: timeout ({} sec), failed to reach {}th target joints. preempted!"
                                                  .format(self.__class__.__name__, self.timeout, i+1))
                return 'preempted'
        return 'succeeded'

class TransformWithPose(BaseState):
    def __init__(self, robot, prefix='motion', joint_names=[], joint_trajectory=[], pos_trajectory=[], rot_trajectory=[],
                 joint_thresh=0.05, pos_thresh=0.1, rot_thresh=0.1, timeout=10.0, hold_time=2.0, rotate_cog=False):
        super().__init__(robot, outcomes=['succeeded', 'preempted'])
        self.target_joint_names = joint_names
        self.target_joint_trajectory = joint_trajectory
        self.target_pos_trajectory = pos_trajectory
        self.target_rot_trajectory = rot_trajectory
        self.timeout = timeout
        self.hold_time = hold_time
        self.joint_thresh = joint_thresh
        self.pos_thresh = pos_thresh
        self.rot_thresh = rot_thresh
        self.rotate_cog = rotate_cog

    def execute(self, userdata):
        if self.robot.getFlightState() != self.robot.HOVER_STATE:
            self.robot.get_logger().warning("{}: robot state ({}) is not HOVER_STATE. preempted!"
                                              .format(self.__class__.__name__, self.robot.getFlightState()))
            return 'preempted'
        if len(self.target_joint_trajectory) == 0:
            self.robot.get_logger().warning("{}: joint trajectory empty. preempted".format(self.__class__.__name__))
            return 'preempted'
        for i, target_angles in enumerate(self.target_joint_trajectory):
            target_pos = None
            if len(self.target_pos_trajectory) > 0:
                if len(self.target_pos_trajectory) != len(self.target_joint_trajectory):
                    self.robot.get_logger().warning("{}: pos trajectory size mismatch. preempted!"
                                                      .format(self.__class__.__name__))
                    return 'preempted'
                target_pos = self.target_pos_trajectory[i]
                self.robot.goPos(target_pos, timeout=0)
            target_rot = None
            if len(self.target_rot_trajectory) > 0:
                if len(self.target_rot_trajectory) != len(self.target_joint_trajectory):
                    self.robot.get_logger().warning("{}: rot trajectory size mismatch. preempted!"
                                                      .format(self.__class__.__name__))
                    return 'preempted'
                target_rot = self.target_rot_trajectory[i]
                if self.rotate_cog:
                    if not (2 <= len(target_rot) <= 3):
                        self.robot.get_logger().warning("{}: target rot size must be 2 or 3 for cog rotate mode. preempted!"
                                                          .format(self.__class__.__name__))
                        return 'preempted'
                    self.robot.rotateCog(target_rot[0], target_rot[1])
                    if len(target_rot) == 3:
                        self.robot.rotateYaw(target_rot[2], timeout=0)
                    else:
                        curr_yaw = self.robot.getCogRPY()[2]
                        target_rot.append(curr_yaw)
                else:
                    self.robot.rotate(target_rot, timeout=0)
            start_time = nsecToSec(self.robot.get_clock().now().nanoseconds)
            self.robot.get_logger().info("{}: starting joint motion {} with pose [{}, {}]".format(
                self.__class__.__name__, target_angles, target_pos, target_rot))
            ret = self.robot.setJointAngle(self.target_joint_names, target_angles, self.joint_thresh, self.timeout)
            if ret:
                self.robot.get_logger().info("{}: converged to {}th target joints {}: {} successfully!"
                                               .format(self.__class__.__name__, i+1, self.target_joint_names, target_angles))
            else:
                self.robot.get_logger().warning("{}: timeout ({} sec), failed to reach {}th target joints. preempted!"
                                                  .format(self.__class__.__name__, self.timeout, i+1))
                return 'preempted'
            elapsed = nsecToSec(self.robot.get_clock().now().nanoseconds) - start_time
            timeout_remaining = self.timeout - elapsed
            ret = self.robot.poseConvergenceCheck(timeout_remaining, target_pos, target_rot, self.pos_thresh, rot_thresh=self.rot_thresh)
            if not ret:
                self.robot.get_logger().warning("{}: timeout ({} sec), failed to converge to {}th target pose. preempted!"
                                                  .format(self.__class__.__name__, self.timeout, i+1))
                return 'preempted'
            self.robot.get_logger().info("{}: converged to {}th target pose {}: [{}, {}] successfully!"
                                           .format(self.__class__.__name__, i+1, self.target_joint_names, target_pos, target_rot))
            self.hold(self.hold_time, userdata.flags)
        return 'succeeded'
