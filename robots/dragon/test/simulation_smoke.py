#!/usr/bin/env python3
"""Exercise an already running headless DRAGON simulation through real scripts.

Source the workspace, start bringup_launch.py, then run this file with --output DIR.
The test uses keyboard_command.py over a pseudo-terminal and executes the installed
transformation_demo.py. It verifies measured flight state, CoG position and joints.
"""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8


class FlightTest(Node):
    def __init__(self, output):
        super().__init__('dragon_simulation_test')
        self.out = output
        self.state = None
        self.cog = None
        self.base = None
        self.joints = {}
        self.stage = 'ready'
        self.sim = 0.0
        self.last_record = 0.0
        self.trace = (output / 'telemetry.jsonl').open('w', buffering=1)
        self.events = []
        self.create_subscription(UInt8, '/dragon/flight_state', self.on_state, 10)
        self.create_subscription(Odometry, '/dragon/uav/cog/odom', self.on_cog, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/dragon/ground_truth', self.on_base, qos_profile_sensor_data)
        self.create_subscription(JointState, '/dragon/joint_states', self.on_joints, qos_profile_sensor_data)

    def on_state(self, msg):
        if msg.data != self.state:
            print(f'STATE {self.state} -> {msg.data}, sim={self.sim:.2f}', flush=True)
        self.state = msg.data

    @staticmethod
    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return [p.x,p.y,p.z,q.x,q.y,q.z,q.w]

    def on_cog(self, msg):
        self.cog = self.pose(msg)
        self.sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_base(self, msg):
        self.base = self.pose(msg)

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        if time.monotonic()-self.last_record > 0.1:
            self.trace.write(json.dumps(dict(wall=time.time(), sim=self.sim, stage=self.stage,
                state=self.state, cog=self.cog, ground_truth=self.base, joints=self.joints))+'\n')
            self.last_record = time.monotonic()

    def wait(self, predicate, timeout):
        deadline = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin()
            if predicate(): return
            if self.state in (16,17): raise RuntimeError(f'Failsafe state {self.state}')
        raise RuntimeError(f'Timeout at {self.stage}, state={self.state}')

    def event(self, name):
        self.stage=name
        self.events.append(dict(stage=name,wall=time.time(),sim=self.sim))
        print('STAGE',name,'sim=',self.sim,flush=True)

    def check_hover(self):
        if self.state != 5: raise RuntimeError(f'Lost HOVER in {self.stage}: {self.state}')
        if abs(self.cog[2]-1.5)>0.3: raise RuntimeError(f'Altitude excursion: {self.cog[2]}')
        if math.hypot(self.cog[0]-self.origin[0],self.cog[1]-self.origin[1])>0.4:
            raise RuntimeError('Horizontal position excursion > 0.4 m')
        x,y,z,w=self.cog[3:]
        roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=math.asin(max(-1,min(1,2*(w*y-z*x))))
        if max(abs(roll),abs(pitch))>0.35:
            raise RuntimeError(f'CoG attitude excursion: roll={roll}, pitch={pitch}')

    def hold(self, duration):
        start=self.sim
        deadline=time.monotonic()+duration*5+20
        while self.sim-start<duration:
            self.spin(); self.check_hover()
            if time.monotonic()>deadline: raise RuntimeError('Simulation clock stalled')

    def demo(self, name, args, target):
        self.event(name)
        start = self.sim
        with (self.out/(name+'.log')).open('w') as log:
            proc=subprocess.Popen(['ros2','run','dragon','transformation_demo.py','--ros-args',
                '-p','use_sim_time:=true']+args, stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+450
            try:
                while proc.poll() is None:
                    self.spin();self.check_hover()
                    if self.sim-start>12.0:
                        raise RuntimeError('Default 5-second demo exceeded 12 simulation seconds')
                    if time.monotonic()>deadline:raise RuntimeError('Demo wall timeout')
                if proc.returncode:raise RuntimeError(f'{name} exited {proc.returncode}')
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
        names=[f'joint{i}_{a}' for i in range(1,4) for a in ('pitch','yaw')]
        error=max(abs(self.joints[n]-p) for n,p in zip(names,target))
        if error>0.03:raise RuntimeError(f'{name} joint error {error}')
        self.events.append(dict(stage=name+'_complete',sim=self.sim,wall=time.time(),max_joint_error=error, duration=self.sim-start))
        self.event(name+'_hold');self.hold(20)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=FlightTest(out)
    master,slave=pty.openpty()
    log=(out/'keyboard.log').open('w')
    kb=subprocess.Popen(['ros2','run','aerial_robot_core','keyboard_command.py','--ros-args',
                         '-p','robot_ns:=/dragon'],stdin=slave,stdout=log,stderr=subprocess.STDOUT)
    os.close(slave)
    result={'success':False}
    try:
        n.wait(lambda:n.state==0 and n.cog is not None and len(n.joints)>=18,60)
        n.origin=n.cog[:3]
        # Allow DDS discovery and IMU calibration before pressing keys.
        start=n.sim;n.wait(lambda:n.sim-start>5,30)
        n.event('arming');os.write(master,b'r');n.wait(lambda:n.state==2,20)
        n.event('takeoff');os.write(master,b't');n.wait(lambda:n.state==5,120)
        n.event('initial_hover');n.hold(30)
        n.demo('mode0',['-p','mode:=0'],[-0.6,-1.56,-0.6,0.0,0.0,1.56])
        n.demo('reset',['-p','reset:=true'],[0.0,1.56]*3)
        n.demo('mode1',['-p','mode:=1'],[-0.8,1.2]*3)
        n.demo('reset_final',['-p','reset:=true'],[0.0,1.56]*3)
        n.event('landing');os.write(master,b'l');n.wait(lambda:n.state==0,120)
        result['success']=True
    except Exception as exc:
        result['error']=str(exc);print('FAILED',exc,flush=True)
        os.write(master,b'h')
        until=time.monotonic()+2
        while time.monotonic()<until:n.spin()
    finally:
        result.update(events=n.events,final_state=n.state,final_cog=n.cog)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        os.write(master,b'\x03')
        try:kb.wait(timeout=5)
        except subprocess.TimeoutExpired:kb.terminate();kb.wait(timeout=5)
        os.close(master);log.close();n.trace.close();n.destroy_node();rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
