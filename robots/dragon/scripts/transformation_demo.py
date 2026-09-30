#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Pose targets ported from JSK's ROS 1 dragon/scripts/transformation_demo.py.
"""Transform a hovering DRAGON gradually; verify measured joint convergence."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
from geometry_msgs.msg import Vector3Stamped


class TransformationDemo(Node):
    def __init__(self):
        super().__init__('dragon_transformation_demo')
        for k, v in {'mode': 0, 'reset': False, 'reverse_reset': False,
                     'robot_ns': '/dragon', 'duration': 5.0, 'settle_timeout': 15.0}.items():
            self.declare_parameter(k, v)
        ns = self.get_parameter('robot_ns').value.rstrip('/')
        self.joint_pub = self.create_publisher(JointState, ns + '/joints_ctrl', 10)
        self.att_pub = self.create_publisher(Vector3Stamped, ns + '/final_target_baselink_rpy', 10)
        self.names = [f'joint{i}_{a}' for i in range(1, 4) for a in ('pitch', 'yaw')]
        self.positions = {}
        self.flight_state = None
        self.create_subscription(JointState, ns + '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(UInt8, ns + '/flight_state', self.on_state, 10)

    def on_joints(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def on_state(self, msg):
        self.flight_state = msg.data

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def run(self):
        mode = self.get_parameter('mode').value
        poses = {0: ([-0.6, -1.56, -0.6, 0.0, 0.0, 1.56], [0.0, -0.3, 0.0]),
                 1: ([-0.8, 1.2, -0.8, 1.2, -0.8, 1.2], [0.4, -0.4, 0.0]),
                 2: ([0.0, 1.56, -1.5, 0.0, 0.0, 1.56], [0.0, 0.75, 0.0])}
        if mode not in poses:
            raise ValueError('mode must be 0, 1 or 2')
        target, attitude = poses[mode]
        if self.get_parameter('reset').value:
            target, attitude = [0.0, 1.56] * 3, [0.0, 0.0, 0.0]
        if self.get_parameter('reverse_reset').value:
            target, attitude = [0.0, -1.56] * 3, [0.0, 0.0, 0.0]
        duration = self.get_parameter('duration').value
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError('duration must be finite and positive')
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.flight_state == 5 and all(n in self.positions for n in self.names)
                    and self.joint_pub.get_subscription_count() and self.att_pub.get_subscription_count()):
                break
        else:
            raise RuntimeError('No hovering robot, measured joints or command subscribers within 30 seconds')
        initial = [self.positions[n] for n in self.names]
        # Respect the requested trajectory time. The ROS 1 demo sends a step
        # target; its effort servo differs from this simulation velocity servo.
        start = self.now()
        wall_deadline = time.monotonic() + 5 * (duration + self.get_parameter('settle_timeout').value) + 30
        self.get_logger().info(f'Transforming mode={mode}, target={target}, duration={duration}s')
        while rclpy.ok() and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.01)
            if self.flight_state != 5:
                raise RuntimeError(f'Robot left HOVER: flight_state={self.flight_state}')
            elapsed = self.now() - start
            u = min(max(elapsed / duration, 0.0), 1.0)
            blend = u*u*(3.0-2.0*u)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.names
            msg.position = [a + blend*(b-a) for a,b in zip(initial,target)]
            self.joint_pub.publish(msg)
            att = Vector3Stamped()
            att.header = msg.header
            att.vector.x, att.vector.y, att.vector.z = [float(v) for v in attitude]
            self.att_pub.publish(att)
            error = max(abs(self.positions[n]-p) for n,p in zip(self.names,target))
            if elapsed >= duration + 2.0 and error < 0.03:
                self.get_logger().info(f'Transformation complete; max measured joint error={error:.5f} rad')
                return
            if elapsed > duration + self.get_parameter('settle_timeout').value:
                raise RuntimeError(f'Joint convergence timeout: max error={error:.5f} rad')
        raise RuntimeError('Simulation clock stalled or transformation timed out')


def main():
    rclpy.init()
    node = TransformationDemo()
    status = 0
    try:
        node.run()
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
        status = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
