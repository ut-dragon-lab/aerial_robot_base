#!/usr/bin/env python3
import sys
import select
import termios
import tty
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int8, UInt16
from aerial_robot_msgs.msg import PoseControlPid

msg_text = """
s: start to subscribe the topic regarding to control errors, and start to calculate the RMS
h: stop the calculate the output the RMS results.
"""

class RMSNode(Node):
    def __init__(self):
        super().__init__('rms_error')
        self.subscription = self.create_subscription(
            PoseControlPid,
            'debug/pose/pid',
            self.cb,
            10
        )
        self.start_flag = False
        self.pose_squared_errors_sum = [0.0] * 6
        self.pose_cnt = 0

    def cb(self, data):
        if self.start_flag:
            self.pose_cnt += 1
            self.pose_squared_errors_sum[0] += data.x.err_p * data.x.err_p
            self.pose_squared_errors_sum[1] += data.y.err_p * data.y.err_p
            self.pose_squared_errors_sum[2] += data.z.err_p * data.z.err_p

            self.pose_squared_errors_sum[3] += data.roll.err_p * data.roll.err_p
            self.pose_squared_errors_sum[4] += data.pitch.err_p * data.pitch.err_p
            self.pose_squared_errors_sum[5] += data.yaw.err_p * data.yaw.err_p

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printMsg(msg_str, msg_len=50):
    print(msg_str.ljust(msg_len) + "\r", end="")

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = RMSNode()
    print(msg_text)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            key = getKey(settings)
            if key == 's':
                node.get_logger().info("start to calculate RMS errors")
                node.start_flag = True
            elif key == 'h':
                node.get_logger().info("stop calculation")
                rms = [0.0] * 6
                if node.pose_cnt > 0:
                    rms = [math.sqrt(val / node.pose_cnt) for val in node.pose_squared_errors_sum]
                node.get_logger().info("RMS of pos errors: [%f, %f, %f], att errors: [%f, %f, %f]",
                                       rms[0], rms[1], rms[2], rms[3], rms[4], rms[5])
                node.start_flag = False
                node.pose_cnt = 0
                node.pose_squared_errors_sum = [0.0] * 6
            elif key == '\x03':  # Ctrl+C
                break
            time.sleep(0.001)
    except Exception as e:
        print(e)
        print(repr(e))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
