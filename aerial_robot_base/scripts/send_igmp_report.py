#!/usr/bin/env python3
import socket
import struct
import netifaces
import datetime

import rclpy
from rclpy.node import Node

class SendIgmpReportNode(Node):
    def __init__(self):
        super().__init__('send_igmp_report')
        self.group_ip = "239.255.42.99"
        interface_ip = self.get_wifi_ip()
        if interface_ip:
            self.send_igmp_v2_membership_report(self.group_ip, interface_ip)
        else:
            self.get_logger().error("Wi-Fi interface not found.")

    def get_wifi_ip(self):
        gateways = netifaces.gateways()
        default_ipv4 = gateways.get('default', {}).get(netifaces.AF_INET)
        if default_ipv4:
            interface_name = default_ipv4[1]
            addresses = netifaces.ifaddresses(interface_name).get(netifaces.AF_INET)
            if addresses:
                return addresses[0]['addr']
        return None

    def send_igmp_v2_membership_report(self, group_ip, interface_ip):
        IGMP_MEMBERSHIP_REPORT_TYPE = 0x16  # IGMPv2 Membership Report
        MAX_RESP_TIME = 0   # Membership Report では使用しません
        CHECKSUM_PLACEHOLDER = 0

        group_bytes = socket.inet_aton(group_ip)

        igmp_packet = struct.pack('!BBH4s',
                                  IGMP_MEMBERSHIP_REPORT_TYPE,
                                  MAX_RESP_TIME,
                                  CHECKSUM_PLACEHOLDER,
                                  group_bytes)

        def calculate_checksum(data):
            if len(data) % 2:
                data += b'\x00'
            checksum = sum(struct.unpack('!%dH' % (len(data) // 2), data))
            checksum = (checksum >> 16) + (checksum & 0xFFFF)
            checksum += checksum >> 16
            return ~checksum & 0xFFFF

        checksum = calculate_checksum(igmp_packet)

        igmp_packet = struct.pack('!BBH4s',
                                  IGMP_MEMBERSHIP_REPORT_TYPE,
                                  MAX_RESP_TIME,
                                  checksum,
                                  group_bytes)

        sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_IGMP)
        sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(interface_ip))
        sock.sendto(igmp_packet, (group_ip, 0))
        dt_now = datetime.datetime.now()
        self.get_logger().info("[{}] IGMPv2 Membership Report sent to {} from {}".format(
            dt_now, group_ip, interface_ip))

def main(args=None):
    rclpy.init(args=args)
    node = SendIgmpReportNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
