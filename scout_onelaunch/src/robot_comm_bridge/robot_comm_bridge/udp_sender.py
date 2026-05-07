#!/usr/bin/env python3

import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class UdpSender(Node):
    def __init__(self):
        super().__init__('udp_sender')

        self.declare_parameter('target_ip', '192.168.1.21')
        self.declare_parameter('target_port', 5005)
        self.declare_parameter('robot_name', 'robot1')

        self.target_ip = self.get_parameter('target_ip').value
        self.target_port = int(self.get_parameter('target_port').value)
        self.robot_name = self.get_parameter('robot_name').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.latest_pose = None
        self.latest_status = "unknown"

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            String,
            '/status',
            self.status_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.send_data)

        self.get_logger().info(
            f'UDP sender started. Sending to {self.target_ip}:{self.target_port}'
        )

    def pose_callback(self, msg):
        self.latest_pose = msg

    def status_callback(self, msg):
        self.latest_status = msg.data

    def send_data(self):
        if self.latest_pose is None:
            return

        pose = self.latest_pose.pose.pose

        data = {
            "robot_name": self.robot_name,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
            "status": self.latest_status,
        }

        packet = json.dumps(data).encode('utf-8')
        self.sock.sendto(packet, (self.target_ip, self.target_port))


def main(args=None):
    rclpy.init(args=args)
    node = UdpSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
