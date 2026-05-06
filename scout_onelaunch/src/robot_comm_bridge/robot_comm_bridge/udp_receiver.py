#!/usr/bin/env python3

import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class UdpReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        self.declare_parameter('listen_ip', '0.0.0.0')
        self.declare_parameter('listen_port', 5005)

        self.listen_ip = self.get_parameter('listen_ip').value
        self.listen_port = int(self.get_parameter('listen_port').value)

        self.pose_pub = self.create_publisher(PoseStamped, '/other_robot_pose', 10)
        self.status_pub = self.create_publisher(String, '/other_robot_status', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.receive_data)

        self.get_logger().info(
            f'UDP receiver started. Listening on {self.listen_ip}:{self.listen_port}'
        )

    def receive_data(self):
        try:
            packet, _ = self.sock.recvfrom(4096)
        except BlockingIOError:
            return

        try:
            data = json.loads(packet.decode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Invalid UDP packet: {e}')
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = float(data.get("x", 0.0))
        pose_msg.pose.position.y = float(data.get("y", 0.0))
        pose_msg.pose.position.z = float(data.get("z", 0.0))

        pose_msg.pose.orientation.x = float(data.get("qx", 0.0))
        pose_msg.pose.orientation.y = float(data.get("qy", 0.0))
        pose_msg.pose.orientation.z = float(data.get("qz", 0.0))
        pose_msg.pose.orientation.w = float(data.get("qw", 1.0))

        self.pose_pub.publish(pose_msg)

        status_msg = String()
        status_msg.data = str(data.get("status", "unknown"))
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UdpReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
