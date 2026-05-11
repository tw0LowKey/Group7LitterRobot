#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # <--- Add this

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class FakeInterruptTestNode(Node):
    def __init__(self):
        super().__init__("fake_interrupt_test_node")
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.litter_pub = self.create_publisher(
            PoseStamped,
            "/vision/detected_litter",
            latching_qos
        )

        self.stop_pub = self.create_publisher(
            Bool,
            "/stop_navigation",
            latching_qos
        )

        self.start_client = self.create_client(
            Trigger,
            "/start_navigation"
        )

        self.step = 0

        self.get_logger().info("Fake interrupt test node started.")

        self.timer = self.create_timer(1.0, self.timer_callback)

    def call_start_navigation(self):
        if not self.start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/start_navigation service not available")
            return

        req = Trigger.Request()
        future = self.start_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info("Called /start_navigation")
        )

    def publish_litter(self):
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = 1.0
        msg.pose.position.y = 0.50
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.litter_pub.publish(msg)

        self.get_logger().info("Published fake litter at x=1.0, y= 0.5")

    def stop_robot(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)
        self.get_logger().info("Published stop_navigation = True")

    def timer_callback(self):
        self.step += 1

        # Start navigation after 2 seconds
        if self.step == 2:
            self.call_start_navigation()

        # Publish fake litter near the start
        if self.step == 3:
            self.publish_litter()

        # First pause
        if self.step == 180:
            self.stop_robot()

        # Resume after 10 seconds
        if self.step == 190:
            self.call_start_navigation()

        # Second pause
        if self.step == 360:
            self.stop_robot()

        # Resume after 10 seconds
        if self.step == 370:
            self.call_start_navigation()

        # Stop this fake test node after sequence
        if self.step == 480:
            self.get_logger().info("Fake interrupt test complete.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FakeInterruptTestNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()