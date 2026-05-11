#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


class Go4mPrintGps(Node):
    def __init__(self):
        super().__init__("go_4m_print_gps")

        self.odom = None
        self.gps = None

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.timer = self.create_timer(1.0, self.print_status)
        self.started = False

        self.get_logger().info("Waiting for /odom, /gps/fix and Nav2 action server...")

    def odom_callback(self, msg):
        self.odom = msg

        if not self.started and self.gps is not None:
            self.started = True
            self.send_goal_4m_ahead()

    def gps_callback(self, msg):
        self.gps = msg

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def send_goal_4m_ahead(self):
        if self.odom is None:
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 /navigate_to_pose action server not available")
            return

        pose = self.odom.pose.pose
        yaw = self.yaw_from_quaternion(pose.orientation)

        goal_x = pose.position.x + 4.0 * math.cos(yaw)
        goal_y = pose.position.y + 4.0 * math.sin(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "odom"
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0

        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = pose.orientation

        self.get_logger().info(f"Sending goal 4m ahead: x={goal_x:.2f}, y={goal_y:.2f}")

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} m"
        )

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Goal finished with status: {result.status}")
        rclpy.shutdown()

    def print_status(self):
        if self.odom is None:
            return

        q = self.odom.pose.pose.orientation
        yaw = self.yaw_from_quaternion(q)
        yaw_deg = math.degrees(yaw)

        if self.gps is not None:
            lat = self.gps.latitude
            lon = self.gps.longitude
            self.get_logger().info(
                f"GPS lat={lat:.8f}, lon={lon:.8f}, yaw={yaw_deg:.2f} deg"
            )
        else:
            self.get_logger().info(
                f"GPS not received yet, yaw={yaw_deg:.2f} deg"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Go4mPrintGps()
    rclpy.spin(node)


if __name__ == "__main__":
    main()