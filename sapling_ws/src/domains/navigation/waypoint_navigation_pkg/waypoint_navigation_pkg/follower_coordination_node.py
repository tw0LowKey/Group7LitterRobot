#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

from sapling_interfaces.msg import LeaderPose, FollowerStatus


class FollowerCoordinationNode(Node):


    def __init__(self):
        super().__init__("follower_coordination_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("call_bin_topic", "/call_bin")
        self.declare_parameter("sweep_done_topic", "/sweep_done")
        self.declare_parameter("leader_pose_tx_topic", "/comms/leader_to_follower_tx")
        self.declare_parameter("follower_status_rx_topic", "/comms/follower_to_leader_tx")
        self.declare_parameter("start_nav_topic", "/start_navigation")
        self.declare_parameter("follower_status_echo_topic", "/follower_status")
        self.declare_parameter("pickup_wait_time", 2.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.call_bin_topic = self.get_parameter("call_bin_topic").value
        self.sweep_done_topic = self.get_parameter("sweep_done_topic").value
        self.leader_pose_tx_topic = self.get_parameter("leader_pose_tx_topic").value
        self.follower_status_rx_topic = self.get_parameter("follower_status_rx_topic").value
        self.start_nav_topic = self.get_parameter("start_nav_topic").value
        self.follower_status_echo_topic = self.get_parameter("follower_status_echo_topic").value
        self.pickup_wait_time = self.get_parameter("pickup_wait_time").value

        # ============================================================
        # Internal state
        # ============================================================

        self.leader_lat = None
        self.leader_lon = None
        self.leader_qz = 0.0
        self.leader_qw = 1.0

        self.call_bin_flag = False
        self.leader_seq = 0
        self.pickup_timer = None

        self.call_bin_seq = 0
        self.last_follower_parked_false_seen = True

        # ============================================================
        # QoS
        # ============================================================

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.leader_pose_pub = self.create_publisher(
            LeaderPose,
            self.leader_pose_tx_topic,
            10,
        )

        self.start_nav_pub = self.create_publisher(
            Bool,
            self.start_nav_topic,
            10,
        )

        self.follower_status_echo_pub = self.create_publisher(
            FollowerStatus,
            self.follower_status_echo_topic,
            10,
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10,
            callback_group=self.cb_group,
        )

        self.call_bin_sub = self.create_subscription(
            Bool,
            self.call_bin_topic,
            self.call_bin_callback,
            10,
            callback_group=self.cb_group,
        )

        self.follower_status_sub = self.create_subscription(
            FollowerStatus,
            self.follower_status_rx_topic,
            self.follower_status_callback,
            10,
            callback_group=self.cb_group,
        )

        self.sweep_done_sub = self.create_subscription(
            Bool,
            self.sweep_done_topic,
            self.sweep_done_callback,
            10,
            callback_group=self.cb_group,
        )

        # ============================================================
        # Timers
        # ============================================================

        self.broadcast_timer = self.create_timer(
            0.5,
            self.broadcast_leader_pose,
            callback_group=self.cb_group,
        )

        self.print_startup_info()

    # ============================================================
    # Startup logging
    # ============================================================

    def print_startup_info(self):
        self.get_logger().info("=== Follower Coordination Node Started ===")
        self.get_logger().info(f"Odom topic:             {self.odom_topic}")
        self.get_logger().info(f"GPS topic:              {self.gps_topic}")
        self.get_logger().info(f"Call bin topic:          {self.call_bin_topic}")
        self.get_logger().info(f"Sweep done topic:       {self.sweep_done_topic}")
        self.get_logger().info(f"Leader pose TX topic:   {self.leader_pose_tx_topic}")
        self.get_logger().info(f"Follower status RX:     {self.follower_status_rx_topic}")
        self.get_logger().info(f"Start nav topic:        {self.start_nav_topic}")
        self.get_logger().info(f"Pickup wait time:       {self.pickup_wait_time}s")
        self.get_logger().info(f"Broadcast rate:         every 0.5s")


    def broadcast_leader_pose(self):
        if self.leader_lat is None or self.leader_lon is None:
            return

        self.leader_seq += 1

        msg = LeaderPose()
        msg.seq = self.leader_seq
        msg.latitude = self.leader_lat
        msg.longitude = self.leader_lon
        msg.orientation_z = self.leader_qz
        msg.orientation_w = self.leader_qw
        msg.call_bin = self.call_bin_flag

        self.leader_pose_pub.publish(msg)

    # ============================================================
    # Odom callback — orientation
    # ============================================================

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.leader_qz = q.z
        self.leader_qw = q.w

    # ============================================================
    # GPS callback — lat/lon for broadcast
    # ============================================================

    def gps_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        self.leader_lat = msg.latitude
        self.leader_lon = msg.longitude

    # ============================================================
    # Call bin callback — from litter handler
    # ============================================================

    def call_bin_callback(self, msg: Bool):
        if msg.data:
            with self.lock:
                self.call_bin_flag = True
                self.call_bin_seq += 1

                self.last_follower_parked_false_seen = False

                if self.pickup_timer is not None:
                    self.pickup_timer.cancel()
                    self.destroy_timer(self.pickup_timer)
                    self.pickup_timer = None

            self.get_logger().info(
                f"Call bin received (seq={self.call_bin_seq}) "
                f"— flag set, waiting for fresh park."
            )

    # ============================================================
    # Sweep done callback — from leader nav (all waypoints done)
    # ============================================================

    def sweep_done_callback(self, msg: Bool):
        if msg.data:
            with self.lock:
                self.call_bin_flag = True
                self.call_bin_seq += 1
                self.last_follower_parked_false_seen = False

                if self.pickup_timer is not None:
                    self.pickup_timer.cancel()
                    self.destroy_timer(self.pickup_timer)
                    self.pickup_timer = None

            self.get_logger().info(
                "Sweep done received — flag set for follower to park (final collection)."
            )

    # ============================================================
    # Follower status callback
    # ============================================================

    def follower_status_callback(self, msg: FollowerStatus):

        self.follower_status_echo_pub.publish(msg)

        with self.lock:
         
            if not msg.parked:
                self.last_follower_parked_false_seen = True

            if (msg.parked
                    and self.call_bin_flag
                    and self.last_follower_parked_false_seen
                    and self.pickup_timer is None):

                self.get_logger().info(
                    f"Follower PARKED at ({msg.park_x:.2f}, {msg.park_y:.2f}) "
                    f"— starting pickup timer ({self.pickup_wait_time}s)."
                )
                self.pickup_timer = self.create_timer(
                    self.pickup_wait_time,
                    self.pickup_complete_callback,
                    callback_group=self.cb_group,
                )

    # ============================================================
    # Pickup complete
    # ============================================================

    def pickup_complete_callback(self):
        if self.pickup_timer is not None:
            self.pickup_timer.cancel()
            self.destroy_timer(self.pickup_timer)
            self.pickup_timer = None

        self.get_logger().info("Pickup complete.")

        with self.lock:
            self.call_bin_flag = False

        self.get_logger().info("Call bin cleared — follower will resume following.")

        resume_msg = Bool()
        resume_msg.data = True
        self.start_nav_pub.publish(resume_msg)
        self.get_logger().info("Published /start_navigation — litter handler notified.")


def main(args=None):
    rclpy.init(args=args)

    node = FollowerCoordinationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down follower coordination node.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
