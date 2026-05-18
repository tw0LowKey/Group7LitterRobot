#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus


class LeaderNavigationNode(Node):
    """
    Drives the leader robot through waypoints using FollowWaypoints batch action.

    Subscribes:
        /waypoints            — PoseArray from waypoint generator (TRANSIENT_LOCAL)
        /odom                 — leader position
        /pause_navigation     — pause (from litter handler)
        /resume_navigation    — resume (from litter handler)

    Publishes:
        /sweep_done           — all waypoints completed, follower should park

    Sends goals to:
        /follow_waypoints     — Nav2 FollowWaypoints action server

    States:
        WAITING_WAYPOINTS  — no waypoints received yet
        NAVIGATING         — driving through waypoint batch
        PAUSED             — stopped for litter handling
        IDLE               — all waypoints completed
    """

    STATE_WAITING_WAYPOINTS = "WAITING_WAYPOINTS"
    STATE_NAVIGATING = "NAVIGATING"
    STATE_PAUSED = "PAUSED"
    STATE_IDLE = "IDLE"

    def __init__(self):
        super().__init__("leader_navigation_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("leader_odom_topic", "/odom")
        self.declare_parameter("waypoint_topic", "/waypoints")
        self.declare_parameter("pause_nav_topic", "/pause_navigation")
        self.declare_parameter("resume_nav_topic", "/resume_navigation")
        self.declare_parameter("sweep_done_topic", "/sweep_done")
        self.declare_parameter("goal_frame", "map")

        self.follow_waypoints_action = self.get_parameter("follow_waypoints_action").value
        self.leader_odom_topic = self.get_parameter("leader_odom_topic").value
        self.waypoint_topic = self.get_parameter("waypoint_topic").value
        self.pause_nav_topic = self.get_parameter("pause_nav_topic").value
        self.resume_nav_topic = self.get_parameter("resume_nav_topic").value
        self.sweep_done_topic = self.get_parameter("sweep_done_topic").value
        self.goal_frame = self.get_parameter("goal_frame").value

        # ============================================================
        # Internal state
        # ============================================================

        self.all_waypoints = []
        self.state = self.STATE_WAITING_WAYPOINTS
        self.current_waypoint_index = 0
        self.active_batch_start_index = 0
        self.leader_position = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_in_progress = False
        self.sweep_done_sent = False

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

        self.sweep_done_pub = self.create_publisher(
            Bool,
            self.sweep_done_topic,
            10,
        )

        # ============================================================
        # Subscribers
        # ============================================================

        self.waypoint_sub = self.create_subscription(
            PoseArray,
            self.waypoint_topic,
            self.waypoint_callback,
            latching_qos,
            callback_group=self.cb_group,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.pause_nav_sub = self.create_subscription(
            Bool,
            self.pause_nav_topic,
            self.pause_nav_callback,
            10,
            callback_group=self.cb_group,
        )

        self.resume_nav_sub = self.create_subscription(
            Bool,
            self.resume_nav_topic,
            self.resume_nav_callback,
            10,
            callback_group=self.cb_group,
        )

        # ============================================================
        # Nav2 action client
        # ============================================================

        self.follow_client = ActionClient(
            self,
            FollowWaypoints,
            self.follow_waypoints_action,
            callback_group=self.cb_group,
        )

        self.print_startup_info()

    def print_startup_info(self):
        self.get_logger().info("=== Leader Navigation Node Started ===")
        self.get_logger().info(f"FollowWaypoints action: {self.follow_waypoints_action}")
        self.get_logger().info(f"Odom topic:             {self.leader_odom_topic}")
        self.get_logger().info(f"Waypoint topic:         {self.waypoint_topic}")
        self.get_logger().info(f"Pause nav topic:        {self.pause_nav_topic}")
        self.get_logger().info(f"Resume nav topic:       {self.resume_nav_topic}")
        self.get_logger().info(f"Sweep done topic:       {self.sweep_done_topic}")
        self.get_logger().info(f"Goal frame:             {self.goal_frame}")
        self.get_logger().info(f"State:                  {self.state}")
        self.get_logger().info("Waiting for waypoints...")

    # ============================================================
    # Odom callback
    # ============================================================

    def odom_callback(self, msg: Odometry):
        self.leader_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    # ============================================================
    # Waypoint callback — receive PoseArray, start batch
    # ============================================================

    def waypoint_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty PoseArray — ignoring.")
            return

        pose_stamped_list = []
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header.frame_id = self.goal_frame
            ps.header.stamp.sec = 0
            ps.header.stamp.nanosec = 0
            ps.pose = pose
            pose_stamped_list.append(ps)

        with self.lock:
            self.all_waypoints = pose_stamped_list
            self.current_waypoint_index = 0
            self.active_batch_start_index = 0
            self.sweep_done_sent = False
            self.state = self.STATE_NAVIGATING

        self.get_logger().info(
            f"Received {len(self.all_waypoints)} waypoints — starting batch navigation."
        )

        self.start_batch_from_index(0)

    # ============================================================
    # Batch navigation
    # ============================================================

    def start_batch_from_index(self, start_index):
        if start_index >= len(self.all_waypoints):
            self.publish_sweep_done()
            return

        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("FollowWaypoints action server not available.")
            return

        remaining = self.all_waypoints[start_index:]

 
        now = self.get_clock().now().to_msg()
        for ps in remaining:
             ps.header.stamp.sec = 0
             ps.header.stamp.nanosec = 0

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = remaining

        with self.lock:
            self.active_batch_start_index = start_index
            self.cancel_in_progress = False

        self.get_logger().info(
            f"Sending FollowWaypoints batch: WP {start_index + 1} to "
            f"{len(self.all_waypoints)} ({len(remaining)} waypoints)."
        )

        future = self.follow_client.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_feedback_callback,
        )
        future.add_done_callback(self.follow_goal_response_callback)

    def follow_goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("FollowWaypoints returned no goal handle.")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("FollowWaypoints goal rejected.")
            with self.lock:
                self.goal_handle = None
                self.state = self.STATE_IDLE
            return

        self.get_logger().info("FollowWaypoints goal accepted.")

        with self.lock:
            self.goal_handle = goal_handle

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.follow_result_callback)

    def follow_feedback_callback(self, feedback_msg):
   
        relative_index = feedback_msg.feedback.current_waypoint
        absolute_index = self.active_batch_start_index + relative_index

        with self.lock:
            self.current_waypoint_index = absolute_index

    def follow_result_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"FollowWaypoints result error: {e}")
            with self.lock:
                self.goal_handle = None
            return

        status = result.status

        with self.lock:
            self.goal_handle = None
            self.result_future = None

        if self.cancel_in_progress:
            self.get_logger().info("FollowWaypoints cancelled for pause.")
            with self.lock:
                self.cancel_in_progress = False
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("FollowWaypoints batch complete — all waypoints reached.")
            self.publish_sweep_done()

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("FollowWaypoints cancelled.")

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f"FollowWaypoints aborted at WP {self.current_waypoint_index + 1}. "
                f"Skipping to next."
            )

            with self.lock:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.all_waypoints):
                    self.publish_sweep_done()
                elif self.state == self.STATE_NAVIGATING:
                    self.start_batch_from_index(self.current_waypoint_index)

        else:
            self.get_logger().warn(f"FollowWaypoints ended with status: {status}")
            with self.lock:
                self.state = self.STATE_IDLE



    def pause_nav_callback(self, msg: Bool):
        if not msg.data:
            return
        if self.state != self.STATE_NAVIGATING:
            return

        self.get_logger().info(
            f"Pause received at WP {self.current_waypoint_index + 1} "
            f"— cancelling batch."
        )

        with self.lock:
            self.cancel_in_progress = True
            self.state = self.STATE_PAUSED

            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()

    def resume_nav_callback(self, msg: Bool):
        if not msg.data:
            return
        if self.state != self.STATE_PAUSED:
            return

        with self.lock:
            if self.current_waypoint_index >= len(self.all_waypoints):
                self.publish_sweep_done()
                return

            self.state = self.STATE_NAVIGATING

        self.get_logger().info(
            f"Resuming from WP {self.current_waypoint_index + 1}/"
            f"{len(self.all_waypoints)}."
        )

        self.start_batch_from_index(self.current_waypoint_index)

    def publish_sweep_done(self):
        with self.lock:
            if self.sweep_done_sent:
                return
            self.sweep_done_sent = True
            self.state = self.STATE_IDLE

        msg = Bool()
        msg.data = True
        self.sweep_done_pub.publish(msg)
        self.get_logger().info(
            "All waypoints completed — published /sweep_done."
        )


def main(args=None):
    rclpy.init(args=args)

    node = LeaderNavigationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down leader navigation node.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
