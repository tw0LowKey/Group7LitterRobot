#!/usr/bin/env python3

import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus


class LitterHandlerNode(Node):
    """
    Handles litter detection, navigation to litter, and bin coordination.

    Subscribes:
        /vision/detected_litter  — PoseStamped of detected litter
        /odom                    — leader position for approach calculation
        /start_navigation        — signal from coordination node (pickup done)

    Publishes:
        /pause_navigation        — pause leader waypoint navigation
        /resume_navigation       — resume leader waypoint navigation
        /call_bin                — signal coordination node to call bin

    Sends goals to:
        /navigate_to_pose        — Nav2 action server (drives to litter)

    States:
        IDLE              — waiting for litter detection
        PAUSING           — waiting for leader to fully stop before taking Nav2
        NAVIGATING        — driving to litter pose
        WAITING_PICKUP    — at litter, waiting for bin + pickup
    """

    STATE_IDLE = "IDLE"
    STATE_PAUSING = "PAUSING"
    STATE_NAVIGATING = "NAVIGATING"
    STATE_ROTATING = "ROTATING"
    STATE_WAITING_PICKUP = "WAITING_PICKUP"

    def __init__(self):
        super().__init__("litter_handler_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("leader_action", "/navigate_to_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("litter_topic", "/vision/detected_litter")
        self.declare_parameter("pause_nav_topic", "/pause_navigation")
        self.declare_parameter("resume_nav_topic", "/resume_navigation")
        self.declare_parameter("call_bin_topic", "/call_bin")
        self.declare_parameter("start_nav_topic", "/start_navigation")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("approach_offset", 0.3)
        self.declare_parameter("pause_settle_time", 1.0)

        self.leader_action = self.get_parameter("leader_action").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.litter_topic = self.get_parameter("litter_topic").value
        self.pause_nav_topic = self.get_parameter("pause_nav_topic").value
        self.resume_nav_topic = self.get_parameter("resume_nav_topic").value
        self.call_bin_topic = self.get_parameter("call_bin_topic").value
        self.start_nav_topic = self.get_parameter("start_nav_topic").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.approach_offset = self.get_parameter("approach_offset").value
        self.pause_settle_time = self.get_parameter("pause_settle_time").value

        # ============================================================
        # Internal state
        # ============================================================

        self.state = self.STATE_IDLE
        self.current_pose = None
        self.litter_queue = deque()
        self.current_litter_pose = None
        self.is_navigating = False
        self.goal_handle = None
        self.pause_timer = None

        # ============================================================
        # Publishers (all regular QoS — no latching)
        # ============================================================

        self.pause_nav_pub = self.create_publisher(
            Bool,
            self.pause_nav_topic,
            10,
        )

        self.resume_nav_pub = self.create_publisher(
            Bool,
            self.resume_nav_topic,
            10,
        )

        self.call_bin_pub = self.create_publisher(
            Bool,
            self.call_bin_topic,
            10,
        )

        # ============================================================
        # Subscribers (all regular QoS — no latching)
        # ============================================================

        self.litter_sub = self.create_subscription(
            PoseStamped,
            self.litter_topic,
            self.litter_callback,
            10,
            callback_group=self.cb_group,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.start_nav_sub = self.create_subscription(
            Bool,
            self.start_nav_topic,
            self.start_nav_callback,
            10,
            callback_group=self.cb_group,
        )

        # ============================================================
        # Nav2 action client
        # ============================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.leader_action,
            callback_group=self.cb_group,
        )

        # ============================================================
        # Timers
        # ============================================================

        self.tick_timer = self.create_timer(
            0.5,
            self.state_machine_tick,
            callback_group=self.cb_group,
        )

        self.print_startup_info()

    # ============================================================
    # Startup logging
    # ============================================================

    def print_startup_info(self):
        self.get_logger().info("=== Litter Handler Node Started ===")
        self.get_logger().info(f"Nav2 action:       {self.leader_action}")
        self.get_logger().info(f"Odom topic:        {self.odom_topic}")
        self.get_logger().info(f"Litter topic:      {self.litter_topic}")
        self.get_logger().info(f"Pause nav topic:   {self.pause_nav_topic}")
        self.get_logger().info(f"Resume nav topic:  {self.resume_nav_topic}")
        self.get_logger().info(f"Call bin topic:     {self.call_bin_topic}")
        self.get_logger().info(f"Start nav topic:   {self.start_nav_topic}")
        self.get_logger().info(f"Approach offset:   {self.approach_offset}m")
        self.get_logger().info(f"Pause settle time: {self.pause_settle_time}s")
        self.get_logger().info(f"State:             {self.state}")
        self.get_logger().info("Waiting for litter detections...")

    # ============================================================
    # Odom callback
    # ============================================================

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    # ============================================================
    # Litter detection callback
    # ============================================================

    def litter_callback(self, msg: PoseStamped):
        with self.lock:
            self.litter_queue.append(msg)
            queue_size = len(self.litter_queue)

        self.get_logger().info(
            f"Litter detected at ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}). Queue size: {queue_size}"
        )

        # If idle, pause leader and wait for it to fully stop
        if self.state == self.STATE_IDLE:
            self.pause_leader_navigation()
            with self.lock:
                self.state = self.STATE_PAUSING

            # Wait for leader to cancel its Nav2 goal before we send ours
            self.pause_timer = self.create_timer(
                self.pause_settle_time,
                self.pause_settle_callback,
                callback_group=self.cb_group,
            )

    def pause_settle_callback(self):
        """Called after pause_settle_time — leader should have fully stopped."""
        if self.pause_timer is not None:
            self.pause_timer.cancel()
            self.destroy_timer(self.pause_timer)
            self.pause_timer = None

        self.get_logger().info("Leader settled — starting litter navigation.")
        with self.lock:
            self.state = self.STATE_NAVIGATING

    # ============================================================
    # Start navigation callback — pickup complete from coordination node
    # ============================================================

    def start_nav_callback(self, msg: Bool):
        if msg.data and self.state == self.STATE_WAITING_PICKUP:
            self.get_logger().info("Pickup complete signal received.")

            with self.lock:
                if len(self.litter_queue) > 0:
                    self.get_logger().info(
                        f"{len(self.litter_queue)} more litter in queue — "
                        f"continuing collection."
                    )
                    self.state = self.STATE_NAVIGATING
                else:
                    self.state = self.STATE_IDLE
                    self.get_logger().info(
                        "Litter queue empty — resuming waypoint navigation."
                    )
                    self.resume_leader_navigation()

    # ============================================================
    # Pause / resume leader navigation
    # ============================================================

    def pause_leader_navigation(self):
        msg = Bool()
        msg.data = True
        self.pause_nav_pub.publish(msg)
        self.get_logger().info("Published /pause_navigation.")

    def resume_leader_navigation(self):
        msg = Bool()
        msg.data = True
        self.resume_nav_pub.publish(msg)
        self.get_logger().info("Published /resume_navigation.")

    # ============================================================
    # Call bin
    # ============================================================

    def publish_call_bin(self):
        msg = Bool()
        msg.data = True
        self.call_bin_pub.publish(msg)
        self.get_logger().info("Published /call_bin — waiting for pickup.")

    # ============================================================
    # Approach pose calculation
    # ============================================================

    def get_approach_pose(self, litter_pose: PoseStamped):
        """Calculate a pose approach_offset metres away from litter, facing it."""
        if self.current_pose is None:
            self.get_logger().warn(
                "Current pose unknown — sending raw litter coordinates."
            )
            return litter_pose

        dx = litter_pose.pose.position.x - self.current_pose.position.x
        dy = litter_pose.pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        angle_to_litter = math.atan2(dy, dx)

        approach_pose = PoseStamped()
        approach_pose.header.frame_id = self.goal_frame
        approach_pose.header.stamp = self.get_clock().now().to_msg()

        if distance <= self.approach_offset:
            self.get_logger().info(
                f"Already within {self.approach_offset}m of litter."
            )
            approach_pose.pose.position.x = self.current_pose.position.x
            approach_pose.pose.position.y = self.current_pose.position.y
        else:
            approach_pose.pose.position.x = (
                litter_pose.pose.position.x
                - self.approach_offset * math.cos(angle_to_litter)
            )
            approach_pose.pose.position.y = (
                litter_pose.pose.position.y
                - self.approach_offset * math.sin(angle_to_litter)
            )

        approach_pose.pose.position.z = 0.0

        # Orientation: face the litter
        approach_pose.pose.orientation.z = math.sin(angle_to_litter / 2.0)
        approach_pose.pose.orientation.w = math.cos(angle_to_litter / 2.0)

        return approach_pose

    def get_face_litter_pose(self, litter_pose: PoseStamped):
        """Goal at current position but rotated to face the litter dead-on."""
        if self.current_pose is None:
            return litter_pose

        dx = litter_pose.pose.position.x - self.current_pose.position.x
        dy = litter_pose.pose.position.y - self.current_pose.position.y
        angle_to_litter = math.atan2(dy, dx)

        face_pose = PoseStamped()
        face_pose.header.frame_id = self.goal_frame
        face_pose.header.stamp = self.get_clock().now().to_msg()
        face_pose.pose.position.x = self.current_pose.position.x
        face_pose.pose.position.y = self.current_pose.position.y
        face_pose.pose.position.z = 0.0
        face_pose.pose.orientation.z = math.sin(angle_to_litter / 2.0)
        face_pose.pose.orientation.w = math.cos(angle_to_litter / 2.0)

        return face_pose

    # ============================================================
    # State machine
    # ============================================================

    def state_machine_tick(self):
        if self.state == self.STATE_NAVIGATING:
            self.tick_navigating()
        elif self.state == self.STATE_ROTATING:
            self.tick_rotating()
        elif self.state == self.STATE_PAUSING:
            pass  # waiting for settle timer
        elif self.state == self.STATE_WAITING_PICKUP:
            pass
        elif self.state == self.STATE_IDLE:
            pass

    def tick_navigating(self):
        with self.lock:
            if self.is_navigating:
                return
            if len(self.litter_queue) == 0 and self.current_litter_pose is None:
                self.state = self.STATE_IDLE
                self.resume_leader_navigation()
                return

        self.send_litter_goal()

    def tick_rotating(self):
        with self.lock:
            if self.is_navigating:
                return
            if self.current_litter_pose is None:
                self.state = self.STATE_IDLE
                self.resume_leader_navigation()
                return

        self.send_rotate_goal()

    # ============================================================
    # Goal sending — Phase 1: navigate to approach position
    # ============================================================

    def send_litter_goal(self):
        with self.lock:
            if len(self.litter_queue) == 0:
                return
            self.current_litter_pose = self.litter_queue.popleft()
            self.is_navigating = True

        approach_pose = self.get_approach_pose(self.current_litter_pose)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
                self.litter_queue.appendleft(self.current_litter_pose)
                self.current_litter_pose = None
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = approach_pose

        self.get_logger().info(
            f"[Litter Phase 1] Navigating to approach pose at "
            f"({approach_pose.pose.position.x:.2f}, "
            f"{approach_pose.pose.position.y:.2f})"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.approach_response_callback)

    # ============================================================
    # Goal sending — Phase 2: rotate in place to face litter
    # ============================================================

    def send_rotate_goal(self):
        with self.lock:
            if self.current_litter_pose is None:
                return
            self.is_navigating = True

        face_pose = self.get_face_litter_pose(self.current_litter_pose)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = face_pose

        self.get_logger().info(
            f"[Litter Phase 2] Rotating to face litter at "
            f"({self.current_litter_pose.pose.position.x:.2f}, "
            f"{self.current_litter_pose.pose.position.y:.2f})"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.rotate_response_callback)

    # ============================================================
    # Phase 1 callbacks (approach)
    # ============================================================

    def approach_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("Failed to get approach goal handle.")
            with self.lock:
                self.is_navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Approach goal rejected.")
            with self.lock:
                self.is_navigating = False
            return

        self.get_logger().info("Approach goal accepted.")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.approach_result_callback)

    def approach_result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().warn("No approach result received.")
            with self.lock:
                self.is_navigating = False
            return

        status = result.status

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    "Phase 1 complete — rotating to face litter."
                )
                self.state = self.STATE_ROTATING

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    "Approach aborted — rotating to face litter anyway."
                )
                self.state = self.STATE_ROTATING

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Approach goal cancelled.")
                self.current_litter_pose = None

    # ============================================================
    # Phase 2 callbacks (rotate to face)
    # ============================================================

    def rotate_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("Failed to get rotate goal handle.")
            with self.lock:
                self.is_navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Rotate goal rejected — calling bin anyway.")
            with self.lock:
                self.is_navigating = False
                self.state = self.STATE_WAITING_PICKUP
                self.current_litter_pose = None
            self.publish_call_bin()
            return

        self.get_logger().info("Rotate goal accepted.")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rotate_result_callback)

    def rotate_result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().warn("No rotate result received.")
            with self.lock:
                self.is_navigating = False
            return

        status = result.status

        with self.lock:
            self.is_navigating = False
            self.current_litter_pose = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    "Phase 2 complete — facing litter. Calling bin."
                )
                self.state = self.STATE_WAITING_PICKUP
                self.publish_call_bin()

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    "Rotate aborted — calling bin at current position."
                )
                self.state = self.STATE_WAITING_PICKUP
                self.publish_call_bin()

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Rotate goal cancelled.")

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = LitterHandlerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down litter handler node.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()