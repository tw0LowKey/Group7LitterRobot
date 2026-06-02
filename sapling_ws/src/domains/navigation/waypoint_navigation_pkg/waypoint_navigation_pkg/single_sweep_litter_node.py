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
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool, Int32
from action_msgs.msg import GoalStatus


class SingleSweepLitterNode(Node):
    """
    ONE NODE VERSION.

    Behaviour:
        - Uses odom frame only.
        - Generates straight-line waypoints internally.
        - Navigates through those waypoints using /navigate_to_pose.
        - Listens to /vision/detected_litter.
        - If litter is within radius of the Scout, it interrupts waypoint navigation.
        - Navigates to the litter approach pose.
        - Optionally rotates to face litter.
        - Publishes /call_bin.
        - Waits for /start_navigation=True as pickup complete signal.
        - Then resumes waypoint navigation.

    Required external inputs:
        /odom
        /vision/detected_litter

    Required external systems:
        Nav2 running with /navigate_to_pose action server.
        /vision/detected_litter publisher already running.

    Optional external:
        A pickup/bin coordination node that publishes /start_navigation=True
        after pickup is complete.
    """

    STATE_WAITING_ODOM = "WAITING_ODOM"
    STATE_WAYPOINT_NAV = "WAYPOINT_NAV"
    STATE_LITTER_NAV = "LITTER_NAV"
    STATE_ROTATE_TO_LITTER = "ROTATE_TO_LITTER"
    STATE_WAITING_PICKUP = "WAITING_PICKUP"
    STATE_DONE = "DONE"

    GOAL_NONE = "NONE"
    GOAL_WAYPOINT = "WAYPOINT"
    GOAL_LITTER_APPROACH = "LITTER_APPROACH"
    GOAL_LITTER_ROTATE = "LITTER_ROTATE"

    def __init__(self):
        super().__init__("single_sweep_litter_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("litter_topic", "/vision/detected_litter")
        self.declare_parameter("start_nav_topic", "/start_navigation")
        self.declare_parameter("call_bin_topic", "/call_bin")
        self.declare_parameter("queue_size_topic", "/queue_size")

        # Option A: everything in odom
        self.declare_parameter("goal_frame", "odom")

        # Straight-line waypoint generation
        self.declare_parameter("num_waypoints", 3)
        self.declare_parameter("spacing", 1.0)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        # Litter behaviour
        self.declare_parameter("litter_accept_radius", 2.0)
        self.declare_parameter("approach_offset", 0.5)
        self.declare_parameter("auto_complete_pickup", False)
        self.declare_parameter("pickup_wait_seconds", 3.0)

        self.nav_action = self.get_parameter("nav_action").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.litter_topic = self.get_parameter("litter_topic").value
        self.start_nav_topic = self.get_parameter("start_nav_topic").value
        self.call_bin_topic = self.get_parameter("call_bin_topic").value
        self.queue_size_topic = self.get_parameter("queue_size_topic").value

        self.goal_frame = self.get_parameter("goal_frame").value

        self.num_waypoints = int(self.get_parameter("num_waypoints").value)
        self.spacing = float(self.get_parameter("spacing").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)

        self.litter_accept_radius = float(
            self.get_parameter("litter_accept_radius").value
        )
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.auto_complete_pickup = bool(
            self.get_parameter("auto_complete_pickup").value
        )
        self.pickup_wait_seconds = float(
            self.get_parameter("pickup_wait_seconds").value
        )

        # ============================================================
        # Internal state
        # ============================================================

        self.state = self.STATE_WAITING_ODOM
        self.current_pose = None

        self.waypoints = self.generate_waypoints()
        self.current_waypoint_index = 0

        self.litter_queue = deque()
        self.current_litter_pose = None

        self.goal_handle = None
        self.active_goal_type = self.GOAL_NONE
        self.goal_active = False
        self.cancel_requested = False

        self.pickup_timer = None

        # ============================================================
        # Publishers
        # ============================================================

        self.call_bin_pub = self.create_publisher(
            Bool,
            self.call_bin_topic,
            10,
        )

        self.queue_size_pub = self.create_publisher(
            Int32,
            self.queue_size_topic,
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

        self.litter_sub = self.create_subscription(
            PoseStamped,
            self.litter_topic,
            self.litter_callback,
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
            self.nav_action,
            callback_group=self.cb_group,
        )

        # Main tick
        self.tick_timer = self.create_timer(
            0.5,
            self.tick,
            callback_group=self.cb_group,
        )

        self.print_startup_info()

    # ============================================================
    # Startup
    # ============================================================

    def print_startup_info(self):
        self.get_logger().info("=== Single Sweep + Litter Node Started ===")
        self.get_logger().info(f"Nav action:          {self.nav_action}")
        self.get_logger().info(f"Odom topic:          {self.odom_topic}")
        self.get_logger().info(f"Litter topic:        {self.litter_topic}")
        self.get_logger().info(f"Start nav topic:     {self.start_nav_topic}")
        self.get_logger().info(f"Call bin topic:      {self.call_bin_topic}")
        self.get_logger().info(f"Queue size topic:    {self.queue_size_topic}")
        self.get_logger().info(f"Goal frame:          {self.goal_frame}")
        self.get_logger().info(f"Num waypoints:       {self.num_waypoints}")
        self.get_logger().info(f"Spacing:             {self.spacing:.2f} m")
        self.get_logger().info(
            f"Start:               ({self.start_x:.2f}, {self.start_y:.2f})"
        )
        self.get_logger().info(
            f"Litter radius:       {self.litter_accept_radius:.2f} m"
        )
        self.get_logger().info(
            f"Approach offset:     {self.approach_offset:.2f} m"
        )
        self.get_logger().info(f"Auto pickup:         {self.auto_complete_pickup}")
        self.get_logger().info("Generated waypoints:")

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f"  WP{i + 1}: ({wp.pose.position.x:.2f}, "
                f"{wp.pose.position.y:.2f}) in {wp.header.frame_id}"
            )

        self.get_logger().info("Waiting for /odom...")

    # ============================================================
    # Waypoint generation
    # ============================================================

    def generate_waypoints(self):
        waypoints = []

        yaw = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        for i in range(self.num_waypoints):
            ps = PoseStamped()
            ps.header.frame_id = self.goal_frame
            ps.header.stamp.sec = 0
            ps.header.stamp.nanosec = 0

            ps.pose.position.x = self.start_x + (i + 1) * self.spacing
            ps.pose.position.y = self.start_y
            ps.pose.position.z = 0.0

            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            waypoints.append(ps)

        return waypoints

    # ============================================================
    # Callbacks
    # ============================================================

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def litter_callback(self, msg: PoseStamped):
        """
        Accept litter only if it is within litter_accept_radius.

        Option A assumption:
            /vision/detected_litter is already in odom frame.
        """

        if self.current_pose is None:
            self.get_logger().warn(
                "Ignoring litter because robot pose from /odom is unknown."
            )
            return

        if msg.header.frame_id and msg.header.frame_id != self.goal_frame:
            self.get_logger().warn(
                f"Received litter in frame '{msg.header.frame_id}', but this node "
                f"is using '{self.goal_frame}'. Option A requires odom everywhere."
            )

        dx = msg.pose.position.x - self.current_pose.position.x
        dy = msg.pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.litter_accept_radius:
            self.get_logger().info(
                f"Ignoring litter at ({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}); distance {dist:.2f} m is outside "
                f"radius {self.litter_accept_radius:.2f} m."
            )
            return

        # Force litter frame to odom for Option A
        msg.header.frame_id = self.goal_frame

        with self.lock:
            self.litter_queue.append(msg)
            queue_size = len(self.litter_queue)

        self.publish_queue_size()

        self.get_logger().warn(
            f"Accepted litter at ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}); distance {dist:.2f} m. "
            f"Queue size: {queue_size}"
        )

        # Interrupt waypoint navigation if currently driving a waypoint
        if self.state == self.STATE_WAYPOINT_NAV:
            self.get_logger().warn(
                "Litter has priority — cancelling current waypoint goal."
            )
            self.cancel_current_goal_for_litter()

    def start_nav_callback(self, msg: Bool):
        """
        This is used as pickup complete signal.

        After robot reaches litter and publishes /call_bin=True,
        some other node/person can publish:
            ros2 topic pub /start_navigation std_msgs/msg/Bool "{data: true}" --once

        Then this node resumes waypoint navigation.
        """

        if not msg.data:
            return

        if self.state != self.STATE_WAITING_PICKUP:
            return

        self.get_logger().info("Pickup complete signal received.")

        self.finish_current_litter_and_resume()

    # ============================================================
    # Main tick
    # ============================================================

    def tick(self):
        if self.current_pose is None:
            return

        if self.state == self.STATE_WAITING_ODOM:
            self.get_logger().info("Received /odom. Starting waypoint navigation.")
            self.state = self.STATE_WAYPOINT_NAV

        if self.goal_active:
            return

        if self.state == self.STATE_WAYPOINT_NAV:
            # Litter priority
            if len(self.litter_queue) > 0:
                self.state = self.STATE_LITTER_NAV
                self.send_next_litter_goal()
                return

            self.send_next_waypoint_goal()
            return

        if self.state == self.STATE_LITTER_NAV:
            self.send_next_litter_goal()
            return

        if self.state == self.STATE_ROTATE_TO_LITTER:
            self.send_rotate_to_litter_goal()
            return

        if self.state == self.STATE_WAITING_PICKUP:
            return

        if self.state == self.STATE_DONE:
            return

    # ============================================================
    # Navigation goal helpers
    # ============================================================

    def send_next_waypoint_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.state = self.STATE_DONE
            self.get_logger().info("All straight-line waypoints completed.")
            return

        wp = self.waypoints[self.current_waypoint_index]
        wp.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"Sending waypoint {self.current_waypoint_index + 1}/"
            f"{len(self.waypoints)}: "
            f"({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})"
        )

        self.send_nav_goal(wp, self.GOAL_WAYPOINT)

    def send_next_litter_goal(self):
        with self.lock:
            if self.current_litter_pose is None:
                if len(self.litter_queue) == 0:
                    self.state = self.STATE_WAYPOINT_NAV
                    return

                self.current_litter_pose = self.litter_queue.popleft()
                self.publish_queue_size()

        approach_pose = self.get_approach_pose(self.current_litter_pose)

        self.get_logger().warn(
            f"[Litter Phase 1] Navigating to litter approach pose: "
            f"({approach_pose.pose.position.x:.2f}, "
            f"{approach_pose.pose.position.y:.2f})"
        )

        self.send_nav_goal(approach_pose, self.GOAL_LITTER_APPROACH)

    def send_rotate_to_litter_goal(self):
        if self.current_litter_pose is None:
            self.state = self.STATE_WAYPOINT_NAV
            return

        face_pose = self.get_face_litter_pose(self.current_litter_pose)

        self.get_logger().info(
            f"[Litter Phase 2] Rotating to face litter at "
            f"({self.current_litter_pose.pose.position.x:.2f}, "
            f"{self.current_litter_pose.pose.position.y:.2f})"
        )

        self.send_nav_goal(face_pose, self.GOAL_LITTER_ROTATE)

    def send_nav_goal(self, pose_stamped: PoseStamped, goal_type: str):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                f"Nav2 action server {self.nav_action} not available."
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        with self.lock:
            self.goal_active = True
            self.active_goal_type = goal_type
            self.cancel_requested = False

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("No goal handle received from Nav2.")
            with self.lock:
                self.goal_active = False
                self.active_goal_type = self.GOAL_NONE
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            with self.lock:
                self.goal_active = False
                failed_goal_type = self.active_goal_type
                self.active_goal_type = self.GOAL_NONE

            self.handle_goal_finished(
                failed_goal_type,
                GoalStatus.STATUS_ABORTED,
            )
            return

        with self.lock:
            self.goal_handle = goal_handle

        self.get_logger().info("Nav2 goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            with self.lock:
                self.goal_active = False
                goal_type = self.active_goal_type
                self.active_goal_type = self.GOAL_NONE
                self.goal_handle = None
            self.handle_goal_finished(goal_type, GoalStatus.STATUS_ABORTED)
            return

        status = result.status

        with self.lock:
            goal_type = self.active_goal_type
            was_cancel_requested = self.cancel_requested

            self.goal_active = False
            self.active_goal_type = self.GOAL_NONE
            self.goal_handle = None
            self.cancel_requested = False

        if was_cancel_requested:
            self.get_logger().info("Goal cancellation completed.")

        self.handle_goal_finished(goal_type, status)

    def feedback_callback(self, feedback_msg):
        pass

    # ============================================================
    # Goal completion logic
    # ============================================================

    def handle_goal_finished(self, goal_type: str, status: int):
        if goal_type == self.GOAL_WAYPOINT:
            self.handle_waypoint_finished(status)

        elif goal_type == self.GOAL_LITTER_APPROACH:
            self.handle_litter_approach_finished(status)

        elif goal_type == self.GOAL_LITTER_ROTATE:
            self.handle_litter_rotate_finished(status)

    def handle_waypoint_finished(self, status: int):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} reached."
            )
            self.current_waypoint_index += 1

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Waypoint goal cancelled.")
            if len(self.litter_queue) > 0:
                self.state = self.STATE_LITTER_NAV
                return

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f"Waypoint {self.current_waypoint_index + 1} aborted. "
                f"Skipping to next waypoint."
            )
            self.current_waypoint_index += 1

        # Litter still has priority
        if len(self.litter_queue) > 0:
            self.state = self.STATE_LITTER_NAV
        else:
            self.state = self.STATE_WAYPOINT_NAV

    def handle_litter_approach_finished(self, status: int):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "Reached litter approach pose. Now rotating to face litter."
            )
            self.state = self.STATE_ROTATE_TO_LITTER

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                "Litter approach aborted. Trying rotate/call-bin phase anyway."
            )
            self.state = self.STATE_ROTATE_TO_LITTER

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Litter approach cancelled.")
            self.current_litter_pose = None
            self.state = self.STATE_WAYPOINT_NAV

    def handle_litter_rotate_finished(self, status: int):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Facing litter. Calling bin/pickup.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Rotate-to-litter aborted. Calling bin anyway.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Rotate-to-litter cancelled.")
            self.current_litter_pose = None
            self.state = self.STATE_WAYPOINT_NAV
            return

        self.publish_call_bin()
        self.state = self.STATE_WAITING_PICKUP

        if self.auto_complete_pickup:
            self.get_logger().info(
                f"Auto pickup enabled. Resuming in {self.pickup_wait_seconds:.1f}s."
            )
            self.start_auto_pickup_timer()
        else:
            self.get_logger().info(
                "Waiting for pickup complete signal on /start_navigation."
            )

    # ============================================================
    # Litter approach / rotation pose calculations
    # ============================================================

    def get_approach_pose(self, litter_pose: PoseStamped) -> PoseStamped:
        """
        Returns a pose approach_offset metres before the litter,
        facing the litter.
        """

        if self.current_pose is None:
            return litter_pose

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        litter_x = litter_pose.pose.position.x
        litter_y = litter_pose.pose.position.y

        dx = litter_x - robot_x
        dy = litter_y - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        angle_to_litter = math.atan2(dy, dx)

        approach_pose = PoseStamped()
        approach_pose.header.frame_id = self.goal_frame
        approach_pose.header.stamp = self.get_clock().now().to_msg()

        if distance <= self.approach_offset:
            approach_pose.pose.position.x = robot_x
            approach_pose.pose.position.y = robot_y
        else:
            approach_pose.pose.position.x = (
                litter_x - self.approach_offset * math.cos(angle_to_litter)
            )
            approach_pose.pose.position.y = (
                litter_y - self.approach_offset * math.sin(angle_to_litter)
            )

        approach_pose.pose.position.z = 0.0
        approach_pose.pose.orientation.z = math.sin(angle_to_litter / 2.0)
        approach_pose.pose.orientation.w = math.cos(angle_to_litter / 2.0)

        return approach_pose

    def get_face_litter_pose(self, litter_pose: PoseStamped) -> PoseStamped:
        """
        Same current robot position, but yaw faces the litter.
        """

        face_pose = PoseStamped()
        face_pose.header.frame_id = self.goal_frame
        face_pose.header.stamp = self.get_clock().now().to_msg()

        if self.current_pose is None:
            face_pose.pose = litter_pose.pose
            return face_pose

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        litter_x = litter_pose.pose.position.x
        litter_y = litter_pose.pose.position.y

        dx = litter_x - robot_x
        dy = litter_y - robot_y
        angle_to_litter = math.atan2(dy, dx)

        face_pose.pose.position.x = robot_x
        face_pose.pose.position.y = robot_y
        face_pose.pose.position.z = 0.0
        face_pose.pose.orientation.z = math.sin(angle_to_litter / 2.0)
        face_pose.pose.orientation.w = math.cos(angle_to_litter / 2.0)

        return face_pose

    # ============================================================
    # Cancel / resume
    # ============================================================

    def cancel_current_goal_for_litter(self):
        with self.lock:
            if self.goal_handle is None or not self.goal_active:
                self.state = self.STATE_LITTER_NAV
                return

            self.cancel_requested = True
            goal_handle = self.goal_handle

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Cancel request sent to Nav2.")
        except Exception as e:
            self.get_logger().warn(f"Cancel request failed: {e}")

        self.state = self.STATE_LITTER_NAV

    def finish_current_litter_and_resume(self):
        with self.lock:
            self.current_litter_pose = None

            if len(self.litter_queue) > 0:
                self.get_logger().info(
                    f"{len(self.litter_queue)} litter items still in queue."
                )
                self.state = self.STATE_LITTER_NAV
            else:
                self.get_logger().info("Litter queue empty. Resuming waypoints.")
                self.state = self.STATE_WAYPOINT_NAV

        self.publish_queue_size()

    # ============================================================
    # Pickup / queue publishers
    # ============================================================

    def publish_call_bin(self):
        msg = Bool()
        msg.data = True
        self.call_bin_pub.publish(msg)
        self.get_logger().warn("Published /call_bin=True.")

    def publish_queue_size(self):
        msg = Int32()
        with self.lock:
            msg.data = len(self.litter_queue)
            if self.current_litter_pose is not None:
                msg.data += 1

        self.queue_size_pub.publish(msg)

    def start_auto_pickup_timer(self):
        if self.pickup_timer is not None:
            self.pickup_timer.cancel()
            self.destroy_timer(self.pickup_timer)
            self.pickup_timer = None

        self.pickup_timer = self.create_timer(
            self.pickup_wait_seconds,
            self.auto_pickup_done_callback,
            callback_group=self.cb_group,
        )

    def auto_pickup_done_callback(self):
        if self.pickup_timer is not None:
            self.pickup_timer.cancel()
            self.destroy_timer(self.pickup_timer)
            self.pickup_timer = None

        if self.state == self.STATE_WAITING_PICKUP:
            self.get_logger().info("Auto pickup complete.")
            self.finish_current_litter_and_resume()


def main(args=None):
    rclpy.init(args=args)

    node = SingleSweepLitterNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down single sweep litter node.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()