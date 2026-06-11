import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

from sapling_interfaces.msg import LeaderPose, FollowerStatus


class GPSFollowerBehaviorNode(Node):
    """
    GPS follower behaviour node with litter pickup mode.

    Uses two position sources:
        - GPS: for inter-robot distance calculations (parking gap, trail
          distance, resume). Both robots go through gps_to_xy with the
          same reference point — same coordinate frame.
        - Odom: for Nav2 goals. Nav2 operates in the odom/map frame.

    When computing parking targets, the node:
        1. Calculates the target in GPS-XY space (relative to leader GPS)
        2. Converts to odom frame using the follower's GPS-odom offset
        3. Sends the odom-frame target to Nav2

    Subscribes:
        /follower/odom                  — follower odom position + orientation
        /follower/navsat (or /gps/fix)  — follower GPS for distance calculations
        /comms/leader_to_follower_tx    — leader pose (GPS + orientation + call_bin)

    Publishes:
        /comms/follower_to_leader_tx    — follower status (every 0.5s)
        /bin_ready                      — Bool, True when bin is in position
    """

    STATE_FOLLOWING = "FOLLOWING"
    STATE_PARKING_APPROACH = "PARKING_APPROACH"
    STATE_PARKING_FINAL = "PARKING_FINAL"
    STATE_WAITING = "WAITING"

    EARTH_RADIUS = 6371000.0

    def __init__(self):
        super().__init__("gps_follower_behavior_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ================================================================
        # Parameters
        # ================================================================

        self.declare_parameter("breadcrumb_distance", 1.0)
        self.declare_parameter("min_follow_distance", 2.0)
        self.declare_parameter("approach_distance", 0.3)
        self.declare_parameter("park_distance", 0.1)
        self.declare_parameter("resume_distance", 2.5)
        self.declare_parameter("skip_park_distance", 0.9)
        self.declare_parameter("leader_half_length", 0.325)
        self.declare_parameter("follower_half_length", 0.325)
        self.declare_parameter("follower_odom_topic", "/follower/odom")
        self.declare_parameter("follower_gps_topic", "/follower/navsat")
        self.declare_parameter("follower_action", "/follower/navigate_to_pose")
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("leader_pose_rx_topic", "/comms/leader_to_follower_tx")
        self.declare_parameter("follower_status_tx_topic", "/comms/follower_to_leader_tx")
        self.declare_parameter("ref_lat", 0.0)
        self.declare_parameter("ref_lon", 0.0)
        self.declare_parameter("auto_ref", True)

        # Read parameters
        self.breadcrumb_distance = self.get_parameter("breadcrumb_distance").value
        self.min_follow_distance = self.get_parameter("min_follow_distance").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.park_distance = self.get_parameter("park_distance").value
        self.resume_distance = self.get_parameter("resume_distance").value
        self.skip_park_distance = self.get_parameter("skip_park_distance").value
        self.leader_half_length = self.get_parameter("leader_half_length").value
        self.follower_half_length = self.get_parameter("follower_half_length").value
        self.follower_odom_topic = self.get_parameter("follower_odom_topic").value
        self.follower_gps_topic = self.get_parameter("follower_gps_topic").value
        self.follower_action = self.get_parameter("follower_action").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.leader_pose_rx_topic = self.get_parameter("leader_pose_rx_topic").value
        self.follower_status_tx_topic = self.get_parameter("follower_status_tx_topic").value
        self.ref_lat = self.get_parameter("ref_lat").value
        self.ref_lon = self.get_parameter("ref_lon").value
        self.auto_ref = self.get_parameter("auto_ref").value

        # ================================================================
        # Internal state
        # ================================================================

        self.state = self.STATE_FOLLOWING
        self.breadcrumbs = []
        self.current_goal_index = 0

        # Leader position in GPS-XY frame
        self.leader_position = None
        self.leader_yaw = None

        # Follower GPS position (GPS-XY frame) — for distance calculations
        self.follower_gps_position = None

        # Follower odom position (odom/map frame) — for Nav2 goals
        self.follower_odom_position = None
        self.follower_yaw = None

        self.last_breadcrumb_position = None
        self.is_navigating = False
        self.goal_handle = None
        self.ref_set = False
        self.follower_seq = 0
        self.last_leader_seq = 0
        self.bin_is_ready = False
        self.parked_for_current_call = False

        # ================================================================
        # Publishers
        # ================================================================

        self.follower_status_pub = self.create_publisher(
            FollowerStatus,
            self.follower_status_tx_topic,
            10
        )

        self.bin_ready_pub = self.create_publisher(
            Bool,
            "/bin_ready",
            10
        )

        # ================================================================
        # Subscribers
        # ================================================================

        self.leader_pose_sub = self.create_subscription(
            LeaderPose,
            self.leader_pose_rx_topic,
            self.leader_pose_callback,
            10,
            callback_group=self.cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.follower_odom_topic,
            self.odom_callback,
            10,
            callback_group=self.cb_group
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.follower_gps_topic,
            self.gps_callback,
            10,
            callback_group=self.cb_group
        )

        # ================================================================
        # Nav2 action client
        # ================================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.follower_action,
            callback_group=self.cb_group
        )

        # ================================================================
        # Timers
        # ================================================================

        self.tick_timer = self.create_timer(
            0.5,
            self.state_machine_tick,
            callback_group=self.cb_group
        )

        self.status_timer = self.create_timer(
            0.5,
            self.publish_follower_status,
            callback_group=self.cb_group
        )

        self.bin_ready_timer = self.create_timer(
            0.5,
            self.publish_bin_ready,
            callback_group=self.cb_group
        )

        self.print_startup_info()

    # ================================================================
    # Startup logging
    # ================================================================

    def print_startup_info(self):
        self.get_logger().info("=== GPS Follower Behaviour Node Started ===")
        self.get_logger().info(f"Follower action:       {self.follower_action}")
        self.get_logger().info(f"Follower odom topic:   {self.follower_odom_topic}")
        self.get_logger().info(f"Follower GPS topic:    {self.follower_gps_topic}")
        self.get_logger().info(f"Leader pose RX topic:  {self.leader_pose_rx_topic}")
        self.get_logger().info(f"Follower status TX:    {self.follower_status_tx_topic}")
        self.get_logger().info(f"Goal frame:            {self.goal_frame}")
        self.get_logger().info(f"Breadcrumb dist:       {self.breadcrumb_distance}m")
        self.get_logger().info(f"Min follow dist:       {self.min_follow_distance}m")
        self.get_logger().info(f"Approach distance:     {self.approach_distance}m")
        self.get_logger().info(f"Park distance:         {self.park_distance}m")
        self.get_logger().info(f"Skip park distance:    {self.skip_park_distance}m")
        self.get_logger().info(f"Resume distance:       {self.resume_distance}m")
        self.get_logger().info(f"State:                 {self.state}")

    # ================================================================
    # Maths helpers
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gps_to_xy(self, lat, lon):
        x = (
            self.EARTH_RADIUS
            * math.radians(lon - self.ref_lon)
            * math.cos(math.radians(self.ref_lat))
        )
        y = self.EARTH_RADIUS * math.radians(lat - self.ref_lat)
        return x, y

    def gps_xy_to_odom(self, gps_x, gps_y):
        """
        Convert a point from GPS-XY frame to odom frame.
        Uses the follower's own position in both frames to compute the offset.
        """
        if self.follower_gps_position is None or self.follower_odom_position is None:
            return gps_x, gps_y

        offset_x = self.follower_odom_position[0] - self.follower_gps_position[0]
        offset_y = self.follower_odom_position[1] - self.follower_gps_position[1]

        return gps_x + offset_x, gps_y + offset_y

    # ================================================================
    # Bin ready publisher
    # ================================================================

    def publish_bin_ready(self):
        msg = Bool()
        msg.data = self.bin_is_ready
        self.bin_ready_pub.publish(msg)

    def set_bin_ready(self, ready):
        if ready != self.bin_is_ready:
            self.bin_is_ready = ready
            self.get_logger().info(f"Bin ready: {ready}")

    # ================================================================
    # Follower status publisher (continuous — every 0.5s)
    # ================================================================

    def publish_follower_status(self):
        self.follower_seq += 1

        msg = FollowerStatus()
        msg.seq = self.follower_seq
        msg.parked = (self.state == self.STATE_WAITING)
        msg.bin_ready = self.bin_is_ready

        if self.follower_gps_position is not None:
            msg.park_x = self.follower_gps_position[0]
            msg.park_y = self.follower_gps_position[1]
        else:
            msg.park_x = 0.0
            msg.park_y = 0.0

        self.follower_status_pub.publish(msg)

    # ================================================================
    # Leader pose callback
    # ================================================================

    def leader_pose_callback(self, msg: LeaderPose):
        if msg.seq <= self.last_leader_seq:
            return
        self.last_leader_seq = msg.seq

        # Reference point from first leader message
        if self.auto_ref and not self.ref_set:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_set = True
            self.get_logger().info(
                f"Reference point set from leader: "
                f"({self.ref_lat:.8f}, {self.ref_lon:.8f})"
            )

        if not self.ref_set:
            return

        # Update leader position in GPS-XY frame
        x, y = self.gps_to_xy(msg.latitude, msg.longitude)
        self.leader_position = (x, y)

        # Update leader yaw
        self.leader_yaw = self.quaternion_to_yaw(
            0.0, 0.0, msg.orientation_z, msg.orientation_w
        )

        # Reset when litter is cleared (pickup complete)
        if not msg.call_bin:
            self.set_bin_ready(False)
            self.parked_for_current_call = False

        # Handle call_bin while FOLLOWING -> start parking
        if msg.call_bin and self.state == self.STATE_FOLLOWING:
            self.get_logger().info("Call bin received — switching to PARKING mode.")
            with self.lock:
                if self.is_navigating and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.is_navigating = False
                self.state = self.STATE_PARKING_APPROACH
                self.parked_for_current_call = False
            self.set_bin_ready(False)
            return

        # Handle call_bin while already WAITING
        if msg.call_bin and self.state == self.STATE_WAITING:
            if self.parked_for_current_call:
                # Same call — already parked, stay put
                if not self.bin_is_ready:
                    self.set_bin_ready(True)
                return
            else:
                # New call — need to re-park behind leader
                self.get_logger().info("New call_bin — re-parking behind leader.")
                with self.lock:
                    self.state = self.STATE_PARKING_APPROACH
                    self.parked_for_current_call = False
                self.set_bin_ready(False)
                return

        # Handle call_bin while mid-parking (PARKING_APPROACH or PARKING_FINAL)
        if msg.call_bin and self.state in (
            self.STATE_PARKING_APPROACH, self.STATE_PARKING_FINAL
        ):
            # Already parking — let it finish
            return

        # Drop breadcrumbs (only in FOLLOWING state)
        if self.state != self.STATE_FOLLOWING:
            return
        if self.last_breadcrumb_position is None:
            self.last_breadcrumb_position = (x, y)
            return

        dx = x - self.last_breadcrumb_position[0]
        dy = y - self.last_breadcrumb_position[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist >= self.breadcrumb_distance:
            with self.lock:
                odom_x, odom_y = self.gps_xy_to_odom(x, y)
                self.breadcrumbs.append((odom_x, odom_y))
                count = len(self.breadcrumbs)
            self.last_breadcrumb_position = (x, y)
            self.get_logger().info(
                f"Breadcrumb #{count} dropped at XY ({x:.2f}, {y:.2f})"
            )

    # ================================================================
    # Local sensor callbacks
    # ================================================================

    def odom_callback(self, msg: Odometry):
        self.follower_odom_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        q = msg.pose.pose.orientation
        self.follower_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def gps_callback(self, msg: NavSatFix):
        if not self.ref_set:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        x, y = self.gps_to_xy(msg.latitude, msg.longitude)
        self.follower_gps_position = (x, y)

    # ================================================================
    # Trail distance (breadcrumbs are stored in odom frame)
    # ================================================================

    def compute_trail_distance(self):
        total = 0.0
        for i in range(self.current_goal_index, len(self.breadcrumbs) - 1):
            dx = self.breadcrumbs[i + 1][0] - self.breadcrumbs[i][0]
            dy = self.breadcrumbs[i + 1][1] - self.breadcrumbs[i][1]
            total += math.sqrt(dx * dx + dy * dy)
        if len(self.breadcrumbs) > 0 and self.leader_position is not None:
            last = self.breadcrumbs[-1]
            leader_odom_x, leader_odom_y = self.gps_xy_to_odom(
                self.leader_position[0], self.leader_position[1]
            )
            dx = leader_odom_x - last[0]
            dy = leader_odom_y - last[1]
            total += math.sqrt(dx * dx + dy * dy)
        return total

    # ================================================================
    # State machine
    # ================================================================

    def state_machine_tick(self):
        if self.state == self.STATE_FOLLOWING:
            self.tick_following()
        elif self.state == self.STATE_PARKING_APPROACH:
            self.tick_parking_approach()
        elif self.state == self.STATE_PARKING_FINAL:
            self.tick_parking_final()
        elif self.state == self.STATE_WAITING:
            self.tick_waiting()

    def tick_following(self):
        with self.lock:
            if len(self.breadcrumbs) == 0:
                return
            if self.current_goal_index >= len(self.breadcrumbs):
                return
            if self.is_navigating:
                return
            if self.leader_position is not None and self.follower_gps_position is not None:
                trail_dist = self.compute_trail_distance()
                if trail_dist < self.min_follow_distance:
                    return
        self.send_breadcrumb_goal()

    def tick_parking_approach(self):
        """Phase 1: drive to approach_distance behind leader using leader's heading."""
        if self.leader_position is None or self.leader_yaw is None:
            return
        if self.follower_odom_position is None:
            return
        with self.lock:
            if self.is_navigating:
                return

        total_offset = (
            self.leader_half_length
            + self.approach_distance
            + self.follower_half_length
        )

        # Leader position in odom frame
        leader_odom_x, leader_odom_y = self.gps_xy_to_odom(
            self.leader_position[0], self.leader_position[1]
        )

        # Position directly behind the leader based on leader's heading
        behind_angle = self.leader_yaw + math.pi
        target_x = leader_odom_x + total_offset * math.cos(behind_angle)
        target_y = leader_odom_y + total_offset * math.sin(behind_angle)

        # Check if already close enough to skip to phase 2
        dx = target_x - self.follower_odom_position[0]
        dy = target_y - self.follower_odom_position[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.goal_tolerance:
            self.get_logger().info(
                f"[PARKING Phase 1] Already within {dist:.2f}m — skipping to Phase 2."
            )
            with self.lock:
                self.state = self.STATE_PARKING_FINAL
            return

        # Orientation: face the leader
        angle_to_leader = math.atan2(
            leader_odom_y - target_y,
            leader_odom_x - target_x
        )
        q = Quaternion()
        q.z = math.sin(angle_to_leader / 2.0)
        q.w = math.cos(angle_to_leader / 2.0)

        self.get_logger().info(
            f"[PARKING Phase 1] Leader odom ({leader_odom_x:.2f}, {leader_odom_y:.2f}), "
            f"target ({target_x:.2f}, {target_y:.2f}), dist {dist:.2f}m"
        )
        self.send_park_goal(target_x, target_y, q)

    def tick_parking_final(self):
        """Phase 2: close remaining gap to park_distance behind leader."""
        if self.leader_position is None or self.leader_yaw is None:
            return
        if self.follower_odom_position is None:
            return
        with self.lock:
            if self.is_navigating:
                return

        total_offset = (
            self.leader_half_length
            + self.park_distance
            + self.follower_half_length
        )

        # Leader position in odom frame
        leader_odom_x, leader_odom_y = self.gps_xy_to_odom(
            self.leader_position[0], self.leader_position[1]
        )

        # Position directly behind the leader based on leader's heading
        behind_angle = self.leader_yaw + math.pi
        target_x = leader_odom_x + total_offset * math.cos(behind_angle)
        target_y = leader_odom_y + total_offset * math.sin(behind_angle)

        # Orientation: face the leader
        angle_to_leader = math.atan2(
            leader_odom_y - target_y,
            leader_odom_x - target_x
        )
        q = Quaternion()
        q.z = math.sin(angle_to_leader / 2.0)
        q.w = math.cos(angle_to_leader / 2.0)

        self.get_logger().info(
            f"[PARKING Phase 2] Leader odom ({leader_odom_x:.2f}, {leader_odom_y:.2f}), "
            f"target ({target_x:.2f}, {target_y:.2f})"
        )
        self.send_park_goal(target_x, target_y, q)

    def tick_waiting(self):
        # Use GPS positions for resume distance (same frame)
        if self.leader_position is None or self.follower_gps_position is None:
            return

        dist_to_leader = math.sqrt(
            (self.leader_position[0] - self.follower_gps_position[0]) ** 2
            + (self.leader_position[1] - self.follower_gps_position[1]) ** 2
        )

        if dist_to_leader > self.resume_distance:
            self.get_logger().info(
                f"Leader moved away ({dist_to_leader:.2f}m) — resuming FOLLOWING."
            )
            self.set_bin_ready(False)
            self.parked_for_current_call = False
            with self.lock:
                self.state = self.STATE_FOLLOWING
                self.breadcrumbs.clear()
                self.current_goal_index = 0
                leader_odom_x, leader_odom_y = self.gps_xy_to_odom(
                    self.leader_position[0], self.leader_position[1]
                )
                self.last_breadcrumb_position = (
                    self.leader_position[0], self.leader_position[1]
                )
                self.breadcrumbs.append((leader_odom_x, leader_odom_y))
            self.get_logger().info(
                f"Fresh breadcrumb at leader XY "
                f"({self.leader_position[0]:.2f}, {self.leader_position[1]:.2f})"
            )

    # ================================================================
    # Goal sending
    # ================================================================

    def send_breadcrumb_goal(self):
        with self.lock:
            if self.current_goal_index >= len(self.breadcrumbs):
                return
            target = self.breadcrumbs[self.current_goal_index]
            self.is_navigating = True
            total = len(self.breadcrumbs)
            index = self.current_goal_index

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
            return

        # Breadcrumbs are already in odom frame
        target_odom_x, target_odom_y = target[0], target[1]

        # Calculate heading from follower to breadcrumb
        if self.follower_odom_position is not None:
            dx = target_odom_x - self.follower_odom_position[0]
            dy = target_odom_y - self.follower_odom_position[1]
            heading = math.atan2(dy, dx)
        else:
            heading = 0.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_odom_x
        goal_msg.pose.pose.position.y = target_odom_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        self.get_logger().info(
            f"[FOLLOWING] Breadcrumb {index + 1}/{total}: "
            f"-> odom ({target_odom_x:.2f}, {target_odom_y:.2f})"
        )

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.breadcrumb_response_callback)

    def send_park_goal(self, x, y, orientation):
        with self.lock:
            self.is_navigating = True

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info(f"[PARKING] Sending goal at odom XY ({x:.2f}, {y:.2f})")

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.park_response_callback)

    # ================================================================
    # Breadcrumb goal callbacks
    # ================================================================

    def breadcrumb_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("Failed to get goal handle.")
            with self.lock:
                self.is_navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Breadcrumb goal rejected.")
            with self.lock:
                self.is_navigating = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.breadcrumb_result_callback)

    def breadcrumb_result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().warn("No breadcrumb result received.")
            with self.lock:
                self.is_navigating = False
            return

        status = result.status
        with self.lock:
            self.is_navigating = False
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Breadcrumb {self.current_goal_index + 1} reached."
                )
                self.current_goal_index += 1
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    f"Breadcrumb {self.current_goal_index + 1} aborted. Skipping."
                )
                self.current_goal_index += 1
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Breadcrumb goal cancelled.")

    # ================================================================
    # Park goal callbacks
    # ================================================================

    def park_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("Failed to get park goal handle.")
            with self.lock:
                self.is_navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Park goal rejected.")
            with self.lock:
                self.is_navigating = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.park_result_callback)

    def park_result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().warn("No park result received.")
            with self.lock:
                self.is_navigating = False
            return

        status = result.status

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.state == self.STATE_PARKING_APPROACH:
                    self.get_logger().info(
                        "=== Phase 1 complete — starting final approach ==="
                    )
                    self.state = self.STATE_PARKING_FINAL
                elif self.state == self.STATE_PARKING_FINAL:
                    self.get_logger().info("=== PARKED — switching to WAITING ===")
                    self.state = self.STATE_WAITING
                    self.set_bin_ready(True)
                    self.parked_for_current_call = True

            elif status == GoalStatus.STATUS_ABORTED:
                if self.state == self.STATE_PARKING_APPROACH:
                    self.get_logger().warn(
                        "Phase 1 aborted — advancing to Phase 2."
                    )
                    self.state = self.STATE_PARKING_FINAL
                elif self.state == self.STATE_PARKING_FINAL:
                    self.get_logger().warn("Phase 2 aborted — parking here.")
                    self.state = self.STATE_WAITING
                    self.set_bin_ready(True)
                    self.parked_for_current_call = True
                else:
                    self.get_logger().warn("Park goal aborted.")

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Park goal cancelled.")

    # ================================================================
    # Feedback
    # ================================================================

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = GPSFollowerBehaviorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GPS follower behaviour node.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()