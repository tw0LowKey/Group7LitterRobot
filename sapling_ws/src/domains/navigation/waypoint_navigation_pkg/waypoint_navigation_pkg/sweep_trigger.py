#!/usr/bin/env python3

import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
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
        - Waits startup_detection_grace_seconds first,
          so the CV node has time to start and publish detections.
        - After the countdown, waits for manual_start_topic=True before starting navigation.
        - Accepts litter only if it is within litter_accept_radius of the Scout.
        - If accepted litter appears during waypoint navigation, waypoint navigation is interrupted.
        - Navigates to the litter approach pose.
        - Rotates to face litter.
        - Publishes /call_bin=True when litter is reached.
        - Publishes /start_grasp=True when litter is reached.
        - Waits for external grasp/pickup system to publish /start_grasp=False.
        - After /start_grasp=True, ignores CV queue updates while pickup is running.
        - After external /start_grasp=False, clears old litter goals and performs a 10 second post-pickup check.
        - During that check, if CV still sees litter in front and close enough,
          /start_grasp=True is published again. Otherwise waypoint navigation resumes.
        - When several litter targets exist, chooses the closest/front-most target first.

    Important:
        /start_grasp does NOT mean navigation active.
        /start_grasp=True means start grasp/pickup now.
        /start_grasp=False means grasp/pickup is complete or idle.

    Required external inputs:
        /odom
        /vision/detected_litter
        /vision/detected_litter_base

    Required external systems:
        Nav2 running with /navigate_to_pose action server.
        /vision/detected_litter publisher already running.

    Optional external:
        A pickup/grasp coordination node that publishes:
            /start_grasp=False
        after pickup is complete.
    """

    STATE_WAITING_ODOM = "WAITING_ODOM"
    STATE_WAITING_START_TRIGGER = "WAITING_START_TRIGGER"
    STATE_WAYPOINT_NAV = "WAYPOINT_NAV"
    STATE_LITTER_NAV = "LITTER_NAV"
    STATE_ROTATE_TO_LITTER = "ROTATE_TO_LITTER"
    STATE_WAITING_PICKUP = "WAITING_PICKUP"
    STATE_POST_PICKUP_CHECK = "POST_PICKUP_CHECK"
    STATE_DONE = "DONE"

    GOAL_NONE = "NONE"
    GOAL_WAYPOINT = "WAYPOINT"
    GOAL_LITTER_APPROACH = "LITTER_APPROACH"
    GOAL_LITTER_ROTATE = "LITTER_ROTATE"

    def __init__(self):
        super().__init__("single_sweep_litter_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        # ============================================================
        # QoS
        # ============================================================
        # Latched QoS: late subscribers get the latest Bool value.
        self.latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("litter_topic", "/vision/detected_litter")
        self.declare_parameter("litter_base_topic", "/vision/detected_litter_base")

        # Latched grasp trigger / handshake topic.
        self.declare_parameter("start_grasp_topic", "/start_grasp")

        self.declare_parameter("call_bin_topic", "/call_bin")
        self.declare_parameter("queue_size_topic", "/queue_size")

        # Option A: everything in odom
        self.declare_parameter("goal_frame", "odom")

        # Straight-line waypoint generation
        self.declare_parameter("num_waypoints", 3)
        self.declare_parameter("spacing", 1.0)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        # Startup behaviour
        # Gives the CV node time to start before this node sends any Nav2 goals.
        self.declare_parameter("startup_detection_grace_seconds", 30.0)

        # Manual start trigger after startup countdown.
        # Default is /drive_front_3m so this command starts the sweep:
        # ros2 topic pub --once /drive_front_3m std_msgs/msg/Bool "{data: true}"
        self.declare_parameter("manual_start_topic", "/drive_front_3m")

        # Litter behaviour
        self.declare_parameter("litter_accept_radius", 2.0)
        self.declare_parameter("approach_offset", 0.4)
        self.declare_parameter("grasp_trigger_distance", 0.55)
        self.declare_parameter("duplicate_litter_distance", 0.25)
        self.declare_parameter("post_pickup_check_seconds", 10.0)
        self.declare_parameter("front_angle_threshold_deg", 45.0)
        self.declare_parameter("front_litter_angle_weight", 0.3)

        # Testing convenience.
        # If true, it automatically simulates pickup complete after pickup_wait_seconds.
        # For the real system, keep this false and let the grasp node publish /start_grasp=False.
        self.declare_parameter("auto_complete_pickup", False)
        self.declare_parameter("pickup_wait_seconds", 3.0)

        self.nav_action = self.get_parameter("nav_action").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.litter_topic = self.get_parameter("litter_topic").value
        self.litter_base_topic = self.get_parameter("litter_base_topic").value
        self.start_grasp_topic = self.get_parameter("start_grasp_topic").value
        self.call_bin_topic = self.get_parameter("call_bin_topic").value
        self.queue_size_topic = self.get_parameter("queue_size_topic").value

        self.goal_frame = self.get_parameter("goal_frame").value

        self.num_waypoints = int(self.get_parameter("num_waypoints").value)
        self.spacing = float(self.get_parameter("spacing").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)

        self.startup_detection_grace_seconds = float(
            self.get_parameter("startup_detection_grace_seconds").value
        )
        self.manual_start_topic = self.get_parameter("manual_start_topic").value
        self.start_trigger_received = False
        self.litter_accept_radius = float(
            self.get_parameter("litter_accept_radius").value
        )
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.grasp_trigger_distance = float(
            self.get_parameter("grasp_trigger_distance").value
        )
        self.duplicate_litter_distance = float(
            self.get_parameter("duplicate_litter_distance").value
        )
        self.post_pickup_check_seconds = float(
            self.get_parameter("post_pickup_check_seconds").value
        )
        self.front_angle_threshold = math.radians(
            float(self.get_parameter("front_angle_threshold_deg").value)
        )
        self.front_litter_angle_weight = float(
            self.get_parameter("front_litter_angle_weight").value
        )
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
        self.node_start_time = self.get_clock().now()

        self.waypoints = self.generate_waypoints()
        self.current_waypoint_index = 0

        self.litter_queue = deque()
        self.current_litter_pose = None
        self.current_litter_base_pose = None
        self.latest_base_litter_poses = deque(maxlen=20)

        self.goal_handle = None
        self.active_goal_type = self.GOAL_NONE
        self.goal_active = False
        self.cancel_requested = False
        self.ignore_next_cancel_result = False

        # Internal copy of latest start_grasp state.
        # False = idle/no grasp running.
        # True = grasp requested/running.
        self.start_grasp_active = False

        self.pickup_timer = None
        self.post_pickup_timer = None
        self.post_pickup_check_started_time = None
        self.post_pickup_seen_litter = False


        # ============================================================
        # Publishers
        # ============================================================

        self.start_grasp_pub = self.create_publisher(
            Bool,
            self.start_grasp_topic,
            self.latch_qos,
        )

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

        self.litter_base_sub = self.create_subscription(
            PoseStamped,
            self.litter_base_topic,
            self.litter_base_callback,
            10,
            callback_group=self.cb_group,
        )

        self.start_grasp_sub = self.create_subscription(
            Bool,
            self.start_grasp_topic,
            self.start_grasp_callback,
            self.latch_qos,
            callback_group=self.cb_group,
        )

        self.manual_start_sub = self.create_subscription(
            Bool,
            self.manual_start_topic,
            self.manual_start_callback,
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
        self.get_logger().info(f"Litter base topic:   {self.litter_base_topic}")
        self.get_logger().info(f"Start grasp topic:   {self.start_grasp_topic}")
        self.get_logger().info(f"Call bin topic:      {self.call_bin_topic}")
        self.get_logger().info(f"Queue size topic:    {self.queue_size_topic}")
        self.get_logger().info(f"Manual start topic:  {self.manual_start_topic}")
        self.get_logger().info(f"Goal frame:          {self.goal_frame}")
        self.get_logger().info(f"Num waypoints:       {self.num_waypoints}")
        self.get_logger().info(f"Spacing:             {self.spacing:.2f} m")
        self.get_logger().info(
            f"Start:               ({self.start_x:.2f}, {self.start_y:.2f})"
        )
        self.get_logger().info(
            f"Startup grace:       {self.startup_detection_grace_seconds:.1f} s"
        )
        self.get_logger().info(
            f"Litter radius:       {self.litter_accept_radius:.2f} m"
        )
        self.get_logger().info(
            f"Approach offset:     {self.approach_offset:.2f} m"
        )
        self.get_logger().info(
            f"Grasp trigger dist:  {self.grasp_trigger_distance:.2f} m"
        )
        self.get_logger().info(f"Duplicate distance: {self.duplicate_litter_distance:.2f} m")
        self.get_logger().info(
            f"Post-pickup check:  {self.post_pickup_check_seconds:.1f} s"
        )
        self.get_logger().info(
            f"Front angle thresh: {math.degrees(self.front_angle_threshold):.1f} deg"
        )
        self.get_logger().info(f"Auto pickup:         {self.auto_complete_pickup}")
        self.get_logger().info("Generated waypoints:")

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f"  WP{i + 1}: ({wp.pose.position.x:.2f}, "
                f"{wp.pose.position.y:.2f}) in {wp.header.frame_id}"
            )

        self.get_logger().info("Waiting for /odom...")
        self.get_logger().info(
            f"After countdown, start with: ros2 topic pub --once "
            f"{self.manual_start_topic} std_msgs/msg/Bool '{{data: true}}'"
        )

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

    def manual_start_callback(self, msg: Bool):
        """
        Manual start gate.

        The node still performs the startup countdown first. After that,
        navigation only begins when this topic receives True.
        """

        if not msg.data:
            return

        with self.lock:
            self.start_trigger_received = True

            if self.state == self.STATE_WAITING_START_TRIGGER:
                self.get_logger().warn(
                    "Manual start trigger received. Starting waypoint/litter navigation."
                )
                self.state = self.STATE_WAYPOINT_NAV

        self.publish_start_grasp(False)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def litter_base_callback(self, msg: PoseStamped):
        """
        Stores litter detections in base_link frame.

        This topic is NOT used for Nav2 goals. It is used only for immediate
        robot-relative checks, especially during the 10s post-pickup check:
            x > 0  -> litter is in front of Scout
            y      -> side offset
            sqrt(x^2 + y^2) -> distance from Scout/base_link
        """

        if msg.header.frame_id and msg.header.frame_id != "base_link":
            self.get_logger().warn(
                f"Expected base_link litter on {self.litter_base_topic}, "
                f"got frame '{msg.header.frame_id}'."
            )

        with self.lock:
            self.latest_base_litter_poses.append(msg)

        if self.state == self.STATE_POST_PICKUP_CHECK:
            if self.is_base_litter_in_front_and_close(msg):
                self.get_logger().warn(
                    f"Post-pickup check: base_link litter still visible in front: "
                    f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}. "
                    "Requesting grasp again."
                )
                self.post_pickup_seen_litter = True
                self.trigger_grasp()

    def get_latest_base_litter_locked(self):
        """
        Returns the latest base_link litter detection.
        Assumes self.lock is already held.

        The CV node publishes /vision/detected_litter_base immediately before
        /vision/detected_litter for each centroid, so this gives the matching
        base-frame decision coordinates for the odom-frame navigation target.
        """
        if len(self.latest_base_litter_poses) == 0:
            return None
        return self.latest_base_litter_poses[-1]

    def litter_callback(self, msg: PoseStamped):
        """
        Accepts odom-frame litter for navigation goals.

        /vision/detected_litter is expected in odom frame and is used for Nav2.
        /vision/detected_litter_base is used separately for robot-relative
        front/close decisions.
        """

        if self.current_pose is None:
            self.get_logger().warn(
                "Ignoring litter because robot pose from /odom is unknown."
            )
            return

        if msg.header.frame_id and msg.header.frame_id != self.goal_frame:
            self.get_logger().warn(
                f"Received litter in frame '{msg.header.frame_id}', but this node "
                f"is using '{self.goal_frame}' for Nav2 goals."
            )

        dx = msg.pose.position.x - self.current_pose.position.x
        dy = msg.pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.litter_accept_radius:
            self.get_logger().info(
                f"Ignoring odom litter at ({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}); distance {dist:.2f} m is outside "
                f"radius {self.litter_accept_radius:.2f} m."
            )
            return

        # Force litter frame to odom for Nav2 goals.
        msg.header.frame_id = self.goal_frame

        # During post-pickup, do not queue odom targets. The retry decision is
        # made only from /vision/detected_litter_base so odom drift/yaw does not
        # affect the "is it in front?" check.
        if self.state == self.STATE_POST_PICKUP_CHECK:
            return

        # While pickup is running, ignore CV queue updates. This prevents the
        # same litter from being re-added while the arm is picking it up.
        # Old litter goals are cleared only after external /start_grasp=False.
        if self.state == self.STATE_WAITING_PICKUP:
            return

        with self.lock:
            if self.is_duplicate_litter_locked(msg):
                self.get_logger().info(
                    f"Ignoring duplicate litter near ({msg.pose.position.x:.2f}, "
                    f"{msg.pose.position.y:.2f})."
                )
                return

            base_msg = self.get_latest_base_litter_locked()
            self.litter_queue.append({
                "odom": msg,
                "base": base_msg,
            })
            queue_size = len(self.litter_queue)

        self.publish_queue_size()

        base_text = "no base_link pair"
        if base_msg is not None:
            base_text = (
                f"base=({base_msg.pose.position.x:.2f}, "
                f"{base_msg.pose.position.y:.2f})"
            )

        self.get_logger().warn(
            f"Accepted litter odom=({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}); odom distance {dist:.2f} m; "
            f"{base_text}. Queue size: {queue_size}"
        )

        # Interrupt waypoint navigation if currently driving a waypoint.
        if self.state == self.STATE_WAYPOINT_NAV:
            self.get_logger().warn(
                "Litter has priority — cancelling current waypoint goal."
            )
            self.cancel_current_goal_for_litter()

    def start_grasp_callback(self, msg: Bool):
        """
        /start_grasp handshake.

        This node publishes:
            True when litter is reached and grasp should start.

        External grasp/pickup node publishes:
            False when grasp/pickup is complete.

        After False is received while WAITING_PICKUP:
            - if more litter exists in queue, navigate to next litter.
            - if no litter exists, keep /start_grasp=False and resume waypoint navigation.

        Important:
            We do NOT publish /start_grasp=True just because litter_queue is non-empty.
            We first navigate to the next litter, then publish True only after reaching it.
        """

        requested_start_grasp = bool(msg.data)

        with self.lock:
            self.start_grasp_active = requested_start_grasp

        # True usually comes from this node when it reaches litter.
        # Nothing else to do here.
        if requested_start_grasp:
            return

        # False only means pickup complete if we are actually waiting for pickup.
        if self.state != self.STATE_WAITING_PICKUP:
            return

        self.get_logger().info(
            "/start_grasp=False received while waiting for pickup. Starting 10s post-pickup check."
        )

        self.start_post_pickup_check()

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_current_yaw(self) -> float:
        """Get robot yaw from current odometry quaternion."""
        if self.current_pose is None:
            return 0.0

        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_litter_distance_and_relative_angle(self, litter_pose: PoseStamped):
        """Return base-to-litter distance and angle relative to robot heading."""
        if self.current_pose is None:
            return None, None

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        litter_x = litter_pose.pose.position.x
        litter_y = litter_pose.pose.position.y

        dx = litter_x - robot_x
        dy = litter_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        angle_to_litter = math.atan2(dy, dx)
        rel_angle = self.normalize_angle(angle_to_litter - self.get_current_yaw())

        return dist, rel_angle

    def is_base_litter_in_front_and_close(self, base_pose: PoseStamped) -> bool:
        """
        Used after pickup completes.

        This uses base_link coordinates from /vision/detected_litter_base, so no
        odom yaw calculation is needed:
            x > 0 means in front of the Scout.
            y is side offset.
        """
        if base_pose is None:
            return False

        x = base_pose.pose.position.x
        y = base_pose.pose.position.y
        dist = math.sqrt(x * x + y * y)

        # Keep same practical distance idea as before.
        post_pickup_distance_threshold = self.approach_offset + 0.30

        # Convert the old angle threshold into a side limit at this distance.
        # This is equivalent to a front cone but uses base_link x/y directly.
        if x <= 0.0:
            return False

        rel_angle = math.atan2(y, x)

        if dist > post_pickup_distance_threshold:
            return False

        if abs(rel_angle) > self.front_angle_threshold:
            return False

        self.get_logger().info(
            f"Post-pickup base candidate in front: x={x:.2f} m, y={y:.2f} m, "
            f"dist={dist:.2f} m, angle={math.degrees(rel_angle):.1f} deg, "
            f"threshold={post_pickup_distance_threshold:.2f} m."
        )
        return True

    def get_litter_item_score_locked(self, item) -> float:
        """
        Score queued litter using odom distance for main priority and base_link
        angle when available for a small front-most preference.
        Assumes self.lock is already held.
        """
        odom_pose = item["odom"]
        base_pose = item.get("base")

        dist, rel_angle = self.get_litter_distance_and_relative_angle(odom_pose)
        if dist is None:
            dist = 999.0

        # Prefer front-most target using base_link angle if available. This
        # avoids depending on odom yaw for the front preference.
        if base_pose is not None:
            bx = base_pose.pose.position.x
            by = base_pose.pose.position.y
            angle_penalty = abs(math.atan2(by, bx)) if bx != 0.0 or by != 0.0 else 0.0
        elif rel_angle is not None:
            angle_penalty = abs(rel_angle)
        else:
            angle_penalty = 0.0

        return dist + self.front_litter_angle_weight * angle_penalty

    def pop_best_litter_locked(self):
        """
        Choose closest/front-most litter instead of raw FIFO order.
        Queue items store both:
            item["odom"] -> Nav2 goal coordinates
            item["base"] -> robot-relative decision coordinates
        Assumes self.lock is already held.
        """
        if len(self.litter_queue) == 0:
            return None

        best_index = 0
        best_score = None

        for i, item in enumerate(self.litter_queue):
            score = self.get_litter_item_score_locked(item)
            if best_score is None or score < best_score:
                best_score = score
                best_index = i

        best_item = self.litter_queue[best_index]
        del self.litter_queue[best_index]

        odom_pose = best_item["odom"]
        base_pose = best_item.get("base")
        dist, rel_angle = self.get_litter_distance_and_relative_angle(odom_pose)

        if dist is not None:
            if base_pose is not None:
                self.get_logger().info(
                    f"Selected best litter: odom_dist={dist:.2f} m, "
                    f"base=({base_pose.pose.position.x:.2f}, "
                    f"{base_pose.pose.position.y:.2f}), score={best_score:.2f}."
                )
            elif rel_angle is not None:
                self.get_logger().info(
                    f"Selected best litter: odom_dist={dist:.2f} m, "
                    f"rel_angle={math.degrees(rel_angle):.1f} deg, "
                    f"score={best_score:.2f}."
                )

        return best_item

    def is_duplicate_litter_locked(self, msg: PoseStamped) -> bool:
        """
        Assumes self.lock is already held.
        Prevents the same odom-frame litter point being queued repeatedly when
        the vision node republishes the same centroid.
        """

        x = msg.pose.position.x
        y = msg.pose.position.y

        poses_to_check = []
        if self.current_litter_pose is not None:
            poses_to_check.append(self.current_litter_pose)

        for item in self.litter_queue:
            poses_to_check.append(item["odom"])

        for existing in poses_to_check:
            ex = existing.pose.position.x
            ey = existing.pose.position.y
            d = math.sqrt((x - ex) ** 2 + (y - ey) ** 2)
            if d < self.duplicate_litter_distance:
                return True

        return False

    # ============================================================
    # Main tick
    # ============================================================

    def tick(self):
        if self.current_pose is None:
            return

        # Safety/robustness: if the robot gets close enough to the litter,
        # stop Nav2 and trigger grasp immediately instead of driving over it.
        if self.check_grasp_trigger_distance():
            return

        if self.state == self.STATE_WAITING_ODOM:
            elapsed = (
                self.get_clock().now() - self.node_start_time
            ).nanoseconds / 1e9

            if elapsed < self.startup_detection_grace_seconds:
                # Do not send waypoint goals yet. CV detections are still accepted
                # by litter_callback during this time.
                remaining = self.startup_detection_grace_seconds - elapsed
                self.get_logger().info(
                    f"Waiting {remaining:.1f}s for CV node startup before navigation."
                )
                self.publish_start_grasp(False)
                return

            self.get_logger().info(
                "Startup grace complete. Waiting for manual start trigger."
            )
            self.state = self.STATE_WAITING_START_TRIGGER
            # Ensure latched topic starts as False/idle.
            self.publish_start_grasp(False)
            return

        if self.state == self.STATE_WAITING_START_TRIGGER:
            if not self.start_trigger_received:
                self.get_logger().info(
                    f"Waiting for manual start. Run: ros2 topic pub --once "
                    f"{self.manual_start_topic} std_msgs/msg/Bool '{{data: true}}'"
                )
                self.publish_start_grasp(False)
                return

            self.get_logger().warn(
                "Manual start trigger already received. Starting waypoint/litter navigation."
            )
            self.state = self.STATE_WAYPOINT_NAV
            self.publish_start_grasp(False)

        if self.goal_active:
            return

        if self.state == self.STATE_WAYPOINT_NAV:
            # Litter priority.
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

        if self.state == self.STATE_POST_PICKUP_CHECK:
            self.handle_post_pickup_check_tick()
            return

        if self.state == self.STATE_DONE:
            return

    # ============================================================
    # Grasp trigger safety check
    # ============================================================

    def check_grasp_trigger_distance(self) -> bool:
        """
        If the robot is close enough to current litter, stop Nav2 and trigger grasp.
        This prevents the Scout from driving over the litter if Nav2 overshoots
        or if the approach goal is too close.
        """

        if self.current_pose is None or self.current_litter_pose is None:
            return False

        if self.state not in [
            self.STATE_LITTER_NAV,
            self.STATE_ROTATE_TO_LITTER,
        ]:
            return False

        if self.state == self.STATE_WAITING_PICKUP:
            return False

        dx = self.current_litter_pose.pose.position.x - self.current_pose.position.x
        dy = self.current_litter_pose.pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.grasp_trigger_distance:
            return False

        self.get_logger().warn(
            f"Robot is within grasp trigger distance of litter: {dist:.2f} m. "
            "Cancelling Nav2 and publishing /start_grasp=True."
        )

        if self.goal_active and self.goal_handle is not None:
            self.cancel_current_goal_for_grasp()

        self.trigger_grasp()

        return True

    # ============================================================
    # Navigation goal helpers
    # ============================================================

    def send_next_waypoint_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.state = self.STATE_DONE
            self.publish_start_grasp(False)
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
        popped_new_litter = False

        with self.lock:
            if self.current_litter_pose is None:
                if len(self.litter_queue) == 0:
                    self.state = self.STATE_WAYPOINT_NAV
                    return

                next_litter = self.pop_best_litter_locked()
                if next_litter is None:
                    self.state = self.STATE_WAYPOINT_NAV
                    return

                self.current_litter_pose = next_litter["odom"]
                self.current_litter_base_pose = next_litter.get("base")
                popped_new_litter = True

            current_litter_pose = self.current_litter_pose

        if popped_new_litter:
            self.publish_queue_size()

        approach_pose = self.get_approach_pose(current_litter_pose)

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
            ignore_cancel_result = self.ignore_next_cancel_result

            self.goal_active = False
            self.active_goal_type = self.GOAL_NONE
            self.goal_handle = None
            self.cancel_requested = False
            self.ignore_next_cancel_result = False

        if was_cancel_requested:
            self.get_logger().info("Goal cancellation completed.")

        if ignore_cancel_result and status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                "Ignoring cancelled Nav2 result because grasp has already been triggered."
            )
            return

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
            if len(self.litter_queue) > 0 or self.current_litter_pose is not None:
                self.get_logger().warn(
                    "Waypoint was cancelled because litter is pending. "
                    "Keeping state as LITTER_NAV."
                )
                self.state = self.STATE_LITTER_NAV
                return

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f"Waypoint {self.current_waypoint_index + 1} aborted. "
                f"Skipping to next waypoint."
            )
            self.current_waypoint_index += 1

        # Litter still has priority.
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
            self.current_litter_base_pose = None
            self.state = self.STATE_WAYPOINT_NAV

    def handle_litter_rotate_finished(self, status: int):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Facing litter. Calling bin/pickup.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Rotate-to-litter aborted. Calling bin anyway.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Rotate-to-litter cancelled.")
            self.current_litter_pose = None
            self.current_litter_base_pose = None
            self.state = self.STATE_WAYPOINT_NAV
            return

        # Reached litter behaviour:
        # 1. Call bin/pickup.
        # 2. Set /start_grasp=True.
        # 3. Clear current/queued litter goals so stale targets are not reused.
        # 4. Wait until external grasp node publishes /start_grasp=False.
        self.trigger_grasp()

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
            self.ignore_next_cancel_result = False
            goal_handle = self.goal_handle

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_current_goal_for_grasp(self):
        with self.lock:
            if self.goal_handle is None or not self.goal_active:
                return

            self.cancel_requested = True
            self.ignore_next_cancel_result = True
            goal_handle = self.goal_handle

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.grasp_cancel_done_callback)

    def grasp_cancel_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info(
                "Cancel request sent to Nav2 because grasp distance was reached."
            )
        except Exception as e:
            self.get_logger().warn(f"Grasp cancel request failed: {e}")

        with self.lock:
            self.goal_active = False
            self.active_goal_type = self.GOAL_NONE
            self.goal_handle = None
            self.cancel_requested = False

    def cancel_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Cancel request sent to Nav2.")
        except Exception as e:
            self.get_logger().warn(f"Cancel request failed: {e}")

        with self.lock:
            # Force-clear the old waypoint goal so tick() can immediately
            # send the litter approach goal. This avoids getting stuck after
            # Nav2 accepts the cancel but before/while the result callback runs.
            self.goal_active = False
            self.active_goal_type = self.GOAL_NONE
            self.goal_handle = None
            self.cancel_requested = False
            self.state = self.STATE_LITTER_NAV
            queue_size = len(self.litter_queue)

        self.get_logger().warn(
            f"Waypoint cancelled for litter. Queue size={queue_size}. "
            "Switching to LITTER_NAV."
        )

    def trigger_grasp(self):
        """
        Request the arm/vision pickup system to grasp.

        Important: do NOT clear current_litter_pose/litter_queue here.
        While pickup is running, CV may still see the same litter. If we clear
        immediately on /start_grasp=True, the node could accept that same litter
        as a fresh navigation target. Instead, litter data is cleared only when
        external /start_grasp changes back to False and the post-pickup check starts.
        """
        self.stop_post_pickup_timer()

        self.publish_call_bin()
        self.publish_start_grasp(True)

        self.state = self.STATE_WAITING_PICKUP

        if self.auto_complete_pickup:
            self.get_logger().info(
                f"Auto pickup enabled. Publishing /start_grasp=False in "
                f"{self.pickup_wait_seconds:.1f}s."
            )
            self.start_auto_pickup_timer()
        else:
            self.get_logger().info(
                "Waiting for external /start_grasp=False to start post-pickup check."
            )

    def start_post_pickup_check(self):
        """
        After external /start_grasp=False, clear old litter data and wait 10s.
        If a fresh CV detection appears in front and close enough, trigger grasp again.
        Otherwise resume waypoint navigation.
        """
        self.stop_post_pickup_timer()

        with self.lock:
            self.current_litter_pose = None
            self.current_litter_base_pose = None
            self.litter_queue.clear()
            self.post_pickup_seen_litter = False
            self.latest_base_litter_poses.clear()
            self.post_pickup_check_started_time = self.get_clock().now()
            self.state = self.STATE_POST_PICKUP_CHECK

        self.publish_queue_size()
        self.publish_start_grasp(False)

        self.get_logger().warn(
            f"Post-pickup check started for {self.post_pickup_check_seconds:.1f}s. "
            "If litter is still detected in front and close enough, grasp will restart."
        )

        self.post_pickup_timer = self.create_timer(
            self.post_pickup_check_seconds,
            self.post_pickup_check_timeout_callback,
            callback_group=self.cb_group,
        )

    def handle_post_pickup_check_tick(self):
        if self.post_pickup_check_started_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.post_pickup_check_started_time
        ).nanoseconds / 1e9
        remaining = max(0.0, self.post_pickup_check_seconds - elapsed)

        # Keep this lightweight; the actual trigger happens in litter_callback
        # when a fresh /vision/detected_litter_base message arrives.
        if int(remaining) in [10, 5, 3, 1]:
            self.get_logger().info(
                f"Post-pickup check active. Remaining={remaining:.1f}s."
            )

    def post_pickup_check_timeout_callback(self):
        self.stop_post_pickup_timer()

        if self.state != self.STATE_POST_PICKUP_CHECK:
            return

        if self.post_pickup_seen_litter:
            # This should usually already have triggered grasp in litter_callback.
            return

        self.get_logger().info(
            "Post-pickup check finished. No close front litter detected. "
            "Resuming waypoint navigation."
        )

        with self.lock:
            self.current_litter_pose = None
            self.current_litter_base_pose = None
            self.litter_queue.clear()
            self.post_pickup_check_started_time = None
            self.state = self.STATE_WAYPOINT_NAV

        self.publish_queue_size()
        self.publish_start_grasp(False)

    def stop_post_pickup_timer(self):
        if self.post_pickup_timer is not None:
            self.post_pickup_timer.cancel()
            self.destroy_timer(self.post_pickup_timer)
            self.post_pickup_timer = None

    def finish_current_litter_and_resume(self):
        # Kept for compatibility with older calls. New behaviour uses the
        # 10-second post-pickup check instead of immediately resuming.
        self.start_post_pickup_check()

    # ============================================================
    # Pickup / queue / grasp publishers
    # ============================================================

    def publish_call_bin(self):
        msg = Bool()
        msg.data = True
        self.call_bin_pub.publish(msg)
        self.get_logger().warn("Published /call_bin=True.")

    def publish_start_grasp(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self.start_grasp_pub.publish(msg)

        with self.lock:
            self.start_grasp_active = bool(active)

        self.get_logger().warn(f"Published /start_grasp={bool(active)}.")

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
        self.post_pickup_timer = None
        self.post_pickup_check_started_time = None
        self.post_pickup_seen_litter = False


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
        self.post_pickup_timer = None
        self.post_pickup_check_started_time = None
        self.post_pickup_seen_litter = False


        if self.state == self.STATE_WAITING_PICKUP:
            self.get_logger().info("Auto pickup complete.")
            self.publish_start_grasp(False)
            self.start_post_pickup_check()


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