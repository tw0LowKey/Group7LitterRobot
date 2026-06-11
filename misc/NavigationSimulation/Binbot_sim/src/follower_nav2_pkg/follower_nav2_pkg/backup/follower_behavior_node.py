import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus
import math
import threading
from builtin_interfaces.msg import Time


class FollowerBehaviorNode(Node):
    """
    Leader-follower behavior node with litter pickup mode.

    States:
        FOLLOWING    — following leader's breadcrumb trail
        PARKING      — driving to park_distance directly behind leader
        WAITING      — parked behind leader, waiting for leader to move away
    """

    STATE_FOLLOWING = 'FOLLOWING'
    STATE_PARKING = 'PARKING'
    STATE_WAITING = 'WAITING'

    def __init__(self):
        super().__init__('follower_behavior_node')

        # --- Parameters ---
        self.declare_parameter('breadcrumb_distance', 0.5)
        self.declare_parameter('min_follow_distance', 1.5)
        self.declare_parameter('park_distance', 0.2)
        self.declare_parameter('resume_distance', 1.0)
        self.declare_parameter('leader_odom_topic', '/odom')
        self.declare_parameter('follower_action', '/follower/navigate_to_pose')
        self.declare_parameter('litter_topic', '/litter_detected')
        self.declare_parameter('goal_tolerance', 0.3)

        self.breadcrumb_distance = self.get_parameter('breadcrumb_distance').value
        self.min_follow_distance = self.get_parameter('min_follow_distance').value
        self.park_distance = self.get_parameter('park_distance').value
        self.resume_distance = self.get_parameter('resume_distance').value
        self.leader_odom_topic = self.get_parameter('leader_odom_topic').value
        self.follower_action = self.get_parameter('follower_action').value
        self.litter_topic = self.get_parameter('litter_topic').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # --- State ---
        self.state = self.STATE_FOLLOWING
        self.breadcrumbs = []
        self.current_goal_index = 0
        self.leader_position = None
        self.leader_yaw = None
        self.leader_orientation = None
        self.last_breadcrumb_position = None
        self.follower_position = None
        self.is_navigating = False
        self.goal_handle = None
        self.parked_published = False
        self.lock = threading.Lock()

        # --- Callback group ---
        self.cb_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self.parked_pub = self.create_publisher(Bool, '/follower_parked', 10)

        # --- Subscriptions ---
        self.leader_odom_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.leader_odom_callback,
            10,
            callback_group=self.cb_group
        )

        self.follower_odom_sub = self.create_subscription(
            Odometry,
            '/follower/odom',
            self.follower_odom_callback,
            10,
            callback_group=self.cb_group
        )

        self.litter_sub = self.create_subscription(
            Bool,
            self.litter_topic,
            self.litter_callback,
            10,
            callback_group=self.cb_group
        )

        # --- Action client ---
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.follower_action,
            callback_group=self.cb_group
        )

        # --- Timer ---
        self.timer = self.create_timer(
            0.5, self.state_machine_tick, callback_group=self.cb_group
        )

        self.get_logger().info('=== Follower Behavior Node Started ===')
        self.get_logger().info(f'  Leader odom topic:    {self.leader_odom_topic}')
        self.get_logger().info(f'  Follower action:      {self.follower_action}')
        self.get_logger().info(f'  Litter topic:         {self.litter_topic}')
        self.get_logger().info(f'  Parked topic:         /follower_parked')
        self.get_logger().info(f'  Breadcrumb distance:  {self.breadcrumb_distance}m')
        self.get_logger().info(f'  Min follow distance:  {self.min_follow_distance}m')
        self.get_logger().info(f'  Park distance:        {self.park_distance}m behind leader')
        self.get_logger().info(f'  Resume distance:      {self.resume_distance}m')
        self.get_logger().info(f'State: {self.state}')
        self.get_logger().info('Waiting for leader to move...')

    # ================================================================
    # ODOM CALLBACKS
    # ================================================================

    def leader_odom_callback(self, msg: Odometry):
        """Track leader position and drop breadcrumbs."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        self.leader_position = (x, y)
        self.leader_yaw = yaw
        self.leader_orientation = msg.pose.pose.orientation

        # Only drop breadcrumbs in FOLLOWING state
        if self.state != self.STATE_FOLLOWING:
            return

        if self.last_breadcrumb_position is None:
            self.last_breadcrumb_position = (x, y)
            return

        dx = x - self.last_breadcrumb_position[0]
        dy = y - self.last_breadcrumb_position[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist >= self.breadcrumb_distance:
            breadcrumb = PoseStamped()
            breadcrumb.header.frame_id = 'map'
            breadcrumb.header.stamp = self.get_clock().now().to_msg()
            breadcrumb.pose.position.x = x
            breadcrumb.pose.position.y = y
            breadcrumb.pose.position.z = 0.0
            breadcrumb.pose.orientation = msg.pose.pose.orientation

            with self.lock:
                self.breadcrumbs.append(breadcrumb)
                count = len(self.breadcrumbs)

            self.last_breadcrumb_position = (x, y)
            self.get_logger().info(
                f'Breadcrumb #{count} dropped at ({x:.2f}, {y:.2f})'
            )

    def follower_odom_callback(self, msg: Odometry):
        """Track follower's current position."""
        self.follower_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # ================================================================
    # LITTER DETECTION
    # ================================================================

    def litter_callback(self, msg: Bool):
        """Handle litter detection signal from leader."""
        if msg.data and self.state == self.STATE_FOLLOWING:
            self.get_logger().info('!!! LITTER DETECTED — switching to PARKING mode !!!')

            with self.lock:
                # Cancel current navigation if any
                if self.is_navigating and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.is_navigating = False

                self.state = self.STATE_PARKING
                self.parked_published = False

    # ================================================================
    # STATE MACHINE
    # ================================================================

    def state_machine_tick(self):
        """Main state machine — runs every 0.5s."""
        if self.state == self.STATE_FOLLOWING:
            self.tick_following()
        elif self.state == self.STATE_PARKING:
            self.tick_parking()
        elif self.state == self.STATE_WAITING:
            self.tick_waiting()

    def tick_following(self):
        """FOLLOWING state: send next breadcrumb when ready."""
        with self.lock:
            if len(self.breadcrumbs) == 0:
                return
            if self.current_goal_index >= len(self.breadcrumbs):
                return
            if self.is_navigating:
                return

            # Check minimum follow distance — don't get too close to leader
            if self.leader_position is not None and self.follower_position is not None:
                dist_to_leader = math.sqrt(
                    (self.leader_position[0] - self.follower_position[0]) ** 2 +
                    (self.leader_position[1] - self.follower_position[1]) ** 2
                )
                if dist_to_leader < self.min_follow_distance:
                    return

        self.send_breadcrumb_goal()

    def tick_parking(self):
        """PARKING state: drive to position directly behind leader."""
        if self.leader_position is None or self.leader_yaw is None:
            return

        with self.lock:
            if self.is_navigating:
                return

        # Calculate park position: park_distance directly behind leader
        park_x = self.leader_position[0] - self.park_distance * math.cos(self.leader_yaw)
        park_y = self.leader_position[1] - self.park_distance * math.sin(self.leader_yaw)

        self.get_logger().info(
            f'Parking behind leader at ({park_x:.2f}, {park_y:.2f}), '
            f'leader at ({self.leader_position[0]:.2f}, {self.leader_position[1]:.2f})'
        )

        self.send_park_goal(park_x, park_y, self.leader_orientation)

    def tick_waiting(self):
        """WAITING state: wait until leader moves away, then resume following."""
        if self.leader_position is None or self.follower_position is None:
            self.get_logger().info(f'WAITING: leader={self.leader_position}, follower={self.follower_position}'
    )
            return

        dist_to_leader = math.sqrt(
            (self.leader_position[0] - self.follower_position[0]) ** 2 +
            (self.leader_position[1] - self.follower_position[1]) ** 2
        )
        self.get_logger().info(
        f'WAITING: leader=({self.leader_position[0]:.2f}, {self.leader_position[1]:.2f}), '
        f'follower=({self.follower_position[0]:.2f}, {self.follower_position[1]:.2f}), '
        f'dist={dist_to_leader:.2f}, resume_dist={self.resume_distance}'
    )

        if dist_to_leader > self.resume_distance:
            self.get_logger().info(
                f'Leader moved away ({dist_to_leader:.2f}m) — resuming FOLLOWING'
            )
            with self.lock:
                self.state = self.STATE_FOLLOWING
                self.last_breadcrumb_position = self.leader_position
                self.breadcrumbs.clear()
                self.current_goal_index = 0

                # Drop an immediate breadcrumb at leader's current position
                breadcrumb = PoseStamped()
                breadcrumb.header.frame_id = 'map'
                breadcrumb.header.stamp = Time()
                breadcrumb.pose.position.x = self.leader_position[0]
                breadcrumb.pose.position.y = self.leader_position[1]
                breadcrumb.pose.position.z = 0.0
                if self.leader_orientation is not None:
                    breadcrumb.pose.orientation = self.leader_orientation
                else:
                    breadcrumb.pose.orientation.w = 1.0
                self.breadcrumbs.append(breadcrumb)

            self.get_logger().info(
                f'Dropped immediate breadcrumb at leader position '
                f'({self.leader_position[0]:.2f}, {self.leader_position[1]:.2f})'
            )

    # ================================================================
    # GOAL SENDING
    # ================================================================

    def send_breadcrumb_goal(self):
        """Send the next breadcrumb as a NavigateToPose goal."""
        with self.lock:
            if self.current_goal_index >= len(self.breadcrumbs):
                return
            target = self.breadcrumbs[self.current_goal_index]
            self.is_navigating = True
            total = len(self.breadcrumbs)
            index = self.current_goal_index

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available!')
            with self.lock:
                self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = Time()
        goal_msg.pose.pose = target.pose

        self.get_logger().info(
            f'[FOLLOWING] Sending breadcrumb {index + 1}/{total}: '
            f'({target.pose.position.x:.2f}, {target.pose.position.y:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.breadcrumb_response_callback)

    def send_park_goal(self, x, y, orientation):
        """Send a park-behind goal."""
        with self.lock:
            self.is_navigating = True

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available!')
            with self.lock:
                self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = Time()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info(
            f'[PARKING] Sending park goal at ({x:.2f}, {y:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.park_response_callback)

    # ================================================================
    # BREADCRUMB GOAL CALLBACKS
    # ================================================================

    def breadcrumb_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Breadcrumb goal rejected')
            with self.lock:
                self.is_navigating = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.breadcrumb_result_callback)

    def breadcrumb_result_callback(self, future):
        result = future.result()
        status = result.status

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Breadcrumb {self.current_goal_index + 1} reached!'
                )
                self.current_goal_index += 1
                remaining = len(self.breadcrumbs) - self.current_goal_index
                if remaining <= 0:
                    self.get_logger().info('All breadcrumbs visited — waiting for more')
                else:
                    self.get_logger().info(f'{remaining} breadcrumbs remaining')

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    f'Breadcrumb {self.current_goal_index + 1} aborted, skipping...'
                )
                self.current_goal_index += 1

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Breadcrumb goal canceled')

    # ================================================================
    # PARK GOAL CALLBACKS
    # ================================================================

    def park_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Park goal rejected')
            with self.lock:
                self.is_navigating = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.park_result_callback)

    def park_result_callback(self, future):
        result = future.result()
        status = result.status

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    '=== PARKED behind leader — switching to WAITING ==='
                )
                self.state = self.STATE_WAITING

                # Tell the leader we're parked
                if not self.parked_published:
                    parked_msg = Bool()
                    parked_msg.data = True
                    self.parked_pub.publish(parked_msg)
                    self.parked_published = True
                    self.get_logger().info('Published /follower_parked for leader')

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn('Park goal aborted — retrying next tick')
                # Stay in PARKING state, will retry on next tick

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Park goal canceled')

    # ================================================================
    # FEEDBACK
    # ================================================================

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        if remaining > 0:
            self.get_logger().debug(f'Distance remaining: {remaining:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = FollowerBehaviorNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down follower behavior node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()