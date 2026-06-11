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

class LeaderBehaviorNode(Node):
    """
    Leader behavior node for lawnmower coverage pattern.

    States:
        NAVIGATING       — driving to next waypoint
        WAITING_FOLLOWER — litter detected, waiting for follower to park
        PICKUP           — follower parked, simulating arm pickup
        IDLE             — all waypoints completed
    """

    STATE_NAVIGATING = 'NAVIGATING'
    STATE_WAITING_FOLLOWER = 'WAITING_FOLLOWER'
    STATE_PICKUP = 'PICKUP'
    STATE_IDLE = 'IDLE'

    def __init__(self):
        super().__init__('leader_behavior_node')

        # --- Parameters ---
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('leader_action', '/navigate_to_pose')
        self.declare_parameter('litter_topic', '/litter_detected')
        self.declare_parameter('leader_odom_topic', '/odom')
        self.declare_parameter('pickup_wait_time', 2.0)
        self.declare_parameter('goal_frame', 'map')

        waypoints_flat = self.get_parameter('waypoints').value
        self.leader_action = self.get_parameter('leader_action').value
        self.litter_topic = self.get_parameter('litter_topic').value
        self.leader_odom_topic = self.get_parameter('leader_odom_topic').value
        self.pickup_wait_time = self.get_parameter('pickup_wait_time').value
        self.goal_frame = self.get_parameter('goal_frame').value

        # Parse flat list into (x, y) pairs
        self.waypoints = []
        for i in range(0, len(waypoints_flat) - 1, 2):
            self.waypoints.append((waypoints_flat[i], waypoints_flat[i + 1]))

        # --- State ---
        self.state = self.STATE_IDLE
        self.current_waypoint_index = 0
        self.leader_position = None
        self.leader_yaw = None
        self.is_navigating = False
        self.goal_handle = None
        self.pickup_timer = None
        self.lock = threading.Lock()

        # --- Callback group ---
        self.cb_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self.litter_pub = self.create_publisher(Bool, self.litter_topic, 10)

        # --- Subscriptions ---
        self.litter_trigger_sub = self.create_subscription(
            Bool,
            '/litter_trigger',
            self.litter_trigger_callback,
            10,
            callback_group=self.cb_group
        )

        self.follower_parked_sub = self.create_subscription(
            Bool,
            '/follower_parked',
            self.follower_parked_callback,
            10,
            callback_group=self.cb_group
        )

        self.leader_odom_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.leader_odom_callback,
            10,
            callback_group=self.cb_group
        )

        # --- Action client ---
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.leader_action,
            callback_group=self.cb_group
        )

        # --- Timer ---
        self.tick_timer = self.create_timer(
            0.5, self.state_machine_tick, callback_group=self.cb_group
        )

        self.get_logger().info('=== Leader Behavior Node Started ===')
        self.get_logger().info(f'  Leader action:      {self.leader_action}')
        self.get_logger().info(f'  Litter topic:       {self.litter_topic}')
        self.get_logger().info(f'  Litter trigger:     /litter_trigger')
        self.get_logger().info(f'  Follower parked:    /follower_parked')
        self.get_logger().info(f'  Pickup wait time:   {self.pickup_wait_time}s')
        self.get_logger().info(f'  Goal frame:         {self.goal_frame}')
        self.get_logger().info(f'  Waypoints loaded:   {len(self.waypoints)}')

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'    WP{i + 1}: ({wp[0]:.2f}, {wp[1]:.2f})')

        # Start navigating if we have waypoints
        if len(self.waypoints) > 0:
            self.state = self.STATE_NAVIGATING
            self.get_logger().info('Starting lawnmower pattern...')
        else:
            self.get_logger().warn('No waypoints loaded! Add waypoints to config.')

    # ================================================================
    # CALLBACKS
    # ================================================================

    def leader_odom_callback(self, msg: Odometry):
        """Track leader's current position."""
        self.leader_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.leader_yaw = 2.0 * math.atan2(qz, qw)

    def litter_trigger_callback(self, msg: Bool):
        """User triggers litter detection."""
        if msg.data and self.state == self.STATE_NAVIGATING:
            self.get_logger().info('!!! LITTER TRIGGER received — stopping leader !!!')

            with self.lock:
                # Cancel current Nav2 goal
                if self.is_navigating and self.goal_handle is not None:
                    self.get_logger().info('Canceling current Nav2 goal...')
                    self.goal_handle.cancel_goal_async()
                    self.is_navigating = False

                self.state = self.STATE_WAITING_FOLLOWER

            # Publish litter_detected for the follower
            litter_msg = Bool()
            litter_msg.data = True
            self.litter_pub.publish(litter_msg)
            self.get_logger().info('Published /litter_detected — waiting for follower to park...')

    def follower_parked_callback(self, msg: Bool):
        """Follower signals it has parked behind the leader."""
        if msg.data and self.state == self.STATE_WAITING_FOLLOWER:
            self.get_logger().info('!!! Follower PARKED — starting pickup !!!')

            with self.lock:
                self.state = self.STATE_PICKUP

            # Start pickup timer
            self.pickup_timer = self.create_timer(
                self.pickup_wait_time,
                self.pickup_complete_callback,
                callback_group=self.cb_group
            )

    # ================================================================
    # STATE MACHINE
    # ================================================================

    def state_machine_tick(self):
        """Main state machine — runs every 0.5s."""
        if self.state == self.STATE_NAVIGATING:
            self.tick_navigating()
        elif self.state == self.STATE_WAITING_FOLLOWER:
            self.tick_waiting_follower()
        elif self.state == self.STATE_PICKUP:
            pass  # Waiting for pickup timer
        elif self.state == self.STATE_IDLE:
            pass

    def tick_navigating(self):
        """NAVIGATING: send next waypoint if not already navigating."""
        with self.lock:
            if self.current_waypoint_index >= len(self.waypoints):
                self.state = self.STATE_IDLE
                self.get_logger().info('=== All waypoints completed! ===')
                return

            if self.is_navigating:
                return

        self.send_next_waypoint()

    def tick_waiting_follower(self):
        """WAITING_FOLLOWER: just log periodically while waiting."""
        self.get_logger().info(
            'Waiting for follower to park... (listening on /follower_parked)'
        )

    def pickup_complete_callback(self):
        """Called when pickup wait time is done."""
        # Cancel the one-shot timer
        if self.pickup_timer is not None:
            self.pickup_timer.cancel()
            self.destroy_timer(self.pickup_timer)
            self.pickup_timer = None

        self.get_logger().info('Pickup complete — resuming navigation')

        with self.lock:
            self.state = self.STATE_NAVIGATING
            # Don't increment waypoint index — retry the same waypoint
            # since we were interrupted mid-way

    # ================================================================
    # GOAL SENDING
    # ================================================================

    def send_next_waypoint(self):
        """Send the next lawnmower waypoint."""
        with self.lock:
            if self.current_waypoint_index >= len(self.waypoints):
                return

            wp = self.waypoints[self.current_waypoint_index]
            self.is_navigating = True
            index = self.current_waypoint_index
            total = len(self.waypoints)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateToPose action server not available!')
            with self.lock:
                self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = Time()
        goal_msg.pose.pose.position.x = wp[0]
        goal_msg.pose.pose.position.y = wp[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'[NAVIGATING] Sending waypoint {index + 1}/{total}: '
            f'({wp[0]:.2f}, {wp[1]:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # ================================================================
    # GOAL CALLBACKS
    # ================================================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint goal rejected')
            with self.lock:
                self.is_navigating = False
            return

        self.get_logger().info('Waypoint goal accepted')
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_index + 1} reached!'
                )
                self.current_waypoint_index += 1

                remaining = len(self.waypoints) - self.current_waypoint_index
                if remaining <= 0:
                    self.state = self.STATE_IDLE
                    self.get_logger().info('=== Lawnmower pattern complete! ===')
                else:
                    self.get_logger().info(f'{remaining} waypoints remaining')

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    f'Waypoint {self.current_waypoint_index + 1} aborted, skipping...'
                )
                self.current_waypoint_index += 1

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Waypoint goal canceled (litter detected?)')
                # Don't increment — will retry same waypoint after pickup

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LeaderBehaviorNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down leader behavior node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()