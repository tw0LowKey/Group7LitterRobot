import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time


class SLAMFollowerBehaviorNode(Node):
    """
    SLAM-based follower behavior node with litter pickup mode.

    Both leader and follower run SLAM/localisation independently and publish
    their poses (PoseStamped) in a shared `map` frame over WiFi via ROS 2 DDS.

    States:
        FOLLOWING    - following leader's breadcrumb trail
        PARKING      - driving to park_distance directly behind leader
        WAITING      - parked behind leader, waiting for leader to move away
    """

    STATE_FOLLOWING = 'FOLLOWING'
    STATE_PARKING = 'PARKING'
    STATE_WAITING = 'WAITING'

    def __init__(self):
        super().__init__('slam_follower_behavior_node')

        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('breadcrumb_distance', 0.5)
        self.declare_parameter('min_follow_distance', 1.0)
        self.declare_parameter('park_distance', 0.5)
        self.declare_parameter('resume_distance', 1.5)
        self.declare_parameter('leader_pose_topic', '/leader/pose')
        self.declare_parameter('follower_pose_topic', '/follower/pose')
        self.declare_parameter('follower_action', '/follower/navigate_to_pose')
        self.declare_parameter('litter_topic', '/litter_detected')
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('goal_frame', 'map')

        self.breadcrumb_distance = self.get_parameter('breadcrumb_distance').value
        self.min_follow_distance = self.get_parameter('min_follow_distance').value
        self.park_distance = self.get_parameter('park_distance').value
        self.resume_distance = self.get_parameter('resume_distance').value
        self.leader_pose_topic = self.get_parameter('leader_pose_topic').value
        self.follower_pose_topic = self.get_parameter('follower_pose_topic').value
        self.follower_action = self.get_parameter('follower_action').value
        self.litter_topic = self.get_parameter('litter_topic').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_frame = self.get_parameter('goal_frame').value

        # --- State ---
        self.state = self.STATE_FOLLOWING
        self.breadcrumbs = []
        self.current_goal_index = 0
        self.leader_position = None
        self.leader_yaw = None
        self.follower_position = None
        self.follower_yaw = None
        self.last_breadcrumb_position = None
        self.is_navigating = False
        self.goal_handle = None
        self.parked_published = False
        self.lock = threading.Lock()

        # --- Action client ---
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.follower_action,
            callback_group=self.cb_group
        )

        # --- Publishers (outward communication) ---
        self.parked_pub = self.create_publisher(Bool, '/follower_parked', 10)
        self.state_pub = self.create_publisher(String, '/follower_state', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/follower_position', 10)

        # --- Services ---
        self.reset_service = self.create_service(
            Trigger,
            '/follower/reset_breadcrumbs',
            self.reset_breadcrumbs_callback,
            callback_group=self.cb_group
        )

        # --- Subscriptions ---
        self.leader_pose_sub = self.create_subscription(
            PoseStamped,
            self.leader_pose_topic,
            self.leader_pose_callback,
            10,
            callback_group=self.cb_group
        )

        self.follower_pose_sub = self.create_subscription(
            PoseStamped,
            self.follower_pose_topic,
            self.follower_pose_callback,
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

        # --- Timers ---
        self.state_timer = self.create_timer(
            0.5,
            self.state_machine_tick,
            callback_group=self.cb_group
        )

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status,
            callback_group=self.cb_group
        )

        self.get_logger().info('=== SLAM Follower Behavior Node Started ===')
        self.get_logger().info(f'  Leader pose topic:    {self.leader_pose_topic}')
        self.get_logger().info(f'  Follower pose topic:  {self.follower_pose_topic}')
        self.get_logger().info(f'  Follower action:      {self.follower_action}')
        self.get_logger().info(f'  Goal frame:           {self.goal_frame}')
        self.get_logger().info(f'  Breadcrumb dist:      {self.breadcrumb_distance}m')
        self.get_logger().info(f'  Min follow dist:      {self.min_follow_distance}m')
        self.get_logger().info(f'  Park distance:        {self.park_distance}m')
        self.get_logger().info(f'  Resume distance:      {self.resume_distance}m')
        self.get_logger().info(f'State: {self.state}')
        self.get_logger().info('Waiting for leader pose and movement...')

    # ================================================================
    # MATH HELPERS
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ================================================================
    # POSE CALLBACKS
    # ================================================================

    def leader_pose_callback(self, msg):
        """Track leader pose and drop breadcrumbs as it moves."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation

        self.leader_position = (x, y)
        self.leader_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

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
                self.breadcrumbs.append((x, y))
                count = len(self.breadcrumbs)

            self.last_breadcrumb_position = (x, y)
            self.get_logger().info(
                f'Breadcrumb #{count} dropped at XY ({x:.2f}, {y:.2f})'
            )

    def follower_pose_callback(self, msg):
        """Track follower pose."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation

        self.follower_position = (x, y)
        self.follower_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ================================================================
    # LITTER DETECTION
    # ================================================================

    def litter_callback(self, msg):
        """Handle litter detection signal from leader."""
        if msg.data and self.state == self.STATE_FOLLOWING:
            self.get_logger().info('!!! LITTER DETECTED - switching to PARKING mode !!!')

            with self.lock:
                if self.is_navigating and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.is_navigating = False

                self.state = self.STATE_PARKING
                self.parked_published = False

    # ================================================================
    # SERVICES
    # ================================================================

    def reset_breadcrumbs_callback(self, request, response):
        """Reset breadcrumb trail and return to FOLLOWING state."""
        with self.lock:
            if self.is_navigating and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.is_navigating = False

            self.breadcrumbs.clear()
            self.current_goal_index = 0
            self.last_breadcrumb_position = None
            self.state = self.STATE_FOLLOWING
            self.parked_published = False

        self.get_logger().info('Breadcrumbs reset - returned to FOLLOWING state.')
        response.success = True
        response.message = 'Breadcrumbs cleared and state reset to FOLLOWING.'
        return response

    # ================================================================
    # STATE MACHINE
    # ================================================================

    def state_machine_tick(self):
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

        park_x = self.leader_position[0] - self.park_distance * math.cos(self.leader_yaw)
        park_y = self.leader_position[1] - self.park_distance * math.sin(self.leader_yaw)

        angle_to_leader = math.atan2(
            self.leader_position[1] - park_y,
            self.leader_position[0] - park_x
        )
        q = Quaternion()
        q.z = math.sin(angle_to_leader / 2.0)
        q.w = math.cos(angle_to_leader / 2.0)

        self.get_logger().info(
            f'Parking behind leader at XY ({park_x:.2f}, {park_y:.2f}), '
            f'leader at ({self.leader_position[0]:.2f}, {self.leader_position[1]:.2f}), '
            f'leader_yaw={self.leader_yaw:.2f}'
        )

        self.send_park_goal(park_x, park_y, q)

    def tick_waiting(self):
        """WAITING state: wait until leader moves away, then resume following."""
        if self.leader_position is None or self.follower_position is None:
            self.get_logger().info(
                f'WAITING: leader={self.leader_position}, follower={self.follower_position}'
            )
            return

        dist_to_leader = math.sqrt(
            (self.leader_position[0] - self.follower_position[0]) ** 2 +
            (self.leader_position[1] - self.follower_position[1]) ** 2
        )

        self.get_logger().info(
            f'WAITING: dist={dist_to_leader:.2f}, resume_dist={self.resume_distance}'
        )

        if dist_to_leader > self.resume_distance:
            self.get_logger().info(
                f'Leader moved away ({dist_to_leader:.2f}m) - resuming FOLLOWING'
            )
            with self.lock:
                self.state = self.STATE_FOLLOWING
                self.last_breadcrumb_position = self.leader_position
                self.breadcrumbs.clear()
                self.current_goal_index = 0
                self.breadcrumbs.append(self.leader_position)

            self.get_logger().info(
                f'Dropped immediate breadcrumb at leader XY '
                f'({self.leader_position[0]:.2f}, {self.leader_position[1]:.2f})'
            )

    # ================================================================
    # OUTWARD STATUS PUBLISHING
    # ================================================================

    def publish_status(self):
        """Publish current state and follower position for external nodes."""
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        if self.follower_position is not None:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.goal_frame
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = float(self.follower_position[0])
            pose_msg.pose.position.y = float(self.follower_position[1])
            pose_msg.pose.position.z = 0.0

            if self.follower_yaw is not None:
                pose_msg.pose.orientation.z = math.sin(self.follower_yaw / 2.0)
                pose_msg.pose.orientation.w = math.cos(self.follower_yaw / 2.0)
            else:
                pose_msg.pose.orientation.w = 1.0

            self.position_pub.publish(pose_msg)

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
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = Time()
        goal_msg.pose.pose.position.x = float(target[0])
        goal_msg.pose.pose.position.y = float(target[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'[FOLLOWING] Sending breadcrumb {index + 1}/{total}: '
            f'XY ({target[0]:.2f}, {target[1]:.2f})'
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
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = Time()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info(
            f'[PARKING] Sending park goal at XY ({x:.2f}, {y:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.park_response_callback)

    # ================================================================
    # GOAL CALLBACKS
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
                    self.get_logger().info('All breadcrumbs visited - waiting for more')
                else:
                    self.get_logger().info(f'{remaining} breadcrumbs remaining')

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    f'Breadcrumb {self.current_goal_index + 1} aborted, skipping...'
                )
                self.current_goal_index += 1

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Breadcrumb goal canceled')

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
                    '=== PARKED behind leader - switching to WAITING ==='
                )
                self.state = self.STATE_WAITING

                if not self.parked_published:
                    parked_msg = Bool()
                    parked_msg.data = True
                    self.parked_pub.publish(parked_msg)
                    self.parked_published = True
                    self.get_logger().info('Published /follower_parked for leader')

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn('Park goal aborted - retrying next tick')

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Park goal canceled')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        if remaining > 0:
            self.get_logger().debug(f'Distance remaining: {remaining:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = SLAMFollowerBehaviorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    
# {
#     
#     "lat": 53.4808,
#     "lon": -2.2426,
#     "orientation_x_y": (x,y),       
#     "litter": False,       # handshake flag
#             
# }



# {
#     
#     "parked": True,
#     "seq": 56
# }