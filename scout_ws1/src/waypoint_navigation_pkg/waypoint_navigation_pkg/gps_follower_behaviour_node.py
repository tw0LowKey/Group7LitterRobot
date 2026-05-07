import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

from sapling_interfaces.msg import LeaderPose, FollowerStatus


class GPSFollowerBehaviorNode(Node):
    """
    Real-robot GPS follower behaviour node.

    Subscribes:
        /gps/fix
        /follower/imu
        /comms/leader_to_follower_rx

    Publishes:
        /comms/follower_to_leader_tx  (FollowerStatus — every 0.5s with full status)

    Sends goals to Nav2 action server:
        /follower/navigate_to_pose

    States:
        FOLLOWING         - follows leader breadcrumb trail
        PARKING_APPROACH  - Phase 1: drives to approach position behind leader
        PARKING_FINAL     - Phase 2: drives closer to final parking position
        WAITING           - parked behind leader, waiting for leader to move away
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

        self.declare_parameter("use_sim_time", False)

        # Behaviour parameters
        self.declare_parameter("breadcrumb_distance", 1.0)
        self.declare_parameter("min_follow_distance", 2.0)
        self.declare_parameter("approach_distance", 0.5)
        self.declare_parameter("park_distance", 0.2)
        self.declare_parameter("resume_distance", 2.5)
        self.declare_parameter("skip_park_distance", 0.9)

        # Robot geometry
        self.declare_parameter("leader_half_length", 0.325)
        self.declare_parameter("follower_half_length", 0.325)

        # Follower GPS antenna offset from robot centre
        self.declare_parameter("follower_gps_offset_x", -0.11)
        self.declare_parameter("follower_gps_offset_y", -0.16)

        # Real follower topics
        self.declare_parameter("follower_navsat_topic", "/gps/fix")
        self.declare_parameter("follower_imu_topic", "/follower/imu")
        self.declare_parameter("follower_action", "/follower/navigate_to_pose")

        self.declare_parameter("goal_frame", "follower/odom")

        # Communication topics
        self.declare_parameter("leader_pose_rx_topic", "/comms/leader_to_follower_rx")
        self.declare_parameter("follower_status_tx_topic", "/comms/follower_to_leader_tx")

        # GPS reference point
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

        self.follower_gps_offset_x = self.get_parameter("follower_gps_offset_x").value
        self.follower_gps_offset_y = self.get_parameter("follower_gps_offset_y").value

        self.follower_navsat_topic = self.get_parameter("follower_navsat_topic").value
        self.follower_imu_topic = self.get_parameter("follower_imu_topic").value
        self.follower_action = self.get_parameter("follower_action").value
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
        self.last_breadcrumb_position = None

        self.leader_position = None
        self.leader_yaw = None

        self.follower_position = None
        self.follower_yaw = None

        self.is_navigating = False
        self.goal_handle = None

        self.bin_is_ready = False

        self.ref_set = False
        self.follower_seq = 0
        self.last_leader_seq = 0

        # ================================================================
        # Publishers
        # ================================================================

        self.status_pub = self.create_publisher(
            FollowerStatus,
            self.follower_status_tx_topic,
            10
        )

        # ================================================================
        # Subscribers
        # ================================================================

        self.leader_sub = self.create_subscription(
            LeaderPose,
            self.leader_pose_rx_topic,
            self.leader_callback,
            10,
            callback_group=self.cb_group
        )

        self.follower_navsat_sub = self.create_subscription(
            NavSatFix,
            self.follower_navsat_topic,
            self.follower_navsat_callback,
            10,
            callback_group=self.cb_group
        )

        self.follower_imu_sub = self.create_subscription(
            Imu,
            self.follower_imu_topic,
            self.follower_imu_callback,
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

        self.state_timer = self.create_timer(
            0.5,
            self.state_machine_tick,
            callback_group=self.cb_group
        )

        self.status_timer = self.create_timer(
            0.5,
            self.publish_follower_status,
            callback_group=self.cb_group
        )

        self.print_startup_info()

    # ================================================================
    # Startup logging
    # ================================================================

    def print_startup_info(self):
        self.get_logger().info("=== Real GPS Follower Behaviour Node Started ===")
        self.get_logger().info(f"Follower GPS topic:       {self.follower_navsat_topic}")
        self.get_logger().info(f"Follower IMU topic:       {self.follower_imu_topic}")
        self.get_logger().info(f"Follower Nav2 action:     {self.follower_action}")
        self.get_logger().info(f"Goal frame:               {self.goal_frame}")
        self.get_logger().info(f"Leader pose RX topic:     {self.leader_pose_rx_topic}")
        self.get_logger().info(f"Follower status TX topic: {self.follower_status_tx_topic}")
        self.get_logger().info(f"Breadcrumb distance:      {self.breadcrumb_distance} m")
        self.get_logger().info(f"Minimum follow distance:  {self.min_follow_distance} m")
        self.get_logger().info(f"Approach distance:        {self.approach_distance} m")
        self.get_logger().info(f"Park distance:            {self.park_distance} m")
        self.get_logger().info(f"Skip park distance:       {self.skip_park_distance} m")
        self.get_logger().info(f"Resume distance:          {self.resume_distance} m")
        self.get_logger().info(f"Follower GPS offset:      x={self.follower_gps_offset_x}, y={self.follower_gps_offset_y}")

    # ================================================================
    # Maths helpers
    # ================================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def distance_2d(a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2
        )

    def gps_to_xy(self, lat, lon):
        x = (
            self.EARTH_RADIUS
            * math.radians(lon - self.ref_lon)
            * math.cos(math.radians(self.ref_lat))
        )
        y = self.EARTH_RADIUS * math.radians(lat - self.ref_lat)
        return x, y

    def correct_gps_offset_to_robot_centre(self, x, y, yaw, offset_x, offset_y):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        centre_x = x - (offset_x * cos_yaw - offset_y * sin_yaw)
        centre_y = y - (offset_x * sin_yaw + offset_y * cos_yaw)
        return centre_x, centre_y

    # ================================================================
    # Bin ready helper
    # ================================================================

    def set_bin_ready(self, ready):
        if ready != self.bin_is_ready:
            self.bin_is_ready = ready
            self.get_logger().info(f"Bin ready: {ready}")

    # ================================================================
    # Follower status publisher (consolidated — replaces separate bin_ready and parked)
    # ================================================================

    def publish_follower_status(self):
        """Publish full follower status every 0.5s."""
        self.follower_seq += 1

        msg = FollowerStatus()
        msg.seq = self.follower_seq
        msg.parked = (self.state == self.STATE_WAITING)
        msg.bin_ready = self.bin_is_ready

        if self.follower_position is not None:
            msg.park_x = self.follower_position[0]
            msg.park_y = self.follower_position[1]
        else:
            msg.park_x = 0.0
            msg.park_y = 0.0

        self.status_pub.publish(msg)

    # ================================================================
    # Leader communication callback
    # ================================================================

    def leader_callback(self, msg: LeaderPose):
        if msg.seq <= self.last_leader_seq:
            return
        self.last_leader_seq = msg.seq

        # Set reference point from first leader GPS fix
        if self.auto_ref and not self.ref_set:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_set = True
            self.get_logger().info(
                f"Reference point set from leader: "
                f"lat={self.ref_lat:.8f}, lon={self.ref_lon:.8f}"
            )

        if not self.ref_set:
            return

        leader_x, leader_y = self.gps_to_xy(msg.latitude, msg.longitude)
        self.leader_position = (leader_x, leader_y)

        self.leader_yaw = self.quaternion_to_yaw(
            0.0, 0.0, msg.orientation_z, msg.orientation_w
        )

        # Reset when litter is cleared (pickup complete)
        if not msg.call_bin:
            self.set_bin_ready(False)

        # Litter detected while following: cancel Nav2 goal and park
        if msg.call_bin and self.state == self.STATE_FOLLOWING:
            self.get_logger().info(
                "Litter detected from leader. Switching to parking mode."
            )
            with self.lock:
                if self.is_navigating and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.is_navigating = False
                self.state = self.STATE_PARKING_APPROACH
            self.set_bin_ready(False)
            return

        # Already waiting and close enough: skip parking, just set bin_ready
        if msg.call_bin and self.state == self.STATE_WAITING:
            if self.follower_position is not None and self.leader_position is not None:
                dist = self.distance_2d(self.leader_position, self.follower_position)
                if dist <= self.skip_park_distance:
                    self.get_logger().info(
                        f"Litter detected while already close enough "
                        f"({dist:.2f} m). Skipping parking."
                    )
                    self.set_bin_ready(True)
                    return

        # Only drop breadcrumbs in normal following mode
        if self.state != self.STATE_FOLLOWING:
            return

        if self.last_breadcrumb_position is None:
            self.last_breadcrumb_position = self.leader_position
            return

        dist = self.distance_2d(self.leader_position, self.last_breadcrumb_position)

        if dist >= self.breadcrumb_distance:
            with self.lock:
                self.breadcrumbs.append(self.leader_position)
                count = len(self.breadcrumbs)
            self.last_breadcrumb_position = self.leader_position
            self.get_logger().info(
                f"Breadcrumb #{count} dropped at "
                f"x={leader_x:.2f}, y={leader_y:.2f}"
            )

    # ================================================================
    # Local sensor callbacks
    # ================================================================

    def follower_navsat_callback(self, msg: NavSatFix):
        if not self.ref_set:
            return

        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().warn("Invalid GPS fix received.")
            return

        x, y = self.gps_to_xy(msg.latitude, msg.longitude)

        if self.follower_yaw is not None:
            x, y = self.correct_gps_offset_to_robot_centre(
                x, y,
                self.follower_yaw,
                self.follower_gps_offset_x,
                self.follower_gps_offset_y
            )

        self.follower_position = (x, y)

    def follower_imu_callback(self, msg: Imu):
        q = msg.orientation
        self.follower_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ================================================================
    # Trail distance
    # ================================================================

    def compute_trail_distance(self):
        total = 0.0
        for i in range(self.current_goal_index, len(self.breadcrumbs) - 1):
            total += self.distance_2d(
                self.breadcrumbs[i],
                self.breadcrumbs[i + 1]
            )
        if len(self.breadcrumbs) > 0 and self.leader_position is not None:
            total += self.distance_2d(
                self.breadcrumbs[-1],
                self.leader_position
            )
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
            if self.leader_position is not None and self.follower_position is not None:
                trail_dist = self.compute_trail_distance()
                if trail_dist < self.min_follow_distance:
                    return
        self.send_breadcrumb_goal()

    def tick_parking_approach(self):
        if self.leader_position is None or self.leader_yaw is None:
            return
        with self.lock:
            if self.is_navigating:
                return

        total_dist = (
            self.leader_half_length
            + self.approach_distance
            + self.follower_half_length
        )

        behind_angle = self.leader_yaw + math.pi
        approach_x = self.leader_position[0] + total_dist * math.cos(behind_angle)
        approach_y = self.leader_position[1] + total_dist * math.sin(behind_angle)

        angle_to_leader = math.atan2(
            self.leader_position[1] - approach_y,
            self.leader_position[0] - approach_x
        )
        orientation = self.yaw_to_quaternion(angle_to_leader)

        self.get_logger().info(
            f"[Parking Phase 1] Target: x={approach_x:.2f}, y={approach_y:.2f}"
        )
        self.send_park_goal(approach_x, approach_y, orientation)

    def tick_parking_final(self):
        if self.leader_position is None or self.leader_yaw is None:
            return
        with self.lock:
            if self.is_navigating:
                return

        total_dist = (
            self.leader_half_length
            + self.park_distance
            + self.follower_half_length
        )

        behind_angle = self.leader_yaw + math.pi
        park_x = self.leader_position[0] + total_dist * math.cos(behind_angle)
        park_y = self.leader_position[1] + total_dist * math.sin(behind_angle)

        angle_to_leader = math.atan2(
            self.leader_position[1] - park_y,
            self.leader_position[0] - park_x
        )
        orientation = self.yaw_to_quaternion(angle_to_leader)

        self.get_logger().info(
            f"[Parking Phase 2] Target: x={park_x:.2f}, y={park_y:.2f}"
        )
        self.send_park_goal(park_x, park_y, orientation)

    def tick_waiting(self):
        if self.leader_position is None or self.follower_position is None:
            return

        dist_to_leader = self.distance_2d(
            self.leader_position,
            self.follower_position
        )

        if dist_to_leader > self.resume_distance:
            self.get_logger().info(
                f"Leader moved away: {dist_to_leader:.2f} m. Resuming following."
            )
            self.set_bin_ready(False)
            with self.lock:
                self.state = self.STATE_FOLLOWING
                self.breadcrumbs.clear()
                self.current_goal_index = 0
                self.last_breadcrumb_position = self.leader_position
                self.breadcrumbs.append(self.leader_position)

    # ================================================================
    # Goal sending
    # ================================================================

    def send_breadcrumb_goal(self):
        with self.lock:
            if self.current_goal_index >= len(self.breadcrumbs):
                return
            target = self.breadcrumbs[self.current_goal_index]
            index = self.current_goal_index
            total = len(self.breadcrumbs)
            self.is_navigating = True

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
            return

        if self.follower_position is not None:
            dx = target[0] - self.follower_position[0]
            dy = target[1] - self.follower_position[1]
            heading = math.atan2(dy, dx)
        else:
            heading = 0.0

        orientation = self.yaw_to_quaternion(heading)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target[0])
        goal_msg.pose.pose.position.y = float(target[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info(
            f"[Following] Sending breadcrumb {index + 1}/{total}: "
            f"x={target[0]:.2f}, y={target[1]:.2f}"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
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
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = orientation

        self.get_logger().info(
            f"[Parking] Sending goal: x={x:.2f}, y={y:.2f}"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.park_response_callback)

    # ================================================================
    # Breadcrumb callbacks
    # ================================================================

    def breadcrumb_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().warn("Failed to get breadcrumb goal handle.")
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
    # Parking callbacks
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
        self.get_logger().info(
            f"[Debug] Park result status={status}, state={self.state}"
        )

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.state == self.STATE_PARKING_APPROACH:
                    self.get_logger().info(
                        "Phase 1 complete. Starting final parking approach."
                    )
                    self.state = self.STATE_PARKING_FINAL
                elif self.state == self.STATE_PARKING_FINAL:
                    self.get_logger().info("Follower parked. Switching to WAITING.")
                    self.state = self.STATE_WAITING
                    self.set_bin_ready(True)

            elif status == GoalStatus.STATUS_ABORTED:
                if self.state == self.STATE_PARKING_APPROACH:
                    self.get_logger().warn(
                        "Phase 1 aborted. Advancing to Phase 2."
                    )
                    self.state = self.STATE_PARKING_FINAL
                elif self.state == self.STATE_PARKING_FINAL:
                    self.get_logger().warn(
                        "Phase 2 aborted. Treating current position as parked."
                    )
                    self.state = self.STATE_WAITING
                    self.set_bin_ready(True)
                else:
                    self.get_logger().warn("Parking goal aborted.")

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Parking goal cancelled.")

    # ================================================================
    # Feedback callback
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
