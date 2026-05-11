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
from sapling_interfaces.srv import AreaCoords


class GPSLeaderBehaviorNode(Node):
    """
    Real-robot GPS leader behaviour node for lawnmower coverage pattern.

    Subscribes:
        /gps/fix
        /imu
        /call_bin
        /comms/follower_to_leader_rx

    Publishes:
        /comms/leader_to_follower_tx

    Sends goals to Nav2 action server:
        /navigate_to_pose

    Calls service:
        /comms/area_coords  (provided by executor_node — reads stored coords)

    States:
        WAITING_MISSION  — polling executor for area coordinates
        NAVIGATING       — driving to next waypoint
        WAITING_FOLLOWER — litter detected, waiting for follower to park
        PICKUP           — follower parked, waiting for pickup to complete
        IDLE             — all waypoints completed
    """

    STATE_NAVIGATING = "NAVIGATING"
    STATE_WAITING_FOLLOWER = "WAITING_FOLLOWER"
    STATE_PICKUP = "PICKUP"
    STATE_IDLE = "IDLE"
    STATE_WAITING_MISSION = "WAITING_MISSION"

    EARTH_RADIUS = 6371000.0

    def __init__(self):
        super().__init__("gps_leader_behavior_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ================================================================
        # Parameters
        # ================================================================

        #self.declare_parameter("use_sim_time", False)

        self.declare_parameter("leader_action", "/navigate_to_pose")
        self.declare_parameter("leader_navsat_topic", "/gps/fix")
        self.declare_parameter("leader_imu_topic", "/imu")
        self.declare_parameter("pickup_wait_time", 2.0)
        self.declare_parameter("goal_frame", "odom")
        self.declare_parameter("strip_width", 1.35)

        # Leader GPS antenna offset from robot centre
        self.declare_parameter("leader_gps_offset_x", -0.165)
        self.declare_parameter("leader_gps_offset_y", -0.145)

        # Communication topics
        self.declare_parameter("leader_pose_tx_topic", "/comms/leader_to_follower_tx")
        self.declare_parameter("follower_status_rx_topic", "/comms/follower_to_leader_rx")

        # Area coords service (provided by executor_node)
        self.declare_parameter("area_coords_service", "/comms/area_coords")
        self.declare_parameter("area_poll_period", 2.0)

        # GPS reference point
        self.declare_parameter("ref_lat", 0.0)
        self.declare_parameter("ref_lon", 0.0)
        self.declare_parameter("auto_ref", True)

        # Read parameters
        self.leader_action = self.get_parameter("leader_action").value
        self.leader_navsat_topic = self.get_parameter("leader_navsat_topic").value
        self.leader_imu_topic = self.get_parameter("leader_imu_topic").value
        self.pickup_wait_time = self.get_parameter("pickup_wait_time").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.strip_width = self.get_parameter("strip_width").value

        self.leader_gps_offset_x = self.get_parameter("leader_gps_offset_x").value
        self.leader_gps_offset_y = self.get_parameter("leader_gps_offset_y").value

        self.leader_pose_tx_topic = self.get_parameter("leader_pose_tx_topic").value
        self.follower_status_rx_topic = self.get_parameter("follower_status_rx_topic").value

        self.area_coords_service_name = self.get_parameter("area_coords_service").value
        self.area_poll_period = self.get_parameter("area_poll_period").value

        self.ref_lat = self.get_parameter("ref_lat").value
        self.ref_lon = self.get_parameter("ref_lon").value
        self.auto_ref = self.get_parameter("auto_ref").value

        # ================================================================
        # Internal state
        # ================================================================

        self.waypoints_gps = []

        self.state = self.STATE_WAITING_MISSION
        self.current_waypoint_index = 0
        self.leader_position = None
        self.leader_lat = None
        self.leader_lon = None
        self.leader_yaw = None
        self.leader_qz = 0.0
        self.leader_qw = 1.0
        self.is_navigating = False
        self.goal_handle = None
        self.pickup_timer = None
        self.ref_set = False
        self.call_bin_flag = False
        self.leader_seq = 0
        self.follower_park_position = None
        self.follower_bin_ready = False
        self.area_request_pending = False
        self.mission_loaded = False

        # ================================================================
        # Publishers
        # ================================================================

        self.leader_pose_pub = self.create_publisher(
            LeaderPose,
            self.leader_pose_tx_topic,
            10
        )

        # ================================================================
        # Subscribers
        # ================================================================

        self.follower_status_sub = self.create_subscription(
            FollowerStatus,
            self.follower_status_rx_topic,
            self.follower_status_callback,
            10,
            callback_group=self.cb_group
        )

        self.call_bin_sub = self.create_subscription(
            Bool,
            "/call_bin",
            self.call_bin_callback,
            10,
            callback_group=self.cb_group
        )

        self.navsat_sub = self.create_subscription(
            NavSatFix,
            self.leader_navsat_topic,
            self.navsat_callback,
            10,
            callback_group=self.cb_group
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.leader_imu_topic,
            self.imu_callback,
            10,
            callback_group=self.cb_group
        )

        # ================================================================
        # Nav2 action client
        # ================================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.leader_action,
            callback_group=self.cb_group
        )

        # ================================================================
        # Area Coordinate Service Client
        # ================================================================

        self.area_client = self.create_client(
            AreaCoords,
            self.area_coords_service_name,
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

        self.leader_pose_timer = self.create_timer(
            0.5,
            self.publish_leader_pose,
            callback_group=self.cb_group
        )

        self.area_poll_timer = self.create_timer(
            self.area_poll_period,
            self.poll_area_coords,
            callback_group=self.cb_group
        )

        self.print_startup_info()

    # ================================================================
    # Startup logging
    # ================================================================

    def print_startup_info(self):
        self.get_logger().info("=== Real GPS Leader Behaviour Node Started ===")
        self.get_logger().info(f"Leader Nav2 action:       {self.leader_action}")
        self.get_logger().info(f"Leader GPS topic:         {self.leader_navsat_topic}")
        self.get_logger().info(f"Leader IMU topic:         {self.leader_imu_topic}")
        self.get_logger().info(f"Goal frame:               {self.goal_frame}")
        self.get_logger().info(f"Leader pose TX topic:     {self.leader_pose_tx_topic}")
        self.get_logger().info(f"Follower status RX topic: {self.follower_status_rx_topic}")
        self.get_logger().info(f"Area coords service:      {self.area_coords_service_name}")
        self.get_logger().info(f"Area poll period:         {self.area_poll_period}s")
        self.get_logger().info(f"Pickup wait time:         {self.pickup_wait_time}s")
        self.get_logger().info(f"Strip width:              {self.strip_width}m")
        self.get_logger().info(f"Leader GPS offset:        x={self.leader_gps_offset_x}, y={self.leader_gps_offset_y}")
        self.get_logger().info(f"State:                    {self.state}")
        self.get_logger().info("Polling executor for area coordinates...")

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

    def correct_gps_offset_to_robot_centre(self, x, y, yaw, offset_x, offset_y):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        centre_x = x - (offset_x * cos_yaw - offset_y * sin_yaw)
        centre_y = y - (offset_x * sin_yaw + offset_y * cos_yaw)
        return centre_x, centre_y

    # ================================================================
    # Leader pose publisher
    # ================================================================

    def publish_leader_pose(self):
        """Publish leader state to follower every 0.5s."""
        if self.leader_lat is None or self.leader_lon is None:
            return

        self.leader_seq+= 1

        msg = LeaderPose()
        msg.seq = self.leader_seq
    
        msg.latitude = self.leader_lat
        msg.longitude = self.leader_lon
        msg.orientation_z = self.leader_qz
        msg.orientation_w = self.leader_qw
        msg.call_bin = self.call_bin_flag

        self.leader_pose_pub.publish(msg)

    # ================================================================
    # Follower status callback
    # ================================================================

    def follower_status_callback(self, msg: FollowerStatus):
        """Handle incoming status from follower (continuous, every 0.5s)."""
        # Store follower's bin_ready state
        self.follower_bin_ready = msg.bin_ready

        # Store follower position
        if msg.park_x != 0.0 or msg.park_y != 0.0:
            self.follower_park_position = (msg.park_x, msg.park_y)

        # Trigger pickup when follower reports parked while we're waiting
        if msg.parked and self.state == self.STATE_WAITING_FOLLOWER:
            self.follower_park_position = (msg.park_x, msg.park_y)
            self.get_logger().info(
                f"Follower PARKED at XY "
                f"({msg.park_x:.2f}, {msg.park_y:.2f}) — starting pickup."
            )

            with self.lock:
                self.state = self.STATE_PICKUP

            self.pickup_timer = self.create_timer(
                self.pickup_wait_time,
                self.pickup_complete_callback,
                callback_group=self.cb_group
            )

    # ================================================================
    # Lawnmower pattern generation
    # ================================================================

    def generate_lawnmower_gps(self, top_left_lat, top_left_lon,
                                bottom_right_lat, bottom_right_lon):
        north_lat = top_left_lat
        south_lat = bottom_right_lat
        west_lon = top_left_lon
        east_lon = bottom_right_lon

        row_spacing_deg = self.strip_width / 111320.0

        mid_lat = (north_lat + south_lat) / 2.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(mid_lat))
        point_spacing_deg = 1.0 / meters_per_deg_lon

        turn_buffer_deg = 0.001 / meters_per_deg_lon
        row_west_lon = west_lon + turn_buffer_deg
        row_east_lon = east_lon - turn_buffer_deg

        if row_west_lon >= row_east_lon:
            self.get_logger().error("Area too narrow after applying 0.5m turn buffer.")
            return []

        waypoints = []
        lat = north_lat
        row_num = 0

        while lat >= south_lat:
            if row_num % 2 == 0:
                lon = row_west_lon
                while lon <= row_east_lon:
                    waypoints.append((lat, lon))
                    lon += point_spacing_deg
                if waypoints[-1][1] < row_east_lon:
                    waypoints.append((lat, row_east_lon))
            else:
                lon = row_east_lon
                while lon >= row_west_lon:
                    waypoints.append((lat, lon))
                    lon -= point_spacing_deg
                if waypoints[-1][1] > row_west_lon:
                    waypoints.append((lat, row_west_lon))

            lat -= row_spacing_deg
            row_num += 1

        return waypoints

    # ================================================================
    # Area Coordinate Service Client (polls executor_node)
    # ================================================================

    def poll_area_coords(self):
        """Periodically ask the executor for stored area coords until we get them."""
        if self.mission_loaded:
            return
        if self.area_request_pending:
            return

        if not self.area_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                f"Area coords service '{self.area_coords_service_name}' not available yet."
            )
            return

        request = AreaCoords.Request()
        self.area_request_pending = True
        future = self.area_client.call_async(request)
        future.add_done_callback(self.area_coords_response)

    def area_coords_response(self, future):
        """Handle response from executor's AreaCoords service."""
        self.area_request_pending = False

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Area coords service call failed: {e}")
            return

        if response is None:
            self.get_logger().warn("Area coords service returned None.")
            return

        if not response.success:
            # Coords not yet set on executor — quietly try again next poll
            return

        top_left_lat = response.top_left_latitude
        top_left_lon = response.top_left_longitude
        bottom_right_lat = response.bottom_right_latitude
        bottom_right_lon = response.bottom_right_longitude

        self.get_logger().info("=== Area Coordinates Received from Executor ===")
        self.get_logger().info(
            f"Top-left:     ({top_left_lat:.8f}, {top_left_lon:.8f})"
        )
        self.get_logger().info(
            f"Bottom-right: ({bottom_right_lat:.8f}, {bottom_right_lon:.8f})"
        )

        # Validate orientation: top-left must be north-west of bottom-right
        #if (top_left_lat <= bottom_right_lat or
        #        top_left_lon >= bottom_right_lon):
        #    self.get_logger().error(
        #        "Invalid area: top-left must be north-west of bottom-right. "
        #        "Ignoring — will keep polling."
        #    )
        #    return

        waypoints = self.generate_lawnmower_gps(
            top_left_lat, top_left_lon,
            bottom_right_lat, bottom_right_lon
        )

        if len(waypoints) == 0:
            self.get_logger().error("No waypoints generated — area too small?")
            return

        with self.lock:
            self.waypoints_gps = waypoints
            self.current_waypoint_index = 0
            self.is_navigating = False
            self.mission_loaded = True

            if self.ref_set:
                self.state = self.STATE_NAVIGATING
                self.get_logger().info("GPS fix available — starting immediately!")
            else:
                self.state = self.STATE_IDLE
                self.get_logger().info(
                    "Waypoints loaded — waiting for GPS fix to start..."
                )

        self.get_logger().info(f"Generated {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f"  WP{i + 1}: ({wp[0]:.8f}, {wp[1]:.8f})")

        # Stop polling once mission is loaded
        if self.area_poll_timer is not None:
            self.area_poll_timer.cancel()

    # ================================================================
    # Local sensor callbacks
    # ================================================================

    def navsat_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().warn("Invalid GPS fix received.")
            return

        if self.auto_ref and not self.ref_set:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_set = True
            self.get_logger().info(
                f"Reference point set: "
                f"lat={self.ref_lat:.8f}, lon={self.ref_lon:.8f}"
            )
            if len(self.waypoints_gps) > 0 and self.state == self.STATE_IDLE:
                self.state = self.STATE_NAVIGATING
                self.get_logger().info("GPS fix received — starting lawnmower pattern!")

        self.leader_lat = msg.latitude
        self.leader_lon = msg.longitude

        if self.ref_set:
            x, y = self.gps_to_xy(msg.latitude, msg.longitude)

            if self.leader_yaw is not None:
                x, y = self.correct_gps_offset_to_robot_centre(
                    x, y,
                    self.leader_yaw,
                    self.leader_gps_offset_x,
                    self.leader_gps_offset_y
                )

            self.leader_position = (x, y)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.leader_qz = q.z
        self.leader_qw = q.w
        self.leader_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def call_bin_callback(self, msg: Bool):
        """Litter detected — stop leader and signal follower."""
        if msg.data and self.state == self.STATE_NAVIGATING:
            self.get_logger().info("Call bin received — stopping leader.")

            with self.lock:
                if self.is_navigating and self.goal_handle is not None:
                    self.get_logger().info("Cancelling current Nav2 goal...")
                    self.goal_handle.cancel_goal_async()
                    self.is_navigating = False

                self.state = self.STATE_WAITING_FOLLOWER

            self.call_bin_flag = True
            self.get_logger().info("Litter flag set — waiting for follower to park...")

    # ================================================================
    # State machine
    # ================================================================

    def state_machine_tick(self):
        if self.state == self.STATE_NAVIGATING:
            self.tick_navigating()
        elif self.state == self.STATE_WAITING_FOLLOWER:
            pass
        elif self.state == self.STATE_PICKUP:
            pass
        elif self.state == self.STATE_IDLE:
            pass
        elif self.state == self.STATE_WAITING_MISSION:
            pass

    def tick_navigating(self):
        with self.lock:
            if self.current_waypoint_index >= len(self.waypoints_gps):
                self.state = self.STATE_WAITING_FOLLOWER
                self.call_bin_flag = True
                self.get_logger().info(
                    "All GPS waypoints completed. "
                    "Waiting for follower to park for final collection."
                )
                return
            if self.is_navigating:
                return
        self.send_next_waypoint()

    def pickup_complete_callback(self):
        if self.pickup_timer is not None:
            self.pickup_timer.cancel()
            self.destroy_timer(self.pickup_timer)
            self.pickup_timer = None

        self.get_logger().info("Pickup complete.")

        with self.lock:
            self.call_bin_flag = False

            if self.current_waypoint_index >= len(self.waypoints_gps):
                self.state = self.STATE_IDLE
                self.get_logger().info("=== Mission complete! ===")
            else:
                self.state = self.STATE_NAVIGATING
                self.get_logger().info("Resuming navigation.")

    # ================================================================
    # Goal sending
    # ================================================================

    def send_next_waypoint(self):
        with self.lock:
            if self.current_waypoint_index >= len(self.waypoints_gps):
                return
            if not self.ref_set:
                self.get_logger().warn("Reference point not set yet — waiting for GPS fix.")
                return

            wp_gps = self.waypoints_gps[self.current_waypoint_index]
            self.is_navigating = True
            index = self.current_waypoint_index
            total = len(self.waypoints_gps)

        wp_x, wp_y = self.gps_to_xy(wp_gps[0], wp_gps[1])

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
            return

        if self.leader_position is not None:
            dx = wp_x - self.leader_position[0]
            dy = wp_y - self.leader_position[1]
            heading = math.atan2(dy, dx)
        else:
            heading = 0.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp_x
        goal_msg.pose.pose.position.y = wp_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        self.get_logger().info(
            f"[Navigating] WP {index + 1}/{total}: "
            f"({wp_gps[0]:.8f}, {wp_gps[1]:.8f}) -> XY ({wp_x:.2f}, {wp_y:.2f})"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # ================================================================
    # Goal callbacks
    # ================================================================

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().warn("Failed to get goal handle.")
            with self.lock:
                self.is_navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Waypoint goal rejected.")
            with self.lock:
                self.is_navigating = False
            return

        self.get_logger().info("Waypoint goal accepted.")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().warn("No goal result received.")
            with self.lock:
                self.is_navigating = False
            return

        status = result.status

        with self.lock:
            self.is_navigating = False

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"GPS waypoint {self.current_waypoint_index + 1} reached."
                )
                self.current_waypoint_index += 1

                remaining = len(self.waypoints_gps) - self.current_waypoint_index
                if remaining <= 0:
                    self.state = self.STATE_WAITING_FOLLOWER
                    self.call_bin_flag = True
                    self.get_logger().info(
                        "GPS lawnmower pattern complete. "
                        "Waiting for follower to park for final collection."
                    )
                else:
                    self.get_logger().info(f"{remaining} waypoints remaining.")

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    f"GPS waypoint {self.current_waypoint_index + 1} aborted. Skipping."
                )
                self.current_waypoint_index += 1

            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Waypoint goal cancelled (litter detected?).")

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = GPSLeaderBehaviorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GPS leader behaviour node.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()