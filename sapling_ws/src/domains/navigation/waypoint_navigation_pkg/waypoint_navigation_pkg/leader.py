import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

from sapling_interfaces.msg import LeaderPose, FollowerStatus
from sapling_interfaces.srv import AreaCoords


class GPSLeaderBehaviorNode(Node):
    """
    Real-robot GPS leader behaviour node for lawnmower coverage pattern.

    Uses robot_localization dual-EKF + navsat_transform pipeline:
        - Position in map frame comes from /odometry/global
        - Yaw in map frame comes from /odometry/global
        - Raw lat/lon for follower comms comes from /gps/fix
        - Waypoints are converted from lat/lon -> XY using the fixed datum
          (local ENU tangent plane — values are SMALL metres, never UTM/ECEF)
        - Orientations are PRE-COMPUTED at generation time using look-ahead
          (matches waypoint_generator_node logic — prevents the controller
           from spinning in place at row transitions)
    """

    STATE_NAVIGATING = "NAVIGATING"
    STATE_WAITING_FOLLOWER = "WAITING_FOLLOWER"
    STATE_PICKUP = "PICKUP"
    STATE_IDLE = "IDLE"
    STATE_WAITING_MISSION = "WAITING_MISSION"

    EARTH_RADIUS = 6371000.0

    # Sanity-check: any waypoint farther than this from the datum is
    # almost certainly a frame-conversion bug (ECEF/UTM leaking in).
    MAX_WAYPOINT_DISTANCE_M = 10000.0

    def __init__(self):
        super().__init__("gps_leader_behavior_node")

        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # ================================================================
        # Parameters
        # ================================================================

        self.declare_parameter("leader_action", "/navigate_to_pose")
        self.declare_parameter("leader_navsat_topic", "/gps/fix")
        self.declare_parameter("leader_odom_topic", "/odometry/global")
        self.declare_parameter("pickup_wait_time", 2.0)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("strip_width", 1.35)

        self.declare_parameter("leader_pose_tx_topic", "/comms/leader_to_follower_tx")
        self.declare_parameter("follower_status_rx_topic", "/comms/follower_to_leader_rx")

        self.declare_parameter("area_coords_service", "/comms/area_coords")
        self.declare_parameter("area_poll_period", 2.0)

        # Datum — MUST match navsat_transform's datum parameter exactly.
        self.declare_parameter("datum_lat", 53.47249444)
        self.declare_parameter("datum_lon", -2.23482724)

        self.leader_action = self.get_parameter("leader_action").value
        self.leader_navsat_topic = self.get_parameter("leader_navsat_topic").value
        self.leader_odom_topic = self.get_parameter("leader_odom_topic").value
        self.pickup_wait_time = self.get_parameter("pickup_wait_time").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.strip_width = self.get_parameter("strip_width").value

        self.leader_pose_tx_topic = self.get_parameter("leader_pose_tx_topic").value
        self.follower_status_rx_topic = self.get_parameter("follower_status_rx_topic").value

        self.area_coords_service_name = self.get_parameter("area_coords_service").value
        self.area_poll_period = self.get_parameter("area_poll_period").value

        self.datum_lat = self.get_parameter("datum_lat").value
        self.datum_lon = self.get_parameter("datum_lon").value

        # ================================================================
        # Internal state
        # ================================================================

        # Each waypoint: (lat, lon, x, y, qz, qw) — orientation pre-computed.
        self.waypoints = []

        self.state = self.STATE_WAITING_MISSION
        self.current_waypoint_index = 0

        self.leader_position = None
        self.leader_yaw = None
        self.leader_qz = 0.0
        self.leader_qw = 1.0
        self.have_odom = False

        self.leader_lat = None
        self.leader_lon = None
        self.have_gps_fix = False

        self.is_navigating = False
        self.goal_handle = None
        self.pickup_timer = None
        self.call_bin_flag = False
        self.leader_seq = 0
        self.follower_park_position = None
        self.follower_bin_ready = False
        self.area_request_pending = False
        self.mission_loaded = False

        # ================================================================
        # Publishers / Subscribers
        # ================================================================

        self.leader_pose_pub = self.create_publisher(
            LeaderPose, self.leader_pose_tx_topic, 10
        )

        self.follower_status_sub = self.create_subscription(
            FollowerStatus,
            self.follower_status_rx_topic,
            self.follower_status_callback,
            10,
            callback_group=self.cb_group
        )

        self.call_bin_sub = self.create_subscription(
            Bool, "/call_bin", self.call_bin_callback, 10,
            callback_group=self.cb_group
        )

        self.navsat_sub = self.create_subscription(
            NavSatFix,
            self.leader_navsat_topic,
            self.navsat_callback,
            10,
            callback_group=self.cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.odom_callback,
            10,
            callback_group=self.cb_group
        )

        # ================================================================
        # Action client / service client
        # ================================================================

        self.nav_client = ActionClient(
            self, NavigateToPose, self.leader_action,
            callback_group=self.cb_group
        )

        self.area_client = self.create_client(
            AreaCoords, self.area_coords_service_name,
            callback_group=self.cb_group
        )

        # ================================================================
        # Timers
        # ================================================================

        self.tick_timer = self.create_timer(
            0.5, self.state_machine_tick, callback_group=self.cb_group
        )
        self.leader_pose_timer = self.create_timer(
            0.5, self.publish_leader_pose, callback_group=self.cb_group
        )
        self.area_poll_timer = self.create_timer(
            self.area_poll_period, self.poll_area_coords, callback_group=self.cb_group
        )

        self.print_startup_info()

    # ================================================================
    # Startup logging
    # ================================================================

    def print_startup_info(self):
        self.get_logger().info("=== Real GPS Leader Behaviour Node Started ===")
        self.get_logger().info(f"Leader Nav2 action:       {self.leader_action}")
        self.get_logger().info(f"Leader GPS topic:         {self.leader_navsat_topic}")
        self.get_logger().info(f"Leader odom topic:        {self.leader_odom_topic}")
        self.get_logger().info(f"Goal frame:               {self.goal_frame}")
        self.get_logger().info(f"Leader pose TX topic:     {self.leader_pose_tx_topic}")
        self.get_logger().info(f"Follower status RX topic: {self.follower_status_rx_topic}")
        self.get_logger().info(f"Area coords service:      {self.area_coords_service_name}")
        self.get_logger().info(f"Area poll period:         {self.area_poll_period}s")
        self.get_logger().info(f"Pickup wait time:         {self.pickup_wait_time}s")
        self.get_logger().info(f"Strip width:              {self.strip_width}m")
        self.get_logger().info(
            f"Datum (map origin):       lat={self.datum_lat:.8f}, lon={self.datum_lon:.8f}"
        )

        # Self-test the conversion at startup so any wiring bug is caught
        # before we ever send a goal. Datum->datum must give (0, 0).
        test_x, test_y = self.gps_to_map_xy(self.datum_lat, self.datum_lon)
        self.get_logger().info(
            f"Conversion self-test:     datum->XY = ({test_x:.4f}, {test_y:.4f}) "
            f"[expected (0.0000, 0.0000)]"
        )
        # if abs(test_x) > 0.01 or abs(test_y) > 0.01:
        #     self.get_logger().error(
        #         "Conversion self-test FAILED — gps_to_map_xy is broken!"
        #     )

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

    def gps_to_map_xy(self, lat, lon):
        """Convert lat/lon -> XY in map frame (local ENU), datum as origin.

        Returns small metre values (-thousands to +thousands at most).
        If you ever see millions, something is wrong — see the
        MAX_WAYPOINT_DISTANCE_M guard in build_waypoints_with_orientation.
        """
        x = (
            self.EARTH_RADIUS
            * math.radians(lon - self.datum_lon)
            * math.cos(math.radians(self.datum_lat))
        )
        y = self.EARTH_RADIUS * math.radians(lat - self.datum_lat)
        return x, y

    # ================================================================
    # Leader pose publisher
    # ================================================================

    def publish_leader_pose(self):
        if not self.have_gps_fix:
            return

        self.leader_seq += 1

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
        self.follower_bin_ready = msg.bin_ready

        if msg.park_x != 0.0 or msg.park_y != 0.0:
            self.follower_park_position = (msg.park_x, msg.park_y)

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
    # Lawnmower pattern (lat/lon)
    # ================================================================

    def generate_lawnmower_gps(self, top_left_lat, top_left_lon,
                            bottom_right_lat, bottom_right_lon):

        north_lat = max(top_left_lat, bottom_right_lat)
        south_lat = min(top_left_lat, bottom_right_lat)

        west_lon = min(top_left_lon, bottom_right_lon)
        east_lon = max(top_left_lon, bottom_right_lon)

        row_spacing_deg = self.strip_width / 111320.0

        mid_lat = (north_lat + south_lat) / 2.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(mid_lat))
        point_spacing_deg = 1.0 / meters_per_deg_lon

        turn_buffer_deg = 0.5 / meters_per_deg_lon
        row_west_lon = west_lon + turn_buffer_deg
        row_east_lon = east_lon - turn_buffer_deg

        if row_west_lon >= row_east_lon:
            self.get_logger().error("Area too narrow after turn buffer. No waypoints generated.")
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
    # Pre-compute XY + orientation per waypoint.
    # Matches waypoint_generator_node logic — orientation is set once
    # per waypoint, using look-ahead at row transitions so the
    # controller doesn't spin in place.
    # ================================================================

    def build_waypoints_with_orientation(self, gps_waypoints):
        if len(gps_waypoints) == 0:
            return []

        xy_list = [self.gps_to_map_xy(lat, lon) for (lat, lon) in gps_waypoints]

        # Guard against any waypoint that ended up absurdly far from the
        # datum. With a correct local-ENU conversion this can never happen
        # for a real field; if it does, refuse to load the mission so we
        # don't send Nav2 a goal it cannot plan to.
        # for i, (x, y) in enumerate(xy_list):
        #     dist = math.hypot(x, y)
        #     if dist > self.MAX_WAYPOINT_DISTANCE_M:
        #         self.get_logger().error(
        #             f"Waypoint {i + 1} is {dist:.0f} m from datum "
        #             f"(XY = {x:.2f}, {y:.2f}). This is almost certainly a "
        #             f"frame-conversion bug (UTM/ECEF leaking in). "
        #             f"Refusing to load mission."
        #         )
        #         return []

        out = []
        n = len(gps_waypoints)

        for i in range(n):
            lat, lon = gps_waypoints[i]
            x, y = xy_list[i]

            if i < n - 1:
                next_x, next_y = xy_list[i + 1]
                dx = next_x - x
                dy = next_y - y

                # If next step is mostly north/south (row transition),
                # look ahead to the WP after to get the next row's direction
                if abs(dy) > abs(dx) and i + 2 < n:
                    ahead_x, ahead_y = xy_list[i + 2]
                    heading = math.atan2(ahead_y - next_y, ahead_x - next_x)
                else:
                    heading = math.atan2(dy, dx)

                qz = math.sin(heading / 2.0)
                qw = math.cos(heading / 2.0)
                out.append((lat, lon, x, y, qz, qw))
            else:
                # Final waypoint — reuse previous heading
                if len(out) > 0:
                    prev_qz = out[-1][4]
                    prev_qw = out[-1][5]
                    out.append((lat, lon, x, y, prev_qz, prev_qw))
                else:
                    out.append((lat, lon, x, y, 0.0, 1.0))

        return out

    # ================================================================
    # Area Coordinate Service Client
    # ================================================================

    def poll_area_coords(self):
        if self.mission_loaded or self.area_request_pending:
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
            return

        top_left_lat = response.top_left_latitude
        top_left_lon = response.top_left_longitude
        bottom_right_lat = response.bottom_right_latitude
        bottom_right_lon = response.bottom_right_longitude

        self.get_logger().info("=== Area Coordinates Received from Executor ===")
        self.get_logger().info(
            f"Top-left:     lat={top_left_lat:.8f}, lon={top_left_lon:.8f}"
        )
        self.get_logger().info(
            f"Bottom-right: lat={bottom_right_lat:.8f}, lon={bottom_right_lon:.8f}"
        )

        gps_waypoints = self.generate_lawnmower_gps(
            top_left_lat, top_left_lon,
            bottom_right_lat, bottom_right_lon
        )

        if len(gps_waypoints) == 0:
            self.get_logger().error("No waypoints generated — area too small?")
            return

        full_waypoints = self.build_waypoints_with_orientation(gps_waypoints)

        # build_waypoints_with_orientation returns [] if the sanity guard
        # tripped — don't mark the mission loaded in that case.
        # if len(full_waypoints) == 0:
        #     self.get_logger().error(
        #         "Waypoint build failed sanity check. Mission NOT loaded. "
        #         "Check that this leader.py is actually being run "
        #         "(ros2 pkg prefix waypoint_navigation_pkg should point at "
        #         "your latest install/, not an older overlay)."
        #     )
        #     return

        with self.lock:
            self.waypoints = full_waypoints
            self.current_waypoint_index = 0
            self.is_navigating = False
            self.mission_loaded = True

            if self.have_odom:
                self.state = self.STATE_NAVIGATING
                self.get_logger().info("Odom available — starting immediately!")
            else:
                self.state = self.STATE_IDLE
                self.get_logger().info(
                    "Waypoints loaded — waiting for /odometry/global to start..."
                )

        self.get_logger().info(f"Generated {len(full_waypoints)} waypoints:")
        for i, (lat, lon, x, y, qz, qw) in enumerate(full_waypoints):
            yaw_deg = math.degrees(2.0 * math.atan2(qz, qw))
            self.get_logger().info(
                f"  WP{i + 1}: GPS (lat={lat:.8f}, lon={lon:.8f}) "
                f"-> map XY ({x:.2f}, {y:.2f}) yaw={yaw_deg:.1f} deg"
            )

        if self.area_poll_timer is not None:
            self.area_poll_timer.cancel()

    # ================================================================
    # Sensor callbacks
    # ================================================================

    def navsat_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().warn("Invalid GPS fix received.")
            return

        self.leader_lat = msg.latitude
        self.leader_lon = msg.longitude
        self.have_gps_fix = True

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.leader_position = (p.x, p.y)
        self.leader_qz = q.z
        self.leader_qw = q.w
        self.leader_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if not self.have_odom:
            self.have_odom = True
            self.get_logger().info(
                f"First /odometry/global received: "
                f"x={p.x:.2f}, y={p.y:.2f}, yaw={math.degrees(self.leader_yaw):.1f} deg"
            )
            with self.lock:
                if (self.mission_loaded and
                        self.state == self.STATE_IDLE and
                        len(self.waypoints) > 0):
                    self.state = self.STATE_NAVIGATING
                    self.get_logger().info("Odom received — starting lawnmower pattern!")

    def call_bin_callback(self, msg: Bool):
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

    def tick_navigating(self):
        with self.lock:
            if self.current_waypoint_index >= len(self.waypoints):
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

            if self.current_waypoint_index >= len(self.waypoints):
                self.state = self.STATE_IDLE
                self.get_logger().info("=== Mission complete! ===")
            else:
                self.state = self.STATE_NAVIGATING
                self.get_logger().info("Resuming navigation.")

    # ================================================================
    # Goal sending — uses pre-computed XY + orientation
    # ================================================================

    def send_next_waypoint(self):
        with self.lock:
            if self.current_waypoint_index >= len(self.waypoints):
                return
            if not self.have_odom:
                self.get_logger().warn("No odom yet — waiting for /odometry/global.")
                return

            wp = self.waypoints[self.current_waypoint_index]
            self.is_navigating = True
            index = self.current_waypoint_index
            total = len(self.waypoints)

        lat, lon, wp_x, wp_y, qz, qw = wp

        # Final guard before handing a goal to Nav2 — if anything is
        # absurdly out of range we abort the mission rather than spam
        # the planner with off-costmap goals.
        # if math.hypot(wp_x, wp_y) > self.MAX_WAYPOINT_DISTANCE_M:
        #     self.get_logger().error(
        #         f"Refusing to send WP {index + 1}: XY ({wp_x:.2f}, {wp_y:.2f}) "
        #         f"is impossibly far from datum. Mission aborted."
        #     )
        #     with self.lock:
        #         self.is_navigating = False
        #         self.state = self.STATE_IDLE
        #     return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available.")
            with self.lock:
                self.is_navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp_x
        goal_msg.pose.pose.position.y = wp_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        yaw_deg = math.degrees(2.0 * math.atan2(qz, qw))
        self.get_logger().info(
            f"[Navigating] WP {index + 1}/{total}: "
            f"GPS (lat={lat:.8f}, lon={lon:.8f}) "
            f"-> map XY ({wp_x:.2f}, {wp_y:.2f}) yaw={yaw_deg:.1f} deg"
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

                remaining = len(self.waypoints) - self.current_waypoint_index
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
