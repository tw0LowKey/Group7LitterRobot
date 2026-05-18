#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray, Pose

from sapling_interfaces.srv import AreaCoords


class WaypointGeneratorNode(Node):
    """
    Generates lawnmower coverage waypoints from GPS area coordinates.

    Subscribes:
        /gps/fix  — leader GPS for setting reference point

    Publishes:
        /waypoints  — PoseArray of XY waypoints (TRANSIENT_LOCAL)

    Service:
        /area_coord  — receives area corners, generates and publishes waypoints
    """

    EARTH_RADIUS = 6371000.0

    def __init__(self):
        super().__init__("waypoint_generator_node")

        self.cb_group = ReentrantCallbackGroup()

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("waypoint_topic", "/waypoints")
        self.declare_parameter("strip_width", 1.35)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("auto_ref", True)
        self.declare_parameter("ref_lat", 0.0)
        self.declare_parameter("ref_lon", 0.0)

        self.gps_topic = self.get_parameter("gps_topic").value
        self.waypoint_topic = self.get_parameter("waypoint_topic").value
        self.strip_width = self.get_parameter("strip_width").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.auto_ref = self.get_parameter("auto_ref").value
        self.ref_lat = self.get_parameter("ref_lat").value
        self.ref_lon = self.get_parameter("ref_lon").value

        self.ref_set = not self.auto_ref

        # ============================================================
        # QoS — latching so late subscribers get the waypoints
        # ============================================================

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.waypoint_pub = self.create_publisher(
            PoseArray,
            self.waypoint_topic,
            latching_qos,
        )

        # ============================================================
        # Subscriber
        # ============================================================

        self.navsat_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.navsat_callback,
            10,
            callback_group=self.cb_group,
        )

        # ============================================================
        # Service
        # ============================================================

        self.area_srv = self.create_service(
            AreaCoords,
            "/area_coord",
            self.area_coord_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("=== Waypoint Generator Node Started ===")
        self.get_logger().info(f"GPS topic:      {self.gps_topic}")
        self.get_logger().info(f"Waypoint topic: {self.waypoint_topic}")
        self.get_logger().info(f"Strip width:    {self.strip_width}m")
        self.get_logger().info(f"Goal frame:     {self.goal_frame}")

        if self.ref_set:
            self.get_logger().info(
                f"Manual reference: lat={self.ref_lat:.8f}, lon={self.ref_lon:.8f}"
            )
        else:
            self.get_logger().info("Waiting for GPS fix to set reference point...")

    # ============================================================
    # GPS callback
    # ============================================================

    def navsat_callback(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.auto_ref and not self.ref_set:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_set = True
            self.get_logger().info(
                f"Reference point set: lat={self.ref_lat:.8f}, lon={self.ref_lon:.8f}"
            )



    def gps_to_xy(self, lat, lon):
        x = (
            self.EARTH_RADIUS
            * math.radians(lon - self.ref_lon)
            * math.cos(math.radians(self.ref_lat))
        )
        y = self.EARTH_RADIUS * math.radians(lat - self.ref_lat)
        return x, y

    # ============================================================
    # Lawnmower pattern generation
    # ============================================================

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

        turn_buffer_deg = 0.5 / meters_per_deg_lon
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

    # ============================================================
    # Service callback
    # ============================================================

    def area_coord_callback(self, request, response):
        self.get_logger().info("=== Area Coordinate Service Called ===")
        self.get_logger().info(
            f"Top-left:     ({request.top_left_latitude:.8f}, "
            f"{request.top_left_longitude:.8f})"
        )
        self.get_logger().info(
            f"Bottom-right: ({request.bottom_right_latitude:.8f}, "
            f"{request.bottom_right_longitude:.8f})"
        )

        if not self.ref_set:
            self.get_logger().error("Reference point not set — no GPS fix yet.")
            response.success = False
            response.num_waypoints = 0
            return response

        if (request.top_left_latitude <= request.bottom_right_latitude or
                request.top_left_longitude >= request.bottom_right_longitude):
            self.get_logger().error(
                "Invalid area: top-left must be north-west of bottom-right."
            )
            response.success = False
            response.num_waypoints = 0
            return response

        gps_waypoints = self.generate_lawnmower_gps(
            request.top_left_latitude, request.top_left_longitude,
            request.bottom_right_latitude, request.bottom_right_longitude,
        )

        if len(gps_waypoints) == 0:
            self.get_logger().error("No waypoints generated — area too small?")
            response.success = False
            response.num_waypoints = 0
            return response

        pose_array = PoseArray()
        pose_array.header.frame_id = self.goal_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for i, (lat, lon) in enumerate(gps_waypoints):
            x, y = self.gps_to_xy(lat, lon)

            # Calculate heading to next waypoint
            if i < len(gps_waypoints) - 1:
                next_x, next_y = self.gps_to_xy(
                    gps_waypoints[i + 1][0], gps_waypoints[i + 1][1]
                )
                dx = next_x - x
                dy = next_y - y

  
                if abs(dy) > abs(dx) and i + 2 < len(gps_waypoints):
                    ahead_x, ahead_y = self.gps_to_xy(
                        gps_waypoints[i + 2][0], gps_waypoints[i + 2][1]
                    )
                    heading = math.atan2(ahead_y - next_y, ahead_x - next_x)
                else:
                    heading = math.atan2(dy, dx)
            else:
                heading = 0.0

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.z = math.sin(heading / 2.0)
            pose.orientation.w = math.cos(heading / 2.0)

            pose_array.poses.append(pose)

        self.waypoint_pub.publish(pose_array)

        self.get_logger().info(
            f"Published {len(pose_array.poses)} waypoints to {self.waypoint_topic}"
        )
        for i, pose in enumerate(pose_array.poses):
            self.get_logger().info(
                f"  WP{i + 1}: XY ({pose.position.x:.4f}, {pose.position.y:.4f})"
            )

        response.success = True
        response.num_waypoints = len(pose_array.poses)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down waypoint generator node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



