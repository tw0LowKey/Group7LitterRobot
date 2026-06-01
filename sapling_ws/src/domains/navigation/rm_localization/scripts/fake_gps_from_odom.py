#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


class FakeGpsFromOdom(Node):
    def __init__(self):
        super().__init__("fake_gps_from_odom")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("gps_topic", "/gps/fix")


        self.declare_parameter("origin_latitude", 53.47244480)
        self.declare_parameter("origin_longitude", -2.23477520)
        self.declare_parameter("origin_altitude", 50.0)

   
        self.declare_parameter("noise_std_m", 0.0)

        odom_topic = self.get_parameter("odom_topic").value
        gps_topic = self.get_parameter("gps_topic").value

        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.origin_alt = float(self.get_parameter("origin_altitude").value)
        self.noise_std_m = float(self.get_parameter("noise_std_m").value)

        self.gps_pub = self.create_publisher(NavSatFix, gps_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Fake GPS from odom started")
        self.get_logger().info(f"Subscribing odom: {odom_topic}")
        self.get_logger().info(f"Publishing GPS: {gps_topic}")
        self.get_logger().info(
            f"Origin: lat={self.origin_lat}, lon={self.origin_lon}, alt={self.origin_alt}"
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert local metres to latitude/longitude.
 
        earth_radius = 6378137.0

        delta_lat = (y / earth_radius) * (180.0 / math.pi)
        delta_lon = (x / (earth_radius * math.cos(math.radians(self.origin_lat)))) * (180.0 / math.pi)

        gps_msg = NavSatFix()
        gps_msg.header.stamp = msg.header.stamp
        gps_msg.header.frame_id = "mobile_robot_base_link"
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        gps_msg.latitude = self.origin_lat + delta_lat
        gps_msg.longitude = self.origin_lon + delta_lon
        gps_msg.altitude = self.origin_alt


        gps_msg.position_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 1.0
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gps_pub.publish(gps_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGpsFromOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
