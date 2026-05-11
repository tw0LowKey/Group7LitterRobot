#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus


class FakeFixedGpsNode(Node):
    def __init__(self):
        super().__init__("fake_fixed_gps_node")

        self.declare_parameter("latitude", 53.46680000)
        self.declare_parameter("longitude", -2.23390000)
        self.declare_parameter("altitude", 50.0)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("publish_rate", 5.0)

        self.latitude = float(self.get_parameter("latitude").value)
        self.longitude = float(self.get_parameter("longitude").value)
        self.altitude = float(self.get_parameter("altitude").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_gps)

        self.get_logger().info("Fake fixed GPS node started")
        self.get_logger().info(
            f"Publishing /gps/fix: lat={self.latitude}, lon={self.longitude}, alt={self.altitude}, frame={self.frame_id}"
        )

    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude

        # Small covariance = high confidence RTK-style GPS
        msg.position_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.04,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeFixedGpsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()