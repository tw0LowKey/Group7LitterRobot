#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sapling_interfaces.srv import AreaCoords


class FakeAreaCoordsServer(Node):
    def __init__(self):
        super().__init__("fake_area_coords_server")

        self.srv = self.create_service(
            AreaCoords,
            "/comms/area_coords",
            self.area_coords_callback,
        )

        self.get_logger().info("Fake /comms/area_coords service server started.")

    def area_coords_callback(self, request, response):
        response.success = True

        # Reference/start point from your launch:
        # origin_latitude  = 53.4668
        # origin_longitude = -2.2339
        #
        # Fake area:
        # 2 m forward/north
        # 0.85 m left/west
        #
        # top-left = north-west corner
        # bottom-right = start/origin corner

        response.top_left_latitude = 53.46681800
        response.top_left_longitude = -2.23393000

        response.bottom_right_latitude = 53.46680000
        response.bottom_right_longitude = -2.23390000

        self.get_logger().info(
            "Responded with fake area coords: "
            "top_left=(53.46681797, -2.23391283), "
            "bottom_right=(53.46680000, -2.23390000)"
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = FakeAreaCoordsServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down fake area coords server.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()