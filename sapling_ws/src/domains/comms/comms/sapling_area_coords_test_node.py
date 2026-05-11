import rclpy
import math
from rclpy.node import Node
from sapling_interfaces.srv import AreaCoords

class AreaCoordsNode(Node):
	def __init__(self):
		super().__init__("sapling_area_coords_test_node")

		# Define the Square
		topLeftLatitude = 53.47244480
		topLeftLongitude = -2.23477520
		bottomRightLatitude = 53.47249628
		bottomRightLongitude = -2.23485312

		self.areaCoords = {
			"topLeftLatitude": topLeftLatitude,
			"topLeftLongitude": topLeftLongitude,
			"bottomRightLatitude": bottomRightLatitude,
			"bottomRightLongitude": bottomRightLongitude
		}

		# Services
		self.areaCoordsService = self.create_service(
			AreaCoords,
			"/comms/area_coords",
			self.areaCoordsServiceCallback
		)

		self.get_logger().info(f"Sapling Area Coords Node Started")
		self.get_logger().info(f"Top Left: ({self.areaCoords['topLeftLatitude']}, {self.areaCoords['topLeftLongitude']})")
		self.get_logger().info(f"Bottom Right: ({self.areaCoords['bottomRightLatitude']}, {self.areaCoords['bottomRightLongitude']})")

	def areaCoordsServiceCallback(self, request, response):
		response.success = True
		response.top_left_latitude = self.areaCoords["topLeftLatitude"]
		response.top_left_longitude = self.areaCoords["topLeftLongitude"]
		response.bottom_right_latitude = self.areaCoords["bottomRightLatitude"]
		response.bottom_right_longitude = self.areaCoords["bottomRightLongitude"]

		return response

def main(args=None):
	rclpy.init(args=args)
	areaCoordsNode = AreaCoordsNode()

	try:
		rclpy.spin(areaCoordsNode)
	except KeyboardInterrupt:
		pass
	finally:
		areaCoordsNode.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()
