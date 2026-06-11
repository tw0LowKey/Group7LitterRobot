import board
import digitalio
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BeeperNode(Node):
	def __init__(self):
		super().__init__("beeper_node")

		# Pins
		self.beeper = digitalio.DigitalInOut(board.D16)
		self.beeper.direction = digitalio.Direction.OUTPUT

		# Services
		self.pulseBeeperService = self.create_service(Trigger, "/comms/pulse_beeper", self.pulseBeeperServiceCallback)

		self.get_logger().info("Beeper Node Started")

	def pulseBeeperServiceCallback(self, request, response):
		self.get_logger().info("Received request to pulse beeper")

		self.beeper.value = True
		rclpy.sleep(0.2)
		self.beeper.value = False

		response.success = True

		return response

def main(args=None):
	rclpy.init(args=args)
	beeperNode = BeeperNode()

	try:
		rclpy.spin(beeperNode)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			rclpy.shutdown()
		beeperNode.destroy_node()

if __name__ == "__main__":
	main()
