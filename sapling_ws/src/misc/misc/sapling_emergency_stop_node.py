import board
import digitalio
import rclpy
import subprocess
from os import environ
from rclpy.node import Node
from std_srvs.srv import SetBool

class EmergencyStopNode(Node):
	def __init__(self):
		super().__init__("emergency_stop_node")

		# Attributes
		self.SAPLING_ROLE = environ.get("SAPLING_ROLE", "")
		self.stopped = False
		self.physicalStopButtonVal = False
		self.virtualStopButtonVal = False

		# Pins
		self.physicalStopButton = digitalio.DigitalInOut(board.D13)
		self.physicalStopButton.direction = digitalio.Direction.INPUT

		# Services
		self.virtualStopButtonService = self.create_service(SetBool, "/comms/virtual_emergency_stop", self.virtualStopButtonServiceCallback)

		# Timers
		self.checkPhysicalEmergencyStopButtonTimer = self.create_timer(0.05, self.checkPhysicalEmergencyStopButton)
		self.checkStopButtonsTimer = self.create_timer(0.1, self.checkStopButtons)

		self.get_logger().info("Emergency Stop Node Started")

	def checkPhysicalEmergencyStopButton(self):
		self.physicalStopButtonVal = not self.physicalStopButton.value
		self.get_logger().debug(f"The value of the phsyical stop button is {self.physicalStopButtonVal}")

	def virtualStopButtonServiceCallback(self, request, response):
		self.virtualStopButtonVal = request.data

		response.success = True
		response.message = f"Virtual emergency stop set to {request.data}"

		return response

	def checkStopButtons(self):
		if self.physicalStopButtonVal or self.virtualStopButtonVal:
			if not self.stopped:
				self.enableEmergencyStop(self.virtualStopButtonVal)
		else:
			if self.stopped:
				self.disableEmergencyStop()

	def enableEmergencyStop(self, virtual: bool):
		try:
			subprocess.run(["sudo", "-n", "ip", "link", "set", "can_scout", "down"], check=True)
			if self.SAPLING_ROLE == "pickbot":
				subprocess.run(["sudo", "-n", "ip", "link", "set", "can_arm", "down"], check=True)

			self.stopped = True
		except subprocess.CalledProcessError as e:
			self.get_logger().error(f"Failed to bring the CAN interface down: {e}")

			return


		if virtual:
			self.get_logger().warn("The virtual emergency stop button has been pressed")
		else:
			self.get_logger().warn("The physical emergency stop button has been pressed")

	def disableEmergencyStop(self):
		try:
			subprocess.run(["sudo", "-n", "ip", "link", "set", "can_scout", "up"], check=True)
			if self.SAPLING_ROLE == "pickbot":
				subprocess.run(["sudo", "-n", "ip", "link", "set", "can_arm", "up"], check=True)

			self.stopped = False
		except subprocess.CalledProcessError as e:
			self.get_logger().error(f"Failed to bring the CAN interface up: {e}")


def main(args=None):
	rclpy.init(args=args)
	emergencyStopNode = EmergencyStopNode()

	try:
		rclpy.spin(emergencyStopNode)
	except KeyboardInterrupt:
		pass
	finally:
		emergencyStopNode.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()
