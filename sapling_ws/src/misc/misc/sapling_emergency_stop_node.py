import board
import digitalio
import rclpy
import subprocess
from collections import deque
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
		self.lastPhysicalStopButtonVal = False
		self.physicalStopButtonQueue = deque(maxlen=30)
		self.physicalStopButtonTriggerThreshold = 12

		# Pins
		self.physicalStopButton = digitalio.DigitalInOut(board.D19)
		self.physicalStopButton.direction = digitalio.Direction.INPUT

		# Services
		self.virtualStopButtonService = self.create_service(SetBool, "/comms/virtual_emergency_stop", self.virtualStopButtonServiceCallback)

		# Timers
		self.checkPhysicalEmergencyStopButtonTimer = self.create_timer(0.01, self.checkPhysicalEmergencyStopButton)
		self.checkStopButtonsTimer = self.create_timer(0.1, self.checkStopButtons)

		self.get_logger().info("Emergency Stop Node Started")

	def checkPhysicalEmergencyStopButton(self):
		self.physicalStopButtonQueue.append(not self.physicalStopButton.value)

		currentPhysicalStopButtonVal = self.physicalStopButtonQueue.count(True) > self.physicalStopButtonTriggerThreshold

		if currentPhysicalStopButtonVal != self.lastPhysicalStopButtonVal:
			self.physicalStopButtonVal = currentPhysicalStopButtonVal
			self.lastPhysicalStopButtonVal = currentPhysicalStopButtonVal

			if self.physicalStopButtonVal:
				self.get_logger().warn("The physical emergency stop button has been pressed")
			else:
				self.get_logger().warn("The physical emergency stop button has been released")

	def virtualStopButtonServiceCallback(self, request, response):
		self.virtualStopButtonVal = request.data

		response.success = True
		response.message = f"Virtual emergency stop set to {request.data}"

		if self.virtualStopButtonVal:
			self.get_logger().warn("The virtual emergency stop button has been pressed")
		else:
			self.get_logger().warn("The virtual emergency stop button has been released")

		return response

	def checkStopButtons(self):
		if self.physicalStopButtonVal or self.virtualStopButtonVal:
			if not self.stopped:
				self.enableEmergencyStop()
		else:
			if self.stopped:
				self.disableEmergencyStop()

	def enableEmergencyStop(self):
		try:
			subprocess.run(["sudo", "-n", "ip", "link", "set", "can_scout", "down"], check=True)
			if self.SAPLING_ROLE == "pickbot":
				subprocess.run(["sudo", "-n", "ip", "link", "set", "can_arm", "down"], check=True)

			self.stopped = True

		except subprocess.CalledProcessError as e:
			self.get_logger().error(f"Failed to bring the CAN interface down: {e}")

	def disableEmergencyStop(self):
		try:
			subprocess.run(["sudo", "-n", "ip", "link", "set", "can_scout", "up", "type", "can", "bitrate", "500000"], check=True)
			if self.SAPLING_ROLE == "pickbot":
				subprocess.run(["sudo", "-n", "ip", "link", "set", "can_arm", "up", "type", "can", "bitrate", "1000000", "sample-point", "0.875"], check=True)

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
		if rclpy.ok():
			rclpy.shutdown()
		emergencyStopNode.destroy_node()

if __name__ == "__main__":
	main()
