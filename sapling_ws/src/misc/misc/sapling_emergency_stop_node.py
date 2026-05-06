import board
import digitalio
import rclpy
import subprocess
from functools import partial
from rclpy.node import Node
from std_srvs.srv import SetBool

class EmergencyStopNode(Node):
	def __init__(self):
		super().__init__("emergency_stop_node")

		# Attributes
		self.stopped = False
		self.physicalStopButtonVal = False
		self.virtualStopButtonVal = False

		# Pins
		self.physicalStopButton = digitalio.DigitalInOut(board.D13)
		self.physicalStopButton.direction = digitalio.Direction.INPUT

		# Services
		self.virtualStopButtonService = self.create_service(SetBool, "/comms/virtual_emergency_stop", self.virtualStopButtonSubscriberCallback)

		# Timers
		self.checkPhysicalEmergencyStopButtonTimer = self.create_timer(0.1, self.checkPhysicalEmergencyStopButton)
		self.checkStopButtonsTimer = self.create_timer(0.1, self.checkStopButtons)

		self.get_logger().info("Emergency Stop Node Started")

	def checkPhysicalEmergencyStopButton(self):
		self.physicalStopButtonVal = not self.physicalStopButton.value

	def virtualStopButtonSubscriberCallback(self, request, response):
		self.virtualStopButtonVal = request.data

	def checkStopButtons(self):
		if self.physicalStopButtonVal or self.virtualStopButtonVal:
			if not self.stopped:
				self.enableEmergencyStop(self.virtualStopButtonVal)
		else:
			if self.stopped:
				self.disableEmergencyStop(self.virtualStopButtonVal)

	def enableEmergencyStop(self, virtual: bool):
		subprocess.run(["sudo", "candown"], check=True)
		self.stopped = True

		if virtual:
			self.get_logger().error("The virtual emergency stop button has been pressed")
		else:
			self.get_logger().error("The physical emergency stop button has been pressed")

	def disableEmergencyStop(self):
		subprocess.run(["sudo", "canup"], check=True)
		self.stopped = False

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
