import rclpy
import socket
import struct
from comms.sapling_shared import encodePacket
from geometry_msgs.msg import Twist
from ipaddress import ip_address
from json import loads
from os import environ
from rclpy.node import Node
from rclpy.parameter import Parameter
from sapling_interfaces.msg import FollowerStatus, LeaderPose, LoraTransmission
from sapling_interfaces.srv import AreaCoords
from scout_msgs.msg import ScoutStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

class ExecutorNode(Node):
	def __init__(self):
		super().__init__("sapling_executor_node")

		self.SAPLING_ROLE = environ.get("SAPLING_ROLE", "")
		self.protocol = None
		self.batteryPercentage = 0
		self.lat = 0.0
		self.lng = 0.0
		self.areaCoords = None

		if self.SAPLING_ROLE not in ("pickbot", "binbot"):
			if self.SAPLING_ROLE == "":
				self.get_logger().fatal("The SAPLING_ROLE environment variable has not been set - please set it to either 'pickbot' or 'binbot'")
				self.destroy_node()
				rclpy.shutdown()
			else:
				self.get_logger().fatal("The SAPLING_ROLE environment variable has been set to an unrecognised value - please set it to either 'pickbot' or 'binbot'")
				self.destroy_node()
				rclpy.shutdown()

		# Parameters
		self.declare_parameter("lora_destination", 0)

		# Publishers
		self.loraTxPublisher = self.create_publisher(LoraTransmission, "/comms/lora_tx", 10)
		self.movementPublisher = self.create_publisher(Twist, "/cmd_vel", 10)
		self.sendLeaderToFollowerRxPublisher = self.create_publisher(LeaderPose, "/comms/follower_to_leader_rx", 10)
		self.sendFollowerToLeaderRxPublisher = self.create_publisher(FollowerStatus, "/comms/leader_to_follow_rx", 10)

		# Subscribers
		self.batterySubscriber = self.create_subscription(
			ScoutStatus,
			"/scout_status",
			self.scoutStatusSubscriberCallback,
			10
		)

		self.gpsSubscriber = self.create_subscription(
			ScoutStatus,
			"/gps/fix",
			self.gpsSubscriberCallback,
			10
		)

		self.loraSubscriber = self.create_subscription(
			String,
			"/comms/lora_rx",
			self.loraSubscriberCallback,
			10
		)

		self.protocolSubscriber = self.create_subscription(
			String,
			"/comms/comms_protocol",
			self.protocolSubscriberCallback,
			10
		)

		self.sendLeaderToFollowerTxSubscriber = self.create_subscription(
			LeaderPose,
			"/comms/leader_to_follower_tx",
			self.sendLeaderToFollowerTxSubscriberCallback,
			10
		)

		self.sendFollowerToLeaderTxSubscriber = self.create_subscription(
			FollowerStatus,
			"/comms/follower_to_leader_tx",
			self.sendFollowerToLeaderTxSubscriberCallback,
			10
		)

		# Services
		self.areaCoordsService = self.create_service(AreaCoords, "/comms/area_coords", self.areaCoordsServiceCallback)
		self.addLitterMarkerService = self.create_service(Trigger, "/comms/add_litter_marker", self.addLitterMarkerServiceCallback)

		# Clients
		self.pulseBeeperClient = self.create_client(Trigger, "pulse_beeper")
		self.returnToStartClient = self.create_client(Trigger, "return_to_start")

		# Timers
		self.heartbeatTimer = self.create_timer(5.0, self.heartbeatTimerCallback)
		self.sendIpAddressTimer = self.create_timer(5.0, self.sendIpAddressTimerCallback)

		self.get_logger().info("Sapling Executor Node Started - Waiting for Commands")

	def protocolSubscriberCallback(self, msg):
		""" Callback triggered when the communication protocol is published to the /comms/comms_protocol topic """

		try:
			self.protocol = loads(msg.data)
			self.get_logger().info("Protocol updated")
		except Exception as e:
			self.get_logger().error(f"Failed to parse protocol: {e}")

	def loraSubscriberCallback(self, msg):
		""" Processes incoming command messages in the format 'command_name|payload' """

		try:
			# Parse the incoming message
			data = loads(msg.data)
			cmdName = data["cmdName"]
			payload = data["payload"]

			# Handle known commands
			if cmdName == "areaCoords":
				self.executeAreaCoords(payload)
			elif cmdName == "movement":
				self.executeMovement(payload)
			elif cmdName == "resumeAuto":
				self.executeResumeAuto(payload)
			elif cmdName == "setArmStatus":
				self.executeToggleArm(payload)
			elif cmdName == "setBeeperStatus":
				self.executePulseBeeper(payload)
			elif cmdName == "returnToStart":
				self.executeReturnToStart(payload)
			elif cmdName == "sendLeaderToFollower":
				self.sendLeaderToFollowerRx(payload)
			elif cmdName == "sendFollowerToLeader":
				self.sendFollowerToLeaderRx(payload)
			elif cmdName == "assignBinbot":
				self.executeAssignBinbot(payload)
			elif cmdName == "unknown":
				self.get_logger().error(f"Received unknown command: {cmdName}")
			else:
				self.get_logger().info(f"Received Command: {cmdName}, Payload: {payload}")

		except Exception as e:
			self.get_logger().error(f"Error processing command: {e}")

	def scoutStatusSubscriberCallback(self, msg):
		""" Subscriber callback to process incoming ScoutStatus messages, used here to extract battery voltage and calculate battery percentage """

		MAX_VOLTAGE = 29.2
		MIN_VOLTAGE = 22.0

		batteryVoltage = msg.battery_voltage

		batteryPercentage = ((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100
		batteryPercentage = min(batteryPercentage, self.batteryPercentage) # Maintain a stable voltage
		self.batteryPercentage = int(max(0.0, min(100.0, batteryPercentage)))

		# self.get_logger().info(f"battery: {self.batteryPercentage}")

	def gpsSubscriberCallback(self, msg):
		""" Subscriber callback to process incoming GPS messages, used here to extract latitude and longitude """

		self.lat = msg.lat
		self.lng = msg.lng

	def sendLeaderToFollowerTxSubscriberCallback(self, msg: LeaderPose):
		""" Callback triggered when a message is published to the /comms/leader_to_follower_tx topic """

		if self.protocol is None:
			return

		encodedPacket = encodePacket(
			self.protocol,
			"sendLeaderToFollower",
			msg.seq,
			msg.latitude,
			msg.longitude,
			msg.orientation_z,
			msg.orientation_w,
			msg.call_bin
		)

		if encodedPacket is not None:
			msg = LoraTransmission()
			msg.data = encodedPacket
			msg.destination = self.get_parameter("lora_destination").value
			self.loraTxPublisher.publish(msg)

	def sendLeaderToFollowerRx(self, payload):
		""" Callback triggered when a message is published to the /comms/leader_to_follower_rx topic """

		msg = LeaderPose()

		msg.seq = payload.get("seq", -1)
		msg.latitude = payload.get("latitude", 0.0)
		msg.longitude = payload.get("longitude", 0.0)
		msg.orientation_z = payload.get("orientation_z", 0.0)
		msg.orientation_w = payload.get("orientation_w", 0.0)
		msg.call_bin = payload.get("call_bin", False)

		self.sendLeaderToFollowerRxPublisher.publish(msg)

	def sendFollowerToLeaderTxSubscriberCallback(self, msg: FollowerStatus):
		""" Callback triggered when a message is published to the /comms/follower_to_leader_tx topic """

		if self.protocol is None:
			return

		encodedPacket = encodePacket(
			self.protocol,
			"sendFollowerToLeader",
			msg.seq,
			msg.parked,
			msg.park_x,
			msg.park_y,
			msg.bin_ready
		)

		if encodedPacket is not None:
			msg = LoraTransmission()
			msg.data = encodedPacket
			msg.destination = self.get_parameter("lora_destination").value
			self.loraTxPublisher.publish(msg)

	def sendFollowerToLeaderRx(self, payload):
		""" Callback triggered when a message is published to the /comms/leader_to_follow_rx topic """

		msg = FollowerStatus()

		msg.seq = payload.get("seq", -1)
		msg.parked = payload.get("parked", False)
		msg.park_x = payload.get("park_x", 0.0)
		msg.park_y = payload.get("park_y", 0.0)
		msg.bin_ready = payload.get("bin_ready", 0.0)

		self.sendFollowerToLeaderRxPublisher.publish(msg)

	def heartbeatTimerCallback(self):
		""" Timer callback to publish a heartbeat message at regular intervals, containing the battery percentage and GPS coordinates """

		if self.protocol is None:
			return

		encodedPacket = encodePacket(
			self.protocol,
			"heartbeat",
			self.batteryPercentage,
			self.lat,
			self.lng
		)

		if encodedPacket is not None:
			msg = LoraTransmission()
			msg.data = encodedPacket
			self.loraTxPublisher.publish(msg)

	def executeMovement(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'movement' - expects a payload in the format {"direction": "U|D|L|R"} """

		self.get_logger().debug(f"EXECUTING: Movement: {payload}")

		linearSpeed = 0.5 # NOTE: Make this a parameter
		angularSpeed = 1.0

		direction = payload.get("direction", 0.0)

		msg = Twist()
		msg.linear.x = 0.0
		msg.angular.z = 0.0

		if direction == "U":
			msg.linear.x = 1 * linearSpeed
		elif direction == "D":
			msg.linear.x = -1 * linearSpeed
		elif direction == "L":
			msg.angular.z = 1 * angularSpeed
		elif direction == "R":
			msg.angular.z = -1 * angularSpeed
		else:
			pass

		self.movementPublisher.publish(msg)

	def executeAreaCoords(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'areaCoords' - expects a payload in the format {"topLeftLatitude": float, "topLeftLongitude": float, "bottomRightLatitude": float, "bottomRightLongitude": float} """

		self.get_logger().debug(f"EXECUTING: Publishing Area Coords: {payload}")

		self.areaCoords = payload

	def areaCoordsServiceCallback(self, request, response):
		""" Service callback to return the coordinates of the operational area """

		if self.areaCoords is not None:
			response.success = True
			response.top_left_latitude = self.areaCoords["topLeftLatitude"]
			response.top_left_longitude = self.areaCoords["topLeftLongitude"]
			response.bottom_right_latitude = self.areaCoords["bottomRightLatitude"]
			response.bottom_right_longitude = self.areaCoords["bottomRightLongitude"]
		else:
			response.success = False
			response.top_left_latitude = 0.0
			response.top_left_longitude = 0.0
			response.bottom_right_latitude = 0.0
			response.bottom_right_longitude = 0.0

		return response

	def sendIpAddressTimerCallback(self):
		""" Timer callback to send the robot's IP address over LoRa at regular intervals, so the UI can connect to the correct video stream from the robot's onboard camera """

		if self.protocol is None:
			return

		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		try:
			s.connect(("8.8.8.8", 80)) # 8.8.8.8 is a public Google DNS server
			privateIp = s.getsockname()[0]
		except Exception:
			privateIp = None
		finally:
			s.close()

		ipAddress = int(ip_address(privateIp))

		encodedPacket = encodePacket(
			self.protocol,
			"sendCameraIpAddress",
			ipAddress
		)

		if encodedPacket is not None:
			msg = LoraTransmission()
			msg.data = encodedPacket
			self.loraTxPublisher.publish(msg)

	def executeResumeAuto(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'resumeAuto' - expects a payload in the format {} """

		self.get_logger().debug("EXECUTING: Resume Autonomous Navigation")

	def executeToggleArm(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'setArmStatus' - expects a payload in the format {"armExtended": bool} """

		self.get_logger().debug("EXECUTING: Toggle Manipulator Arm")

	def executePulseBeeper(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'setBeeperStatus' - expects a payload in the format {"beeperOn": bool} """

		self.get_logger().debug("EXECUTING: Pulse On-Board Beeper")

		req = Trigger.Request()

		self.pulseBeeperClient.call_async(req)

	def executeReturnToStart(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'returnToStart' - expects a payload in the format {} """

		self.get_logger().debug("EXECUTING: Return to Start Service")

		req = Trigger.Request()
		self.returnToStartClient.call_async(req)

	def executeAssignBinbot(self, payload):
		""" Callback triggered when a message is published to the /comms/lora_rx topic with the command 'assignBinbot' - expects a payload in the format {"binbotNodeId": int} """

		self.get_logger().debug(f"EXECUTING: Assigning new binbot to self with node ID {payload}")

		param = Parameter("lora_destination", Parameter.Type.INTEGER, payload["binbotNodeId"])
		self.set_parameters([param])

	def addLitterMarkerServiceCallback(self, request, response):
		""" Service callback to add a litter marker at the robot's current GPS coordinates """

		self.get_logger().debug(f"EXECUTING: Adding Litter Marker at Lat: {request.top_left_latitude}, Lng: {request.top_left_longitude}")

		if self.protocol is None:
			return

		encodedPacket = encodePacket(
			self.protocol,
			"addLitterMarker"
		)

		if encodedPacket is not None:
			msg = LoraTransmission()
			msg.data = encodedPacket
			self.loraTxPublisher.publish(msg)

			response.success = True

			return response

def main(args=None):
	rclpy.init(args=args)
	executorNode = ExecutorNode()

	try:
		rclpy.spin(executorNode)
	except KeyboardInterrupt:
		pass
	finally:
		executorNode.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()
