
import adafruit_rfm9x
import asyncio
import board
import busio
import digitalio
import rclpy
import struct
from bleak.backends.characteristic import BleakGATTCharacteristic
from bless import BlessServer, GATTCharacteristicProperties, GATTAttributePermissions
from comms.sapling_shared import decodePacket
from json import dumps, loads, JSONDecodeError
from rclpy.node import Node
from sapling_interfaces.msg import LoraTransmission
from sapling_interfaces.srv import AreaCoords
from std_msgs.msg import String
from threading import Thread
from time import sleep

# UUIDs must match the base's Comms class
SERVICE_UUID = "036f33e0-9573-4b0e-88d1-18af960d5a95"
CHAR_UUID = "92eda5fb-c187-4f41-aaf2-3931b9cb4c56"

class CommsNode(Node):
	def __init__(self, btName: str, serviceUuid: str, charUuid: str) -> None:
		# Initialise the ROS 2 Node
		super().__init__("sapling_comms_node")

		self.secretKey = None
		self.loraNodeId = None
		self.loraRadio = None
		self.protocol = None
		self.server = None

		self.btName = btName
		self.charUuid = charUuid
		self.provisionedEvent = None
		self.serviceUuid = serviceUuid
		self.btBuffer = b""

		# Initialise the LoRa radio module
		self.initLora()

		# Create the necessary ROS2 publishers
		self.loraRxPublisher = self.create_publisher(String, "/comms/lora_rx", 10)
		self.protocolPublisher = self.create_publisher(String, "/comms/comms_protocol", 10)

	async def startBleProvisioning(self) -> None:
		""" Starts BT advertising until the secret key is received """

		self.get_logger().info("Starting BLE Server...")

		self.server = BlessServer(name=self.btName)
		self.server.write_request_func = self.onBTWrite

		await self.server.add_new_service(self.serviceUuid)
		await self.server.add_new_characteristic(
			self.serviceUuid,
			self.charUuid,
			GATTCharacteristicProperties.write,		# properties
			None,									# value - None as characteristic is written to
			GATTAttributePermissions.writeable		# permissions
		)

		await self.server.start()
		self.provisionedEvent = asyncio.Event()
		self.get_logger().info(f"Robot ({self.btName}) is advertising... Waiting for Laptop to connect")

		# Wait here until Laptop sends the keys
		await self.provisionedEvent.wait()
		await self.server.stop()
		self.get_logger().info("BLE Stopped - Switching to LoRa mode")

	def onBTWrite(self, characteristic: BleakGATTCharacteristic, value: bytes, **kwargs) -> None:
		""" Callback triggered when the BLE client writes to the characteristic """

		try:
			# Accumulate chunks
			self.btBuffer += value

			try:
				# Attempt to decode and parse the current buffer
				payload = loads(self.btBuffer.decode("utf-8"))
			except (UnicodeDecodeError, JSONDecodeError):
				# Not enough data yet or invalid encoding, wait for next chunk
				return

			# If we reach here, we have a complete JSON object
			self.loraNodeId = payload.get("loraNodeId")
			self.secretKey = payload.get("secretKey")
			self.protocol = payload.get("protocol")
			self.get_logger().info(f"Provisioned Data: ID={self.loraNodeId}, Key={self.secretKey}, Protocol={self.protocol}")

			msg = String()
			msg.data = dumps(self.protocol)
			self.protocolPublisher.publish(msg)

			# Clear the buffer for future use
			self.btBuffer = b""

			# Signal the script to stop BLE and transition to LoRa
			self.server.loop.call_soon_threadsafe(self.provisionedEvent.set)
		except Exception as e:
			self.get_logger().error(f"BLE Callback Error: {e}")

	def initLora(self) -> None:
		""" Initialises the RFM9X hardware via SPI """

		try:
			spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
			cs = digitalio.DigitalInOut(board.CE0)
			reset = digitalio.DigitalInOut(board.D6)

			reset.direction = digitalio.Direction.OUTPUT
			reset.value = False
			sleep(0.1)
			reset.value = True
			sleep(0.1)

			self.loraRadio = adafruit_rfm9x.RFM9x(spi, cs, reset, 433.0, baudrate=1000000)
			self.loraRadio.tx_power = 23 # Max

			self.loraTxSub = self.create_subscription(
				LoraTransmission,
				"/comms/lora_tx",
				self.loraTxCallback,
				10
			)

			self.get_logger().info(f"LoRa Radio Initialised")

		except Exception as e:
			self.get_logger().fatal(f"LoRa Hardware Error: {e}")
			exit()

	def startLoraThread(self) -> None:
		""" Spawns a background thread so the blocking LoRa radio doesn't stall ROS 2 callbacks """

		self.loraThread = Thread(target=self.loraReceiveLoop, daemon=True)
		self.loraThread.start()

	def loraReceiveLoop(self) -> None:
		""" Transitions the node into LoRa operational mode (runs in a separate thread) """

		self.loraRadio.node = self.loraNodeId
		self.get_logger().info(f"LoRa Radio listening as Node {self.loraNodeId}")

		while rclpy.ok():
			packet = self.loraRadio.receive(with_ack=True, timeout=1.0)

			if packet is not None:
				try:
					decodedPacket = decodePacket(packet, self.protocol)
					cmdName, payload = decodedPacket["cmdName"], decodedPacket["payload"]

					if decodedPacket["cmdName"] == "unknown":
						self.get_logger().error(f"Payload decode error: cmdId - {payload['cmdId']} | packet - {packet}")
						return

					msg = String()
					msg.data = dumps(decodedPacket)
					self.loraRxPublisher.publish(msg)
					self.get_logger().info(f"Payload decoded: {cmdName} -> {payload}")

				except Exception as e:
					self.get_logger().error(f"Payload decode error: {e}")

	def loraTxCallback(self, msg: LoraTransmission) -> None:
		if self.loraRadio is not None:
			data = msg.data.encode("utf-8")

			if 0 < len(data) <= 252:
				self.loraRadio.destination = msg.destination
				self.loraRadio.send_with_ack(data)
				self.get_logger().info(f"LoRa Tx: {msg.data}")
			else:
				self.get_logger().warn(f"LoRa Tx: Invalid data length ({len(data)}) - Skipping")

def main(args=None) -> None:
	rclpy.init(args=args)
	commsNode = CommsNode("R-067", SERVICE_UUID, CHAR_UUID)

	try:
		asyncio.run(commsNode.startBleProvisioning())

		if commsNode.loraNodeId is not None:
			commsNode.startLoraThread()

			rclpy.spin(commsNode)
		else:
			commsNode.get_logger().error("Provisioning failed to yield a LoRa Node ID - Exiting")
	except KeyboardInterrupt:
		# if commsNode.server is not None:
		# 	if commsNode.server.is_advertising:
		# 		# await commsNode.server.stop()
		# 		commsNode.get_logger().info("BLE server shutting down safely")
		pass
	finally:
		commsNode.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()
