import adafruit_rfm9x
import asyncio
import board
import busio
import digitalio
import rclpy
from bleak.backends.characteristic import BleakGATTCharacteristic
from bless import BlessServer, GATTCharacteristicProperties, GATTAttributePermissions
from comms.sapling_shared import decodePacket
from json import dumps, loads, JSONDecodeError
from os import environ
from pprint import pformat
from queue import Queue, Empty
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
		super().__init__("sapling_comms_node")

		# Attributes
		self.protocol = None
		self.secretKey = None

		# Bluetooth Attributes
		self.btBuffer = b""
		self.btName = btName
		self.charUuid = charUuid
		self.provisionedEvent = None
		self.server = None
		self.serviceUuid = serviceUuid

		# LoRa Attributes
		self.loraNodeId = None
		self.loraRadio = None
		self.loraRunning = True
		self.loraTxQueue = Queue()

		# Initialise the LoRa radio module
		self.initLora()

		# Publishers
		self.loraRxPublisher = self.create_publisher(String, "/comms/lora_rx", 10)
		self.protocolPublisher = self.create_publisher(String, "/comms/comms_protocol", 10)

		# Subscribers
		self.loraTxSub = self.create_subscription(
			LoraTransmission,
			"/comms/lora_tx",
			self.loraTxCallback,
			10
		)

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
				# Not enough data yet or invalid encoding - wait for next chunk
				return

			# If we reach here, we have a complete JSON object
			self.loraNodeId = payload.get("loraNodeId")
			self.secretKey = payload.get("secretKey")
			self.protocol = payload.get("protocol")
			self.get_logger().info(f"Provisioned Data:\n\tID={self.loraNodeId},\n\tKey={self.secretKey},\n\tProtocol={pformat(self.protocol, sort_dicts=False)}")

			msg = String()
			msg.data = dumps(self.protocol)
			self.protocolPublisher.publish(msg)

			# Clear the buffer
			self.btBuffer = b""

			# Signal the script to stop BLE and transition to LoRa
			self.server.loop.call_soon_threadsafe(self.provisionedEvent.set)

		except Exception as e:
			self.get_logger().error(f"BLE Callback Error: {e}")

	def initLora(self) -> None:
		""" Initialises the RFM9X hardware via SPI """

		try:
			spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
			cs = digitalio.DigitalInOut(board.CE1)
			reset = digitalio.DigitalInOut(board.D13)

			self.loraRadio = adafruit_rfm9x.RFM9x(spi, cs, reset, 433.0)
			self.loraRadio.tx_power = 23 # Max

			self.get_logger().info("LoRa Radio Initialised")

		except Exception as e:
			self.get_logger().fatal(f"LoRa Hardware Error: {e}")
			exit()

	def loraTxCallback(self, msg: LoraTransmission) -> None:
		if self.loraRadio is not None:
			data = msg.data.encode("utf-8")

			if 0 < len(data) <= 252:
				self.loraTxQueue.put((data, msg.destination, ))
				self.get_logger().debug(f"LoRa Tx Queued: {msg.data}")
			else:
				self.get_logger().warn(f"LoRa Tx: Invalid data length ({len(data)}) - Skipping")

	def startLoraThread(self) -> None:
		""" Spawns a background thread so the blocking LoRa radio doesn't stall ROS 2 callbacks """

		self.loraThread = Thread(target=self.loraLoop, daemon=True)
		self.loraThread.start()

	def loraLoop(self) -> None:
		""" Transitions the node into LoRa operational mode (runs in a separate thread) """

		self.loraRadio.node = self.loraNodeId
		self.get_logger().info(f"LoRa Radio listening as Node {self.loraNodeId}")

		while rclpy.ok() and self.loraRunning:
			# Handle the transmitting
			try:
				while True:
					data, destination = self.loraTxQueue.get_nowait()

					self.loraRadio.destination = destination
					self.loraRadio.send_with_ack(data)

					self.get_logger().debug(f"LoRa Message Transmitted: {data}")
			except Empty:
				pass

			# Handle the receiving
			packet = self.loraRadio.receive(with_ack=True, timeout=0.1)

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

def main(args=None) -> None:
	rclpy.init(args=args)
	robotId = environ.get("SAPLING_ID")
	if robotId is None:
		print("SAPLING_ID has not been set - please set the environment variable")
		exit()
	commsNode = CommsNode(robotId, SERVICE_UUID, CHAR_UUID)

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
