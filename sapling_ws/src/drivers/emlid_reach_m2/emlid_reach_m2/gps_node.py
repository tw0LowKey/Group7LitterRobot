import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import Serial, SerialException
from serial.tools import list_ports
from threading import Thread
from time import sleep
from typing import Optional

class ReachM2Node(Node):
	def __init__(self):
		super().__init__("gps_node")

		# Attributes
		self.baudRate = 115200
		self.port = self.getSerialPort()
		self.qualityMap = {
			"1": NavSatStatus.STATUS_GBAS_FIX,	# RTK Fixed - Best precision
			"2": NavSatStatus.STATUS_FIX,		# RTK Float - Decent precision
			"4": NavSatStatus.STATUS_GBAS_FIX,	# DGPS
			"5": NavSatStatus.STATUS_FIX		# Single - Standard GPS
		}

		# Publishers
		self.gpsPublisher = self.create_publisher(NavSatFix, "/gps/fix", 10)

		# Prepare the background serial thread
		self.serialThread = Thread(target=self.readSerialPort, daemon=True)
		self.serialThread.start()

		self.get_logger().info(f"Reach M2 Node started")

	def _shutdown(self):
		raise KeyboardInterrupt

	def getSerialPort(self):
		ports = list_ports.comports()
		acmPorts = [port.device for port in ports if "ttyACM" in port.device]

		if len(acmPorts) == 0:
			self.get_logger().fatal(f"Could not find the Emlid M2 module or any ACM ports - Check the connection")
			self._shutdown()

		return acmPorts[0]

	def readSerialPort(self):
		while rclpy.ok():
			try:
				with Serial(self.port, self.baudRate, timeout=1) as ser:
					self.get_logger().info(f"Successfully connected to {self.port}")

					# Flush buffer once at the start
					ser.reset_input_buffer()

					while rclpy.ok():
						rawLine = ser.readline()

						if not rawLine:
							self.get_logger().warn("Cannot produce GPS coordinates - Antenna may have an obstructed sky view")
							continue

						try:
							line = rawLine.decode("ascii", errors="replace").strip()
							parts = line.split()
						except Exception as e:
							self.get_logger().error(f"Error whilst processing the raw line: {e}")

						msg = self.parseLLHParts(parts)

						self.gpsPublisher.publish(msg)

			except (SerialException, OSError, AttributeError) as e:
				self.get_logger().warn(f"Serial port unavailable - Retrying: {e}")
				sleep(2)

	def parseLLHParts(self, parts: list) -> Optional[NavSatFix]:
		try:
			# Create the message
			msg = NavSatFix()
			msg.header.stamp = self.get_clock().now().to_msg()
			msg.header.frame_id = "gps_link"

			# Parse the LLH data
			msg.latitude = float(parts[2])
			msg.longitude = float(parts[3])
			msg.altitude = float(parts[4])

			# Map Emlid Quality to ROS NavSatStatus
			# Quality 1: Fixed, 2: Float, 4: DGPS, 5: Single
			quality = parts[5]
			msg.status.status = self.qualityMap.get(quality, NavSatStatus.STATUS_NO_FIX)
			self.get_logger().debug(f"Quality of the GPS signal: {quality} [{msg.status.status}]")

			return msg

		except (ValueError, IndexError):
			self.get_logger().debug(f"Received partial or malformed data: {line}")

			return None

def main(args=None):
	rclpy.init(args=args)
	reachM2Node = ReachM2Node()

	try:
		rclpy.spin(reachM2Node)
	except KeyboardInterrupt:
		pass
	finally:
		reachM2Node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()
