import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import Serial, SerialException

class ReachM2Node(Node):
	def __init__(self):
		super().__init__("gps_node")

		# Attributes
		self.baud = 115200
		self.port = "/dev/ttyACM1"
		self.ser = None
		self.waitingMsgPrinted = False
		self.qualityMap = {
			"1": NavSatStatus.STATUS_GBAS_FIX,	# RTK Fixed - Best precision
			"2": NavSatStatus.STATUS_FIX,		# RTK Float - Decent precision
			"4": NavSatStatus.STATUS_GBAS_FIX,	# DGPS
			"5": NavSatStatus.STATUS_FIX		# Single - Standard GPS
		}

		# Publishers
		self.gpsPublisher = self.create_publisher(NavSatFix, "/gps/fix", 10)

		# Connect to the Reach M2 module
		self.connectSerial()

		# Timer: Runs every 0.1s (10Hz) to check for new serial data
		self.timer = self.create_timer(0.1, self.timerCallback)
		self.get_logger().info(f"Reach M2 Node started")

	def connectSerial(self):
		try:
			self.ser = Serial(self.port, self.baud, timeout=0.1)

			# Clear stale data
			while self.ser.in_waiting > 1:
				self.ser.readline()

			self.get_logger().info(f"Successfully connected to {self.port}")
			self.waitingMsgPrinted = False

			connected = True

		except SerialException as e:
			if "Errno 16" in str(e):
				if not self.waitingMsgPrinted:
					self.get_logger().warn("Waiting for Emlid M2 to ready up (Device Busy)...")
					self.waitingMsgPrinted = True

			elif "Errno 2" in str(e):
				self.get_logger().error(f"Could not find {self.port} - Check connection")

			connected = False

		return connected

	def timerCallback(self):
		try:
			if self.ser.in_waiting > 0:
				line = self.ser.readline().decode("ascii", errors="replace").strip()
				if not line:
					return

				parts = line.split()
				if len(parts) >= 7:
					# Create message
					msg = NavSatFix()
					msg.header.stamp = self.get_clock().now().to_msg()
					msg.header.frame_id = "gps_link"

					# Parse LLH data
					msg.latitude = float(parts[2])
					msg.longitude = float(parts[3])
					msg.altitude = float(parts[4])

					# Map Emlid Quality to ROS NavSatStatus
					# Quality 1: Fixed, 2: Float, 4: DGPS, 5: Single
					quality = parts[5]
					self.get_logger().warn(f"quality {quality}")
					msg.status.status = self.qualityMap.get(quality, NavSatStatus.STATUS_NO_FIX) # Quality 0 or others are usually No Fix

					# Publish the message
					self.gpsPublisher.publish(msg)
				else:
					self.get_logger().debug(f"Received partial data: {line}")

		except Exception as e:
			self.get_logger().error(f"Error reading serial: {e}")
			self.ser = None # Trigger reconnection

def main(args=None):
	rclpy.init(args=args)
	reachM2Node = ReachM2Node()

	try:
		rclpy.spin(reachM2Node)
	except KeyboardInterrupt:
		pass
	finally:
		reachM2Node.destroy_node()
		rclpy.shutdown()

if __name__ == "__main__":
	main()
