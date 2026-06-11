
import cv2
import rclpy
import subprocess
import time
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image
from typing import Optional

class VideoNode(Node):
	def __init__(self):
		super().__init__("sapling_video_node")

		self.declare_parameter("image_topic", "/camera/color/image_raw")
		self.declare_parameter("rtsp_url", "rtsp://127.0.0.1:8554/robot")
		self.declare_parameter("fps", 30)
		self.declare_parameter("bitrate", "3000k")
		self.declare_parameter("width", 640)
		self.declare_parameter("height", 480)
		self.declare_parameter("log_fps", True)

		self.imageTopic = self.get_parameter("image_topic").value
		self.rtspUrl = self.get_parameter("rtsp_url").value
		self.fps = int(self.get_parameter("fps").value)
		self.bitrate = self.get_parameter("bitrate").value
		self.width = int(self.get_parameter("width").value)
		self.height = int(self.get_parameter("height").value)
		self.logFps = bool(self.get_parameter("log_fps").value)

		self.bridge = CvBridge()
		self.ffmpeg = None

		self.frame_count = 0
		self.lastLog = time.monotonic()
		self.frameInterval = 1.0 / self.fps
		self.lastSent = 0.0

		# Set up ROS subscription with appropriate QoS for image streaming
		qos = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.VOLATILE,
			history=HistoryPolicy.KEEP_LAST,
			depth=1,
		)

		self.sub = self.create_subscription(
			Image,
			self.imageTopic,
			self.onImage,
			qos,
		)

		self.get_logger().info(f"Subscribing to {self.imageTopic}")
		self.get_logger().info(f"Streaming to {self.rtspUrl}")
		self.get_logger().info(f"Output: {self.width}x{self.height}@{self.fps}, bitrate {self.bitrate}")

	def startFfmpeg(self):
		""" Starts the FFmpeg subprocess with the appropriate command line arguments """

		cmd = [
			"ffmpeg",
			"-hide_banner",
			"-loglevel", "warning",

			# Input from Python stdin
			"-f", "rawvideo",
			"-pix_fmt", "bgr24",
			"-s", f"{self.width}x{self.height}",
			"-r", str(self.fps),
			"-i", "pipe:0",

			# Low-latency H.264
			"-an",
			"-c:v", "libx264",
			"-preset", "ultrafast",
			"-tune", "zerolatency",
			"-b:v", self.bitrate,
			"-maxrate", self.bitrate,
			"-bufsize", self.bitrate,
			"-g", str(max(1, self.fps // 2)),
			"-keyint_min", str(max(1, self.fps // 2)),
			"-bf", "0",
			"-pix_fmt", "yuv420p",

			# Publish to MediaMTX RTSP
			"-f", "rtsp",
			"-rtsp_transport", "tcp",
			self.rtspUrl,
		]

		self.get_logger().info("Starting FFmpeg:")
		self.get_logger().info(" ".join(cmd))

		self.ffmpeg = subprocess.Popen(
			cmd,
			stdin=subprocess.PIPE,
			bufsize=0,
		)

	def onImage(self, msg: Image):
		""" Callback for incoming images - encodes and sends them to FFmpeg """

		try:
			now = time.monotonic()

			# Only send frames at the specified FPS
			if now - self.lastSent < self.frameInterval:
				return

			self.lastSent = now

			if self.ffmpeg is None:
				self.startFfmpeg()

			# Check if FFmpeg process is still running
			if self.ffmpeg.poll() is not None:
				self.get_logger().error("FFmpeg exited - restarting")
				self.startFfmpeg()

			# Convert ROS Image message to OpenCV format
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

			# Resize if needed
			if frame.shape[1] != self.width or frame.shape[0] != self.height:
				frame = cv2.resize(
					frame,
					(self.width, self.height),
					interpolation=cv2.INTER_AREA,
				)

			# Write raw frame data to FFmpeg stdin
			if self.ffmpeg and self.ffmpeg.stdin:
				self.ffmpeg.stdin.write(frame.tobytes())

			# Log FPS
			if self.logFps:
				self.frame_count += 1
				elapsed = now - self.lastLog

				if elapsed >= 2.0:
					self.get_logger().info(f"Sent FPS: {self.frame_count / elapsed:.1f}")
					self.frame_count = 0
					self.lastLog = now

		except BrokenPipeError:
			self.get_logger().error("Broken pipe to FFmpeg - restarting")
			self.ffmpeg = None

		except Exception as e:
			self.get_logger().error(f"Error: {e}")

	def destroy_node(self):
		""" Clean up FFmpeg process on shutdown """

		if self.ffmpeg is not None:
			try:
				if self.ffmpeg.stdin:
					self.ffmpeg.stdin.close()

				self.ffmpeg.terminate()

			except Exception:
				pass

		super().destroy_node()

def main(args=None):
	rclpy.init(args=args)
	videoNode = VideoNode()

	try:
		rclpy.spin(videoNode)
	except KeyboardInterrupt:
		pass
	finally:
		videoNode.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()
