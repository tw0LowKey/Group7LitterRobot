from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="comms",
			executable="sapling_comms_node",
			name="sapling_comms_node",
			output="screen"
		),
		Node(
			package="comms",
			executable="sapling_executor_node",
			name="sapling_executor_node",
			output="screen"
		),
		Node(
            package="comms",
            executable="sapling_video_node",
            name="sapling_video_node",
            output="screen",
            parameters=[
                {
                    "image_topic": "/camera/color/image_raw",
                    "rtsp_url": "rtsp://127.0.0.1:8554/robot",
                    "fps": 10,
                    "width": 640,
                    "height": 480,
                    "bitrate": "3000k",
					"log_fps": False,
                }
            ],
        ),	# ros2 run comms sapling_teleop_video_node --ros-args -p image_topic:=/camera/color/image_raw -p rtsp_url:=rtsp://127.0.0.1:8554/robot -p fps:=10 -p width:=640 -p height:=480 -p bitrate:=3000k
	])
