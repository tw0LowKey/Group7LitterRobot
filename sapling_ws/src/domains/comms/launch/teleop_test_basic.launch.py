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
			package="web_video_server",
			executable="web_video_server",
			name="web_video_server",
			output="screen"
		)
	])
