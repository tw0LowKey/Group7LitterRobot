from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="misc",
			executable="sapling_buzzer_node",
			name="sapling_buzzer_node",
			output="screen"
		),
		Node(
			package="misc",
			executable="sapling_emergency_stop_node",
			name="sapling_emergency_stop_node",
			output="screen"
		),
	])
