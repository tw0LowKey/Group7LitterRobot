from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PythonExpression,)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
	bridge = Node(name="ros2_gz_bridge", package="ros_gz_bridge", executable="parameter_bridge", parameters=[{"config_file": os.path.join(get_package_share_directory("robot_description"), "config", "bridge_gazebo.yaml",), "qos_overrides./tf_static.publisher.durability": "transient_local",}], output="screen",)
	scout_description_file = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
	scout_description_content = Command([FindExecutable(name="xacro"), " ", scout_description_file, " odometry_source:=", LaunchConfiguration("odometry_source"), " load_gazebo:=true", " simulation:=true", " lidar_type:=", LaunchConfiguration("lidar_type")])
	scout_description = {"robot_description": ParameterValue(scout_description_content, value_type=str)}
	robot_state_publisher_node = Node(name="robot_state_publisher", package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[{"use_sim_time": True}, scout_description], remappings=[("/joint_states", "/scout/joint_states"), ("/robot_description", "/scout/robot_description"),],)
	spawn_robot_urdf_node = Node(name="spawn_robot_urdf",package="ros_gz_sim",executable="create",arguments=["-name","scout_v2","-topic","/scout/robot_description","-x","0","-y","0","-z","0.2346","-R","0","-P","0","-Y","0",],output="screen",)
	rviz2_node = Node(package="rviz2",executable="rviz2",arguments=["-d", os.path.join(get_package_share_directory("robot_description"), "rviz", "model_display.rviz",)],parameters=[{"use_sim_time": True}, scout_description],condition=IfCondition(LaunchConfiguration("rviz")),)
	static_tf = Node(package="tf2_ros",executable="static_transform_publisher",arguments=["--x","0.0","--y","0.0","--z","0.0","--yaw","0.0","--pitch","0.0","--roll","0.0","--frame-id","world","--child-frame-id","map",],parameters=[{"use_sim_time": True}])
	teleop_keyboard_node = Node(name="teleop", package="teleop_twist_keyboard", executable="teleop_twist_keyboard", output="screen", prefix="xterm -e",)
	pointcloud_to_laserscan_node = Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', name='pointcloud_to_laserscan_node', remappings=[('cloud_in', "/points"), ('scan', "/laser_scan")], parameters=[{'transform_tolerance': 0.05, 'min_height': 0.0, 'max_height': 1.0, 'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 3.14159 / 180.0 / 2.0, 'scan_time': 1/10, 'range_min': 0.1, 'range_max': 100.0, 'use_inf': True,}], condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"])))	
	return LaunchDescription(
    [
        DeclareLaunchArgument(name="odometry_source",default_value="ground_truth",description="Odometry source (ground_truth or wheel encoders)",choices=["encoders", "ground_truth"],),
        DeclareLaunchArgument(name="rviz", default_value="false", description="Open RViz with model display configuration", choices=["true", "false"],),
        DeclareLaunchArgument(name="lidar_type", default_value="2d", description="choose lidar type: pointcloud with 3d lidar or laserscan with 2d lidar", choices=["3d", "2d"]),
        static_tf,
        robot_state_publisher_node,
        
        # --- CHANGED SECTION START ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        ),
        # --- CHANGED SECTION END ---

        spawn_robot_urdf_node,
        bridge,
        rviz2_node,
        teleop_keyboard_node,
        pointcloud_to_laserscan_node
    ]
)
