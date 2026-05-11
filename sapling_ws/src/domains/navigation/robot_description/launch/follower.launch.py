from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    # 1. Bridge
    # Note: Ensure you create a 'bridge_gazebo_follower.yaml' config for the follower's topics!
    bridge = Node(
        name="ros2_gz_bridge_follower", 
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        parameters=[{
            "config_file": os.path.join(get_package_share_directory("robot_description"), "config", "bridge_gazebo_follower.yaml"),
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }], 
        output="screen",
    )

    # 2. Robot Description
    # Updated to point to the new follower xacro file
    scout_description_file = os.path.join(get_package_share_directory("robot_description"), "urdf", "follower_robot.urdf.xacro")
    scout_description_content = Command([
        FindExecutable(name="xacro"), " ", scout_description_file, 
        " odometry_source:=", LaunchConfiguration("odometry_source"), 
        " load_gazebo:=true", " simulation:=true", 
        " lidar_type:=", LaunchConfiguration("lidar_type"),
        " prefix:=follower_"
    ])
    scout_description = {"robot_description": ParameterValue(scout_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        name="robot_state_publisher", 
        namespace="follower",
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        output="screen", 
        parameters=[{"use_sim_time": True}, scout_description],
    )

    # Updated spawn name to scout2, topic to /follower/robot_description, and offset the starting X position
    spawn_robot_urdf_node = Node(
        name="spawn_robot_urdf",
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "scout2", "-topic", "/follower/robot_description", "-x", "-1.5", "-y", "0", "-z", "0.2"],
        output="screen",
    )

    # 3. Teleop 
    teleop_keyboard_node = Node(
        name="teleop", 
        package="teleop_twist_keyboard", 
        executable="teleop_twist_keyboard", 
        output="screen",
    )

    # 4. Pointcloud to Laserscan
    # Updated target_frame and topic remappings to use the follower prefix
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node', 
        name='pointcloud_to_laserscan_node', 
        namespace='follower',
        remappings=[('cloud_in', "/follower/pointcloud"), ('scan', "/follower/scan")], 
        parameters=[{
            'transform_tolerance': 0.05, 
            'min_height': 0.1, 
            'max_height': 1.0, 
            'angle_min': -3.14, 
            'angle_max': 3.14, 
            'use_inf': True,
            'target_frame': 'follower_base_footprint'
        }], 
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"]))
    )   

    return LaunchDescription([
        DeclareLaunchArgument(name="odometry_source", default_value="ground_truth"),
        DeclareLaunchArgument(name="rviz", default_value="false"),
        DeclareLaunchArgument(name="lidar_type", default_value="2d"),

        robot_state_publisher_node,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(), 
        ),

        spawn_robot_urdf_node,
        bridge,
        pointcloud_to_laserscan_node
    ])