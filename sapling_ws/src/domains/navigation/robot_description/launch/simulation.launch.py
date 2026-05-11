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
    bridge = Node(
        name="ros2_gz_bridge", 
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        parameters=[{
            "config_file": os.path.join(get_package_share_directory("robot_description"), "config", "bridge_gazebo.yaml"),
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }], 
        output="screen",
    )

    # 2. Robot Description
    scout_description_file = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
    scout_description_content = Command([
        FindExecutable(name="xacro"), " ", scout_description_file, 
        " odometry_source:=", LaunchConfiguration("odometry_source"), 
        " load_gazebo:=true", " simulation:=true", 
        " lidar_type:=", LaunchConfiguration("lidar_type")
    ])
    scout_description = {"robot_description": ParameterValue(scout_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        name="robot_state_publisher", 
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        output="screen", 
        parameters=[{"use_sim_time": True}, scout_description],
    )

    spawn_robot_urdf_node = Node(
        name="spawn_robot_urdf",
        package="ros_gz_sim",
        executable="create",
        arguments=["-name","scout_v2","-topic","/robot_description","-x","0","-y","0","-z","0.2"],
        output="screen",
    )

    # 3. Teleop (Removed xterm prefix to avoid crash)
    teleop_keyboard_node = Node(
        name="teleop", 
        package="teleop_twist_keyboard", 
        executable="teleop_twist_keyboard", 
        output="screen",
        # Prefix removed. Run this in a separate terminal if needed.
    )

    # 4. Pointcloud to Laserscan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node', 
        name='pointcloud_to_laserscan_node', 
        remappings=[('cloud_in', "/points"), ('scan', "/scan")], 
        parameters=[{
            'transform_tolerance': 0.05, 
            'min_height': 0.1, 
            'max_height': 1.0, 
            'angle_min': -3.14, 
            'angle_max': 3.14, 
            'use_inf': True,
            'target_frame': 'base_footprint'
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
            # Added -g flag to start Gazebo without GUI if you are having X11/Wayland errors, 
            # or remove -g to see the window.
            launch_arguments={'gz_args': '-r empty.sdf'}.items(), 
        ),

        spawn_robot_urdf_node,
        bridge,
        pointcloud_to_laserscan_node
    ])