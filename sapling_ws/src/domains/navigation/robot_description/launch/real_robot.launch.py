from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PythonExpression)
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 
    # 1. Robot Description (Notice: load_gazebo and simulation are now FALSE)
    scout_description_file = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
    scout_description_content = Command([
        FindExecutable(name="xacro"), " ", scout_description_file, 
        " odometry_source:=", LaunchConfiguration("odometry_source"), 
        " load_gazebo:=false", " simulation:=false", 
        " lidar_type:=", LaunchConfiguration("lidar_type")
    ])
    scout_description = {"robot_description": ParameterValue(scout_description_content, value_type=str)}

    # 2. Robot State Publisher (Notice: use_sim_time is now FALSE)
    robot_state_publisher_node = Node(
        name="robot_state_publisher", 
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        output="screen", 
        parameters=[{"use_sim_time": False}, scout_description],
    )

    # 5. Pointcloud to Laserscan (If you are using a real 3D Lidar like Velodyne/Ouster)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node', 
        name='pointcloud_to_laserscan_node', 
        remappings=[('cloud_in', "/points"), ('scan', "/scan")], 
        parameters=[{
            'use_sim_time': False,
            'transform_tolerance': 0.05, 
            'min_height': 0.1, 
            'max_height': 1.0, 
            'angle_min': -3.14, 
            'angle_max': 3.14, 
            'use_inf': True,
            'target_frame': 'mobile_robot_base_link'
        }], 
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"]))
    )   

    # ========================================================================
    # NOTE: You will also need to add the launch node for your specific 
    # REAL Lidar hardware here (e.g., sllidar_ros2, rplidar_ros, velodyne, etc.)
    # Make sure its output topic is remapped to "/scan" or "/points".
    # ========================================================================

    return LaunchDescription([
        # For the real robot, we rely on the wheel encoders over CAN
        DeclareLaunchArgument(name="odometry_source", default_value="encoders"),
        DeclareLaunchArgument(name="lidar_type", default_value="2d"),

        robot_state_publisher_node,
        
        
        pointcloud_to_laserscan_node
    ])