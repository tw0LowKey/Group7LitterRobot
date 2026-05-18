from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_robot_description = get_package_share_directory("robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    arg_odometry_source = DeclareLaunchArgument("odometry_source", default_value="ground_truth")
    arg_lidar_type = DeclareLaunchArgument("lidar_type", default_value="2d")

    gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
    ),
    launch_arguments={'gz_args': '-r ' + os.path.join(pkg_robot_description, 'worlds', 'empty_gps.sdf')}.items(),
)


    leader_description_file = os.path.join(pkg_robot_description, "urdf", "leader_robot.urdf.xacro")
    leader_description_content = Command([
        FindExecutable(name="xacro"), " ", leader_description_file,
        " odometry_source:=", LaunchConfiguration("odometry_source"),
        " load_gazebo:=true", " simulation:=true",
        " lidar_type:=", LaunchConfiguration("lidar_type")
    ])
    
    leader_rsp = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, {"robot_description": ParameterValue(leader_description_content, value_type=str)}],
    )

    leader_spawn = Node(
        name="spawn_leader",
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "leader", "-topic", "/robot_description", "-x", "0", "-y", "0", "-z", "0.2"],
        output="screen",
    )

    leader_bridge = Node(
        name="ros2_gz_bridge_leader",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(pkg_robot_description, "config", "bridge_gazebo_leader.yaml"),
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen",
    )

    follower_description_file = os.path.join(pkg_robot_description, "urdf", "follower_robot.urdf.xacro")
    follower_description_content = Command([
        FindExecutable(name="xacro"), " ", follower_description_file,
        " odometry_source:=", LaunchConfiguration("odometry_source"),
        " load_gazebo:=true", " simulation:=true",
        " lidar_type:=", LaunchConfiguration("lidar_type"),
        " prefix:=follower_",
        " enable_sensors:=false"
    ])

    follower_rsp = Node(
        name="robot_state_publisher",
        namespace="follower",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, {"robot_description": ParameterValue(follower_description_content, value_type=str)}],
    )

    follower_spawn = Node(
        name="spawn_follower",
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "follower", "-topic", "/follower/robot_description", "-x", "-2.0", "-y", "0", "-z", "0.2"],
        output="screen",
    )

    follower_bridge = Node(
        name="ros2_gz_bridge_follower",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(pkg_robot_description, "config", "bridge_gazebo_follower.yaml"),
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen",
    )

    follower_tf_relay = Node(
        package="topic_tools",
        executable="relay",
        name="follower_tf_relay",
        arguments=["/follower/tf", "/tf"],
        output="screen",
    )
    follower_tf_static_relay = Node(
        package="topic_tools",
        executable="relay",
        name="follower_tf_static_relay",
        arguments=["/tf_static", "/follower/tf_static"],
        output="screen",
    )


    # --- 4. Pointcloud Processing (Lidar 3D handling) ---
    leader_pcl = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='leader_pcl_to_scan',
        remappings=[('cloud_in', "/points"), ('scan', "/scan")],
        parameters=[{'target_frame': 'base_footprint', 'use_inf': True}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"]))
    )

    follower_pcl = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='follower_pcl_to_scan',
        namespace='follower',
        remappings=[('cloud_in', "/follower/pointcloud"), ('scan', "/follower/scan")],
        parameters=[{'target_frame': 'follower_base_footprint', 'use_inf': True}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"]))
    )

    return LaunchDescription([
        arg_odometry_source,
        arg_lidar_type,
        gz_sim,
        # Leader Nodes
        leader_rsp,
        leader_spawn,
        leader_bridge,
        leader_pcl,
        # Follower Nodes (spawn delayed by 5s)
        follower_rsp,
        follower_spawn,
        follower_bridge,
        follower_tf_relay,
        follower_tf_static_relay,
        follower_pcl
    ])
