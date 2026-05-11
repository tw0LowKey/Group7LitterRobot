import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction


def generate_launch_description():
    bringup_dir = get_package_share_directory("rm_localization")

    params_file = os.path.join(
        bringup_dir,
        "config",
        "outdoor_ekf_navsat.yaml"
    )

    gps_topic = LaunchConfiguration("gps_topic")

    declare_gps_topic_cmd = DeclareLaunchArgument(
        "gps_topic",
        default_value="/gps/fix",
        description="GPS NavSatFix topic"
    )

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("gps/fix", gps_topic),
            ("odometry/filtered", "/odometry/filtered"),
            ("odometry/gps", "/odometry/gps"),
        ],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[params_file],
        
        
    )

    return LaunchDescription([
        declare_gps_topic_cmd,
        local_ekf_node,
        navsat_node,

        TimerAction(
            period=6.0,
            actions=[global_ekf_node],
        ),
    ])