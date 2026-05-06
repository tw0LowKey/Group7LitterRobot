from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    scout_description_file = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf",
        "robot.urdf.xacro"
    )

    scout_description_content = Command([
        FindExecutable(name="xacro"), " ", scout_description_file,
        " odometry_source:=", LaunchConfiguration("odometry_source"),
        " load_gazebo:=false",
        " simulation:=false",
        " lidar_type:=", LaunchConfiguration("lidar_type")
    ])

    scout_description = {
        "robot_description": ParameterValue(
            scout_description_content,
            value_type=str
        )
    }

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------
    robot_state_publisher_node = Node(
        name="follower_robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            scout_description
        ],
        remappings=[
            ("/joint_states", "/follower/joint_states"),
        ]
    )

    # --------------------------------------------------
    # 3D Pointcloud to 2D Laserscan
    # Only used if lidar_type:=3d
    # --------------------------------------------------
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="follower_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/follower/pointcloud"),
            ("scan", "/follower/scan"),
        ],
        parameters=[{
            "use_sim_time": False,
            "transform_tolerance": 0.05,
            "min_height": 0.1,
            "max_height": 1.0,
            "angle_min": -3.14,
            "angle_max": 3.14,
            "use_inf": True,
            "target_frame": "follower/base_link"
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("lidar_type"), "'", " == '3d'"
            ])
        )
    )

    # --------------------------------------------------
    # Follower Behaviour Node
    # --------------------------------------------------
    follower_node = Node(
        package="waypoint_navigation_pkg",
        executable="gps_follower_behaviour_node",
        name="gps_follower_behavior_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,

            "follower_navsat_topic": "/follower/gps/fix",
            "follower_imu_topic": "/follower/imu",
            "follower_action": "/follower/navigate_to_pose",
            "goal_frame": "follower/odom",

            "leader_pose_rx_topic": "/comms/leader_to_follower_rx",
            "follower_status_tx_topic": "/comms/follower_to_leader_tx",

            "breadcrumb_distance": 1.0,
            "min_follow_distance": 2.0,
            "approach_distance": 0.5,
            "park_distance": 0.2,
            "skip_park_distance": 0.9,
            "resume_distance": 2.5,

            "leader_half_length": 0.325,
            "follower_half_length": 0.325,

            "follower_gps_offset_x": -0.11,
            "follower_gps_offset_y": -0.16,

            "auto_ref": True,
            "ref_lat": 0.0,
            "ref_lon": 0.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="odometry_source",
            default_value="encoders"
        ),

        DeclareLaunchArgument(
            name="lidar_type",
            default_value="2d"
        ),

        robot_state_publisher_node,
        pointcloud_to_laserscan_node,
        follower_node,
    ])