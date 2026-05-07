from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            description="Simulation or real robot",
            choices=["True", "False", "true", "false"],
        ),

        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):

    sim_arg = LaunchConfiguration("simulation")

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    ydlidar_pkg = get_package_share_directory("ydlidar_ros2_driver")
    scout_base_pkg = get_package_share_directory("scout_base")
    robot_desc_pkg = get_package_share_directory("robot_description")
    rm_nav_pkg = get_package_share_directory("rm_navigation")
    rm_loc_pkg = get_package_share_directory("rm_localization_custom")

    # ============================================================
    # YDLIDAR
    # ============================================================

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ydlidar_pkg,
                "launch",
                "ydlidar_launch.py"
            )
        ),
        launch_arguments={
            "params_file": os.path.join(
                ydlidar_pkg,
                "params",
                "X4-Pro.yaml"
            )
        }.items(),
    )

    # ============================================================
    # CAN UP
    # ============================================================

    can_setup = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            """
            sudo ip link set can_scout down || true
            sudo ip link set can_scout type can bitrate 500000
            sudo ip link set can_scout up
            """
        ],
        output="screen",
    )

    # ============================================================
    # SCOUT BASE
    # ============================================================

    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                scout_base_pkg,
                "launch",
                "scout_mini_base.launch.py"
            )
        ),
        launch_arguments={
            "port_name": "can_scout",
        }.items(),
    )

    # ============================================================
    # ROBOT DESCRIPTION (follower URDF)
    # ============================================================

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_desc_pkg,
                "launch",
                "Fb_real_robot.launch.py"
            )
        )
    )

    # ============================================================
    # NAV2 (controller + costmap only, no SLAM/AMCL)
    # ============================================================

    nav2_launch_file = os.path.join(
        rm_loc_pkg,
        "launch",
        "bringup_launch.py"
    )

    nav2_params_file = os.path.join(
        rm_nav_pkg,
        "config",
        "nav2_real_params_follower.yaml"
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": sim_arg,
            "params_file": nav2_params_file,
            "slam": "False",
            "map": "",
            "use_collision_monitor": "false",
            "namespace": "follower",
            "use_namespace": "true",
        }.items(),
    )

    # ============================================================
    # STATIC TF: map -> follower/odom
    # (replaces SLAM/AMCL — gives Nav2 the map->odom transform)
    # ============================================================

    static_tf_follower = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_follower",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "follower/odom",
        ],
    )

    # ============================================================
    # FOLLOWER BEHAVIOUR NODE
    # ============================================================

    follower_behavior_node = Node(
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
        }],
    )

    # ============================================================
    # LAUNCH SEQUENCE
    # ============================================================

    return [

        # 0 sec : YDLIDAR
        ydlidar_launch,

        # 5 sec : CAN UP
        TimerAction(
            period=5.0,
            actions=[can_setup],
        ),

        # After CAN finishes + 5 sec : SCOUT BASE
        RegisterEventHandler(
            OnProcessExit(
                target_action=can_setup,
                on_exit=[
                    TimerAction(
                        period=5.0,
                        actions=[scout_base_launch],
                    )
                ],
            )
        ),

        # 15 sec : ROBOT DESCRIPTION
        TimerAction(
            period=15.0,
            actions=[robot_description_launch],
        ),

        # 20 sec : NAV2 + STATIC TF
        TimerAction(
            period=20.0,
            actions=[
                nav2_launch,
                static_tf_follower,
            ],
        ),

        # 25 sec : FOLLOWER BEHAVIOUR NODE
        TimerAction(
            period=25.0,
            actions=[follower_behavior_node],
        ),
    ]