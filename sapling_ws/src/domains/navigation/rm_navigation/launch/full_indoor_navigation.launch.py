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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import os


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            name="slam",
            default_value="true",
            choices=["true", "false", "True", "False"],
        ),

        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            choices=["true", "false", "True", "False"],
        ),

        DeclareLaunchArgument(
            name="localization",
            default_value="amcl",
            choices=["amcl", "slam_toolbox"],
        ),

        OpaqueFunction(function=launch_setup),
    ])


def wait_for_topic(topic_name):
    return ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"""
            echo '[WAIT] Waiting for topic {topic_name}...'
            until ros2 topic list | grep -qx '{topic_name}'; do
                sleep 1
            done
            echo '[OK] Topic {topic_name} is available'
            """
        ],
        output="screen",
    )


def wait_for_tf(parent, child):
    return ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"""
            echo '[WAIT] Waiting for TF {parent} -> {child}...'
            until timeout 2 ros2 run tf2_ros tf2_echo {parent} {child} 2>/dev/null | grep -q 'Translation'; do
                sleep 1
            done
            echo '[OK] TF {parent} -> {child} is available'
            """
        ],
        output="screen",
    )


def wait_for_nav2():
    return ExecuteProcess(
        cmd=[
            "bash", "-c",
            """
            echo '[WAIT] Waiting for Nav2 /navigate_to_pose action server...'
            until ros2 action info /navigate_to_pose 2>/dev/null | grep -q 'Action servers: 1'; do
                sleep 1
            done
            echo '[OK] Nav2 action server is available'
            """
        ],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):

    slam_arg = LaunchConfiguration("slam")
    sim_arg = LaunchConfiguration("simulation")
    localization_arg = LaunchConfiguration("localization")

    ydlidar_pkg = get_package_share_directory("ydlidar_ros2_driver")
    scout_base_pkg = get_package_share_directory("scout_base")
    robot_desc_pkg = get_package_share_directory("robot_description")
    rm_nav_pkg = get_package_share_directory("rm_navigation")
    rm_loc_pkg = get_package_share_directory("rm_localization")
    # rm_loc_pkg = get_package_share_directory("rm_localization")

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_pkg, "launch", "ydlidar_launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(ydlidar_pkg, "params", "X4-Pro.yaml")
        }.items(),
    )


    scout_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_base_pkg, "launch", "scout_mini_base.launch.py")
        ),
        launch_arguments={
            "port_name": "can_scout",
        }.items(),
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_desc_pkg, "launch", "real_robot.launch.py")
        )
    )

    nav2_launch_file = os.path.join(rm_loc_pkg, "launch", "bringup_launch.py")
    nav2_params_file = os.path.join(rm_nav_pkg, "config", "nav2_real_params.yaml")
    map_yaml_file = os.path.join(rm_nav_pkg, "maps", "warehouse", "map_slam_v2.yaml")

    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": sim_arg,
            "params_file": nav2_params_file,
            "slam": "True",
            "map": "",
            "use_collision_monitor": "false",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", slam_arg, "'.lower() == 'true'"])
        ),
    )

    nav2_localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": sim_arg,
            "params_file": nav2_params_file,
            "slam": "False",
            "map": map_yaml_file,
            "localization": localization_arg,
            "use_collision_monitor": "false",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", slam_arg, "'.lower() == 'false'"])
        ),
    )

    front_node = Node(
        package="waypoint_navigation_pkg",
        executable="new_litter_monitor_node_new",
        name="front_back_node",
        output="screen",
    )
    cv_nav_node = Node(
        package="waypoint_navigation_pkg",
        executable="pointcloud_to_detected_litter_node",
        name="cv_nav_node",
        output="screen",
    )
    litter_nav_node = Node(
        package="waypoint_navigation_pkg",
        executable="single_sweep_litter_node",
        name="litter_nav_node",
        output="screen",
    )

    wait_scan = wait_for_topic("/scan")
    wait_odom = wait_for_topic("/odom")
    wait_tf = wait_for_tf("odom", "base_link")
    wait_nav2_ready = wait_for_nav2()

    return [

        ydlidar_launch,

        TimerAction(
            period=2.0,
            actions=[wait_scan],
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_scan,
                on_exit=[
                    scout_base_launch,
                    TimerAction(period=3.0, actions=[wait_odom]),
                ],
            )
        ),


        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_odom,
                on_exit=[
                    robot_description_launch,
                    TimerAction(period=3.0, actions=[wait_tf]),
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_tf,
                on_exit=[
                    nav2_slam_launch,
                    nav2_localization_navigation_launch,
                    TimerAction(period=5.0, actions=[wait_nav2_ready]),
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_nav2_ready,
                on_exit=[cv_nav_node],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_nav2_ready,
                on_exit=[litter_nav_node],
            )
        ),
    ]