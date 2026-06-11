#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    robot_desc_pkg = get_package_share_directory("robot_description")
    rm_nav_pkg = get_package_share_directory("rm_navigation")
    follower_nav_pkg = get_package_share_directory("follower_nav2_pkg")

    dual_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_desc_pkg, "launch", "dual_robot.launch.py")
        )
    )

    leader_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rm_nav_pkg, "launch", "rm_navigation.launch.py")
        )
    )

    follower_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rm_nav_pkg, "launch", "rm_navigation_follower.launch.py")
        ),
        launch_arguments={
            "slam": "false",
        }.items(),
    )

    behavior_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_nav_pkg, "launch", "gps_behavior_launch.py")
        )
    )

    return LaunchDescription([

        dual_robot_launch,

        TimerAction(
            period=10.0,
            actions=[leader_nav_launch],
        ),

        TimerAction(
            period=15.0,
            actions=[follower_nav_launch],
        ),

        TimerAction(
            period=25.0,
            actions=[behavior_launch],
        ),
    ])