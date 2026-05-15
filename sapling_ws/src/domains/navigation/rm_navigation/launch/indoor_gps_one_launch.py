from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    GroupAction,
    SetEnvironmentVariable,
)

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)

from launch.conditions import IfCondition

import os


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            name="slam",
            default_value="true",
            description="Launch SLAM or localization/navigation",
            choices=["True", "False", "true", "false"],
        ),

        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            description="Simulation or real robot",
            choices=["True", "False", "true", "false"],
        ),

        DeclareLaunchArgument(
            name="localization",
            default_value="amcl",
            description="Localization mode",
            choices=["amcl", "slam_toolbox"],
        ),

        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):

    sim_arg = LaunchConfiguration("simulation")

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    ydlidar_pkg = get_package_share_directory(
        "ydlidar_ros2_driver"
    )

    scout_base_pkg = get_package_share_directory(
        "scout_base"
    )

    robot_desc_pkg = get_package_share_directory(
        "robot_description"
    )

    rm_nav_pkg = get_package_share_directory(
        "rm_navigation"
    )

    rm_loc_pkg = get_package_share_directory(
        "rm_localization"
    )

    # ============================================================
    # YDLIDAR
    # ============================================================

    ydlidar_launch = GroupAction(
        actions=[
            # This environment variable only exists for the scope of this GroupAction
            SetEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY', 'WARN'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ydlidar_pkg, "launch", "ydlidar_launch.py")
                ),
                launch_arguments={
                    "params_file": os.path.join(ydlidar_pkg, "params", "X4-Pro.yaml")
                }.items(),
            )
        ]
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
    # ROBOT DESCRIPTION
    # ============================================================

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_desc_pkg,
                "launch",
                "real_robot.launch.py"
            )
        )
    )

    # ============================================================
    # NAV2 / SLAM
    # ============================================================

    nav2_launch_file = os.path.join(
        rm_loc_pkg,
        "launch",
        "bringup_launch.py"
    )

    nav2_params_file = os.path.join(
        rm_nav_pkg,
        "config",
        "nav2_real_params.yaml"
    )

    map_yaml_file = os.path.join(
        rm_nav_pkg,
        "maps",
        "warehouse",
        "map_slam_v2.yaml"
    )

    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_launch_file
        ),
        launch_arguments={
            "use_sim_time": sim_arg,
            "params_file": nav2_params_file,
            "slam": "True",
            "map": "",
            "use_collision_monitor": "false",
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration("slam"),
                "' == 'true'"
            ])
        ),
    )

    nav2_localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            nav2_launch_file
        ),
        launch_arguments={
            "use_sim_time": sim_arg,
            "params_file": nav2_params_file,
            "slam": "False",
            "map": map_yaml_file,
            "localization": LaunchConfiguration(
                "localization"
            ),
            "use_collision_monitor": "false",
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration("slam"),
                "' == 'false'"
            ])
        ),
    )

    # ============================================================
    # RVIZ
    # ============================================================



        # ============================================================
    # INDOOR FAKE-GPS NAV2 PIPELINE
    # ============================================================

    indoor_nav_launch_file = os.path.join(
        rm_nav_pkg,
        "launch",
        "indoor_navigation.launch.py"
    )

    indoor_gps_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            indoor_nav_launch_file
        ),
        launch_arguments={
            "origin_latitude": "53.4668",
            "origin_longitude": "-2.2339",
        }.items(),
    )
    # ============================================================
    # LAUNCH SEQUENCE
    # ============================================================
#     lawnmower_interrupt_node = Node(
#        package="waypoint_navigation_pkg",
#        executable="lawnmower_interrupt_node",
#        name="lawnmower_interrupt_node",
#        output="screen",
#    )
#
#    fake_interrupt_test_node = Node(
#        package="waypoint_navigation_pkg",
#        executable="fake_interrupt_test_node",
#        name="fake_interrupt_test_node",
#        output="screen",
#    )
#
    leader_node = Node(
        package="waypoint_navigation_pkg",
        executable="leader_navigation_node_new",
        name="leader_navigation_node_new",
        output="screen",
    )

    fake_area_coords_server_node = Node(
        package="waypoint_navigation_pkg",
        executable="straight_line_waypoint_generator_node",
        name="straight_line_waypoint_generator_node",
        output="screen",
    )

    litter_handler_node = Node(
        package="waypoint_navigation_pkg",
        executable="litter_handler_node_new",
        name="litter_handler_node_new",
        output="screen",
    )

    new_litter_monitor = Node(
        package="waypoint_navigation_pkg",
        executable="new_litter_monitor_new",
        name="new_litter_monitor_new",
        output="screen",
    )

    return [

        ydlidar_launch,

        TimerAction(
            period=0.0,
            actions=[scout_base_launch],
        ),

        TimerAction(
            period=0.0,
            actions=[robot_description_launch],
        ),

        TimerAction(
            period=10.0,
            actions=[
                indoor_gps_nav_launch,
            ],
        ),

        # Start fake area service first
        TimerAction(
            period=0.0,
            actions=[fake_area_coords_server_node],
        ),

        # Then start leader, which polls /comms/area_coords
        TimerAction(
            period=0.0,
            actions=[leader_node],
        ),

        TimerAction(
            period=0.0,
            actions=[litter_handler_node],
        ),

        TimerAction(
            period=0.0,
            actions=[new_litter_monitor],
        ),
    ]