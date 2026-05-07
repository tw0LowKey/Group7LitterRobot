# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.actions import OpaqueFunction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch.conditions import IfCondition
# import os

# def generate_launch_description():
#     return LaunchDescription(
#         [
#             DeclareLaunchArgument(name="slam", default_value="true", description="Launch SLAM or launch localization and navigation", choices=["True", "False", "true", "false"],),
#             # Notice the default is now false for the real robot
#             DeclareLaunchArgument(name="simulation", default_value="false", description="Launch simulation with gazebo or launch real robot navigation", choices=["True", "False", "true", "false"],),
#             DeclareLaunchArgument(name="localization", default_value="amcl", description="Launch localization with AMCL algorithm or SLAM toolbox for localization only", choices=["amcl", "slam_toolbox"],),
#             OpaqueFunction(function=launch_setup),
#         ]
#     )

# def launch_setup(context, *args, **kwargs):
#     sim_arg = LaunchConfiguration("simulation")

#     nav2_launch_file = os.path.join(get_package_share_directory("rm_localization_custom"), "launch", "bringup_launch.py")
    
#     # CHANGED: Pointing to the real robot params file we just made
#     nav2_params_file = os.path.join(get_package_share_directory("rm_navigation"), "config", "nav2_real_params.yaml")
    
#     map_yaml_file = os.path.join(get_package_share_directory("rm_navigation"), "maps", "warehouse/map_slam_v2.yaml")
    
#     # CHANGED: "use_sim_time" is now tied to the 'simulation' argument, not hardcoded to "true"
#     nav2_slam_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(nav2_launch_file), 
#         launch_arguments={
#             "use_sim_time": sim_arg, 
#             "params_file": nav2_params_file, 
#             "slam": "True", 
#             "map": "", 
#             'use_collision_monitor': 'false',
#         }.items(), 
#         condition=IfCondition(PythonExpression(["'" , LaunchConfiguration("slam"), "' == 'true'"]))
#     )
    
#     # CHANGED: "use_sim_time" is now tied to the 'simulation' argument
#     nav2_localization_navigation_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(nav2_launch_file), 
#         launch_arguments={
#             "use_sim_time": sim_arg, 
#             "params_file": nav2_params_file, 
#             "slam": "False", 
#             "map": map_yaml_file, 
#             "localization": LaunchConfiguration("localization")
#         }.items(), 
#         condition=IfCondition(PythonExpression(["'", LaunchConfiguration("slam") , "' == 'false'"]))
#     )
    
#     # DELETED: static_tf (world -> map). SLAM must be the only thing publishing to the map frame!

#     rviz_default_config_file = os.path.join(get_package_share_directory("rm_navigation"), "rviz", "nav2.rviz")
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="screen",
#         arguments=["-d", rviz_default_config_file],
#     )
    
#     # CHANGED: Removed static_tf from the return list
#     return [nav2_slam_launch, nav2_localization_navigation_launch, rviz_node]

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
        "rm_localization_custom"
    )

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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                rm_nav_pkg,
                "rviz",
                "nav2.rviz"
            )
        ],
    )

    # ============================================================
    # LAUNCH SEQUENCE
    # ============================================================

    return [

        # --------------------------------------------------------
        # 0 sec : YDLIDAR
        # --------------------------------------------------------

        ydlidar_launch,

        # --------------------------------------------------------
        # 5 sec : CAN UP
        # --------------------------------------------------------

        TimerAction(
            period=5.0,
            actions=[can_setup],
        ),

        # --------------------------------------------------------
        # After CAN finishes + 5 sec : SCOUT BASE
        # --------------------------------------------------------

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

        # --------------------------------------------------------
        # 15 sec : ROBOT DESCRIPTION
        # --------------------------------------------------------

        TimerAction(
            period=15.0,
            actions=[robot_description_launch],
        ),

        # --------------------------------------------------------
        # 20 sec : NAV2 + RVIZ
        # --------------------------------------------------------

        TimerAction(
            period=20.0,
            actions=[
                nav2_slam_launch,
                nav2_localization_navigation_launch,
                rviz_node,
            ],
        ),
    ]