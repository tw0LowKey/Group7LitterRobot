import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction

def generate_launch_description():

    rm_localization_dir = get_package_share_directory("rm_localization")
    rm_navigation_dir = get_package_share_directory("rm_navigation")

    # ============================================================
    # Localization launch (EKF + navsat_transform)
    # ============================================================

    outdoor_localization_launch = os.path.join(
        rm_localization_dir,
        "launch",
        "outdoor_localization_launch.py"
    )

    # ============================================================
    # Nav2 launch
    # ============================================================

    navigation_launch = os.path.join(
        rm_localization_dir,
        "launch",
        "navigation_launch.py"
    )

    # ============================================================
    # Nav2 params
    # ============================================================

    nav2_params_file = os.path.join(
        rm_navigation_dir,
        "config",
        "nav2_outdoor_params.yaml"
    )

    # ============================================================
    # RViz config
    # ============================================================

    rviz_default_config_file = os.path.join(
        rm_navigation_dir,
        "rviz",
        "nav2.rviz"
    )

    # ============================================================
    # Outdoor Localization
    # Assumes REAL GPS already publishing on /gps/fix
    # ============================================================

    outdoor_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            outdoor_localization_launch
        ),
        launch_arguments={
            "gps_topic": "/gps/fix",
        }.items()
    )

    gps_static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="gps_static_tf",
    arguments=[
        "-0.11",   # x
        "0.14",  # y
        "0.37",   # z
        "0",      # roll
        "0",      # pitch
        "0",      # yaw
        "base_footprint",
        "gps_link",
    ],
    output="screen",
    )
    # ============================================================
    # Nav2
    # ============================================================

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_launch
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
            "autostart": "true",
            "use_composition": "False",
        }.items()
    )

    # ============================================================
    # RViz
    # ============================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_default_config_file],
        parameters=[{"use_sim_time": False}],
    )

    fake_fixed_gps_node = Node(
        package="rm_localization",   # change this to the package where you put the file
        executable="fake_fixed_gps_node.py",
        name="fake_fixed_gps_node",
        output="screen",
        parameters=[
            {"latitude": 53.46680000},
            {"longitude": -2.23390000},
            {"altitude": 50.0},
            {"frame_id": "gps_link"},
            {"publish_rate": 5.0},],
    )
    # ============================================================
    # Launch
    # ============================================================



    return LaunchDescription([

        gps_static_tf,

        # fake_fixed_gps_node,

        outdoor_localization,

        TimerAction(
            period=5.0,
            actions=[nav2_navigation],
        ),

        TimerAction(
            period=6.0,
            actions=[rviz_node],
        ),

    ])