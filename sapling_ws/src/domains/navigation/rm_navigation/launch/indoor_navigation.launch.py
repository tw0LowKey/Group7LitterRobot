import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rm_localization_dir = get_package_share_directory("rm_localization")
    rm_navigation_dir = get_package_share_directory("rm_navigation")

    outdoor_localization_launch = os.path.join(
        rm_localization_dir,
        "launch",
        "outdoor_localization_launch.py"
    )

    navigation_launch = os.path.join(
        rm_localization_dir,
        "launch",
        "navigation_launch.py"
    )

    nav2_params_file = os.path.join(
        rm_navigation_dir,
        "config",
        "nav2_outdoor_params.yaml"
    )

    rviz_default_config_file = os.path.join(
        rm_navigation_dir,
        "rviz",
        "nav2.rviz"
    )

    declare_origin_lat_cmd = DeclareLaunchArgument(
        "origin_latitude",
        default_value="53.4668",
        description="Fake GPS origin latitude"
    )

    declare_origin_lon_cmd = DeclareLaunchArgument(
        "origin_longitude",
        default_value="-2.2339",
        description="Fake GPS origin longitude"
    )

    fake_gps_node = Node(
        package="rm_localization",
        executable="fake_gps_from_odom.py",
        name="fake_gps_from_odom",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "odom_topic": "/odom",
            "gps_topic": "/gps/fix",
            "origin_latitude": LaunchConfiguration("origin_latitude"),
            "origin_longitude": LaunchConfiguration("origin_longitude"),
            "origin_altitude": 50.0,
            "noise_std_m": 0.0,
        }]
    )

    outdoor_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(outdoor_localization_launch),
        launch_arguments={
            "gps_topic": "/gps/fix",
        }.items()
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
            "autostart": "true",
            "use_composition": "False",
        }.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_default_config_file],
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([
        declare_origin_lat_cmd,
        declare_origin_lon_cmd,
        fake_gps_node,
        outdoor_localization,
        nav2_navigation,
        rviz_node,
    ])