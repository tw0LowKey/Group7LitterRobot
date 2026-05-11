from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lidar_params = LaunchConfiguration("lidar_params")

    ydlidar_pkg = get_package_share_directory("ydlidar_ros2_driver")
    scout_base_pkg = get_package_share_directory("scout_base")
    robot_description_pkg = get_package_share_directory("robot_description")
    rm_navigation_pkg = get_package_share_directory("rm_navigation")

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_params",
            default_value=[
                EnvironmentVariable("HOME"),
                "/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml"
            ],
            description="Path to YDLIDAR parameter file"
        ),

        # 1. LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ydlidar_pkg, "launch", "ydlidar_launch.py"])
            ),
            launch_arguments={
                "params_file": lidar_params
            }.items()
        ),

        # 2. Scout base - delayed slightly so CAN is already up
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([scout_base_pkg, "launch", "scout_mini_base.launch.py"])
                    )
                )
            ]
        ),

        # 3. Robot description / real robot
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([robot_description_pkg, "launch", "real_robot.launch.py"])
                    )
                )
            ]
        ),

        # 4. Navigation - delayed more so TF/odom/scan have time to come up
        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([rm_navigation_pkg, "launch", "real_robot_navigation.launch.py"])
                    )
                )
            ]
        ),
    ])