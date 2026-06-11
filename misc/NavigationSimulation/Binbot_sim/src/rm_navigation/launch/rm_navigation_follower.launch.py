from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="slam", default_value="true", choices=["True", "False", "true", "false"]),
            DeclareLaunchArgument(name="simulation", default_value="true", choices=["True", "False", "true", "false"]),
            DeclareLaunchArgument(name="localization", default_value="amcl", choices=["amcl", "slam_toolbox"]),
            OpaqueFunction(function=launch_setup),
        ]
    )

def launch_setup(context, *args, **kwargs):
    nav2_launch_file = os.path.join(get_package_share_directory("rm_localization_custom"), "launch", "bringup_launch.py")
    nav2_params_file = os.path.join(get_package_share_directory("rm_navigation"), "config", "nav2_params_follower.yaml")
    map_yaml_file = os.path.join(get_package_share_directory("rm_navigation"), "maps", "warehouse/map_slam_v2.yaml")

    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params_file,
            "slam": "True",
            "map": "",
            "use_collision_monitor": "false",
            "namespace": "follower",
            "use_namespace": "true",
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("slam"), "' == 'true'"]))
    )

    nav2_localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params_file,
            "slam": "False",
            "map": "",
            "localization": LaunchConfiguration("localization"),
            "namespace": "follower",
            "use_namespace": "true",
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("slam"), "' == 'false'"]))
    )

    static_tf_follower = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="follower",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "map",
                   "--child-frame-id", "follower/odom"],
    )

    static_tf_follower_global = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_follower_global",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "map",
                   "--child-frame-id", "follower/odom"],
    )
    # No RViz — follower displays are in the leader's nav2.rviz
    # No static_tf — leader launch already publishes world -> map
    static_tf_follower = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "map",
                   "--child-frame-id", "follower/odom"],
    )

    return [nav2_slam_launch, nav2_localization_navigation_launch, static_tf_follower, static_tf_follower_global]