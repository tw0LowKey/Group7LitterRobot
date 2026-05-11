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
            DeclareLaunchArgument(name="slam", default_value="true", description="Launch SLAM or launch localization and navigation", choices=["True", "False", "true", "false"],),
            # Notice the default is now false for the real robot
            DeclareLaunchArgument(name="simulation", default_value="false", description="Launch simulation with gazebo or launch real robot navigation", choices=["True", "False", "true", "false"],),
            DeclareLaunchArgument(name="localization", default_value="amcl", description="Launch localization with AMCL algorithm or SLAM toolbox for localization only", choices=["amcl", "slam_toolbox"],),
            OpaqueFunction(function=launch_setup),
        ]
    )

def launch_setup(context, *args, **kwargs):
    sim_arg = LaunchConfiguration("simulation")

    nav2_launch_file = os.path.join(get_package_share_directory("rm_localization"), "launch", "bringup_launch.py")
    
    # CHANGED: Pointing to the real robot params file we just made
    nav2_params_file = os.path.join(get_package_share_directory("rm_navigation"), "config", "nav2_real_params.yaml")
    
    map_yaml_file = os.path.join(get_package_share_directory("rm_navigation"), "maps", "warehouse/map_slam_v2.yaml")
    
    # CHANGED: "use_sim_time" is now tied to the 'simulation' argument, not hardcoded to "true"
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file), 
        launch_arguments={
            "use_sim_time": sim_arg, 
            "params_file": nav2_params_file, 
            "slam": "True", 
            "map": "", 
            'use_collision_monitor': 'false',
        }.items(), 
        condition=IfCondition(PythonExpression(["'" , LaunchConfiguration("slam"), "' == 'true'"]))
    )
    
    # CHANGED: "use_sim_time" is now tied to the 'simulation' argument
    nav2_localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file), 
        launch_arguments={
            "use_sim_time": sim_arg, 
            "params_file": nav2_params_file, 
            "slam": "False", 
            "map": map_yaml_file, 
            "localization": LaunchConfiguration("localization")
        }.items(), 
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("slam") , "' == 'false'"]))
    )
    
    # DELETED: static_tf (world -> map). SLAM must be the only thing publishing to the map frame!

    rviz_default_config_file = os.path.join(get_package_share_directory("rm_navigation"), "rviz", "nav2.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_default_config_file],
    )
    
    # CHANGED: Removed static_tf from the return list
    return [nav2_slam_launch, nav2_localization_navigation_launch, rviz_node]