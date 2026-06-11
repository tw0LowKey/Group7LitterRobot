from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('follower_nav2_pkg')
    

    waypoint_generator = Node(
        package='follower_nav2_pkg',
        executable='waypoint_generator_node',
        name='waypoint_generator_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'gps_topic': '/navsat',
            'waypoint_topic': '/waypoints',
            'strip_width': 1.35,
            'goal_frame': 'map',
            'auto_ref': True,
    }],
    )

    leader_navigation = Node(
        package='follower_nav2_pkg',
        executable='leader_navigation_node',
        name='leader_navigation_node',
        output='screen',
        parameters=[{
        'use_sim_time': True,
        'leader_action': '/navigate_to_pose',
        'leader_odom_topic': '/odom',
        'waypoint_topic': '/waypoints',
        'pause_nav_topic': '/pause_navigation',
        'resume_nav_topic': '/resume_navigation',
        'goal_frame': 'map',
    }],
    )

    follower_coordination = Node(
        package='follower_nav2_pkg',
        executable='follower_coordination_node',
        name='follower_coordination_node',
        output='screen',
        parameters=[{
            'sweep_done_topic': '/sweep_done',
            'odom_topic': '/odom',
            'gps_topic': '/navsat',
            'call_bin_topic': '/call_bin',
            'leader_pose_tx_topic': '/comms/leader_to_follower_tx',
            'follower_status_rx_topic': '/comms/follower_to_leader_tx',
            'start_nav_topic': '/start_navigation',
            'follower_status_echo_topic': '/follower_status',
            'use_sim_time': True,
            'pickup_wait_time': 2.0,
        }],
    )

    gps_follower_behavior = Node(
    package='follower_nav2_pkg',
    executable='gps_follower_behavior_node',
    name='gps_follower_behavior_node',
    output='screen',
    parameters=[{'use_sim_time': True}],
)

    litter_handler = Node(
    package='follower_nav2_pkg',
    executable='litter_handler_node',
    name='litter_handler_node',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'leader_action': '/navigate_to_pose',
        'odom_topic': '/odom',
        'litter_topic': '/vision/detected_litter',
        'pause_nav_topic': '/pause_navigation',
        'resume_nav_topic': '/resume_navigation',
        'call_bin_topic': '/call_bin',
        'start_nav_topic': '/start_navigation',
        'goal_frame': 'map',
        'approach_offset': 0.3,
    }],
)
    trigger = Node(
    package="follower_nav2_pkg",
    executable="litter_trigger_node",
    name="litter_trigger_node",
    output="screen",
    parameters=[
        {"trigger_radius": 0.5},
    ],
)
    

    return LaunchDescription([
    waypoint_generator,
    leader_navigation,
    follower_coordination,
    litter_handler,
    gps_follower_behavior,
    trigger,
])