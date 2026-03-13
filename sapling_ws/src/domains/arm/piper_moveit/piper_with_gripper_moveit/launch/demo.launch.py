from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"publish_rate": 200}]
    )

    exclusion_generator = Node(
        package="add_zones_pkg",
        executable="add_zones_node"
    )

    pick_and_place = Node(
        package="pick_and_place_python",
        executable="pick_and_place_client"
    )

    move_to_home_server = Node(
        package="pick_and_place_pkg",
        executable="move_to_home_server"
    )

    move_to_pose_server = Node(
        package="pick_and_place_pkg",
        executable="move_to_pose_server"
    )

    set_grip_width_server = Node(
        package="pick_and_place_pkg",
        executable="set_grip_width_server"
    )



    return LaunchDescription(
        generate_demo_launch(moveit_config).entities + [
            exclusion_generator,
            pick_and_place,
            set_grip_width_server,
            move_to_pose_server,
            move_to_home_server,
        ]
    )
