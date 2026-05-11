# display_xacro.launch.py (inside piper_description package)

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. FIND THE XACRO FILE
    package_share = get_package_share_directory('piper_description')
    xacro_file = os.path.join(package_share, 'urdf', 'piper_description.xacro') # <-- CHECK THIS PATH!

    # 2. PROCESS XACRO (Get the full URDF content as a string)
    # This uses the 'Command' substitution to execute 'xacro' at launch time.
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    # 3. ROBOT STATE PUBLISHER (Reads the URDF string and publishes the /robot_description topic)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 4. JOINT STATE PUBLISHER (Reads the /robot_description topic and starts publishing /joint_states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui', # Use _gui if you want the slider GUI
        name='joint_state_publisher',
        output='screen'
    )

    # 5. OPTIONAL: RViz2
    rviz_config_file = os.path.join(package_share, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
