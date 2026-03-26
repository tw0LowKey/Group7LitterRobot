from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('orbbec_camera'),
                         'launch', 'gemini_330_series.launch.py')
        ]),
        launch_arguments={'config_file_path': os.path.join(get_package_share_directory('computer_vision'),
                         'launch', 'orbbec_config.yaml')}.items()
    )

    cv_node = Node(
        package='computer_vision',
        executable='rgb_depth_node',
        name='rgb_depth_node',
        output='screen'
    )
    
    grasping_node = Node(
    	package = 'grasp_detection',
    	executable = 'grasp_node',
    	name = 'grasp_node',
    	output = 'screen',
    )

    filter_node = Node(
        package='computer_vision',
        executable='filter_node',
        name='filter_node',
        output='screen'
    )
    	
    
    

    return LaunchDescription([
        orbbec_launch,
        cv_node,
        grasping_node,
        filter_node
    ])
