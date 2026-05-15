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

    return LaunchDescription([
        orbbec_launch,
    ])
