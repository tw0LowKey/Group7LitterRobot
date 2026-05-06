from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('waypoint_navigation_pkg/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matheusrb',
    maintainer_email='matheusrb@todo.todo',
    description='A ROS 2 package for waypoint-based navigation using Nav2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigation_node = waypoint_navigation_pkg.waypoint_navigation_node:main',
            'front_back_node = waypoint_navigation_pkg.front_back_node:main',
            'yaml_obstacle_publisher = waypoint_navigation_pkg.yaml_obstacle_publisher:main',
            'front_interrupt_node = waypoint_navigation_pkg.front_interrupt_node:main',
            'lawnmower_interrupt_node = waypoint_navigation_pkg.lawnmower_interrupt_node:main',
            'fake_litter_interrupt_node = waypoint_navigation_pkg.fake_litter_interrupt_node:main',
            'fake_litter_interrupt_2 = waypoint_navigation_pkg.fake_litter_interrupt_2:main',
            'gps_follower_behaviour_node = waypoint_navigation_pkg.gps_follower_behaviour_node:main',
        ],
    },
)