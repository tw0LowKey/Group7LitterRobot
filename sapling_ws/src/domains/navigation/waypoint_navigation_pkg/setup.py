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
            'nav_with_litter_offset = waypoint_navigation_pkg.nav_with_litter_offset:main',
            'fake_litter_interrupt_node = waypoint_navigation_pkg.fake_litter_interrupt_node:main',
            'lawnmower_litter_nav = waypoint_navigation_pkg.lawnmower_litter_nav:main',
            'new_litter_monitor = waypoint_navigation_pkg.new_litter_monitor:main',
            'simplecentroidprinter = waypoint_navigation_pkg.simplecentroidprinter:main',
            'keyang_leader = waypoint_navigation_pkg.keyang_leader:main',
            'gps_follower_behaviour_node = waypoint_navigation_pkg.gps_follower_behaviour_node:main',
        ],
    },
)
