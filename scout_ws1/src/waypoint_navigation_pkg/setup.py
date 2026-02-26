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
        ],
    },
)
