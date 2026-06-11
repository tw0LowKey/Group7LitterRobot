from setuptools import find_packages, setup

package_name = 'follower_nav2_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/gps_behavior_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keyang',
    maintainer_email='keyang@todo.todo',
    description='Leader-follower behavior using Nav2 NavigateToPose',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follower_coordination_node = follower_nav2_pkg.follower_coordination_node:main',
            'waypoint_generator_node = follower_nav2_pkg.waypoint_generator_node:main',
            'leader_navigation_node= follower_nav2_pkg.leader_navigation_node:main',
            'gps_follower_behavior_node = follower_nav2_pkg.gps_follower_behavior_node:main',
            'litter_handler_node = follower_nav2_pkg.litter_handler_node:main',
            'litter_trigger_node = follower_nav2_pkg.litter_trigger_node:main',

        ],
    },
)