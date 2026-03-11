from setuptools import find_packages, setup

package_name = 'computer_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/models', ['models/Model_1_x2.engine']),
    ('share/' + package_name + '/launch', ['launch/vision_start.launch.py']),
    ('share/' + package_name + '/launch', ['launch/orbbec_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group7',
    maintainer_email='Thierry_popat@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgb_depth_node = computer_vision.rgb_depth_node:main',
            'ground_plane_node = computer_vision.ground_plane_node:main'
        ],
    },
)
