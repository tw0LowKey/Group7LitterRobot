import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'sapling_interfaces'],
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
            'sapling_comms_node = comms.sapling_comms_node:main',
            'sapling_executor_node = comms.sapling_executor_node:main',
            'sapling_area_coords_test_node = comms.sapling_area_coords_test_node:main',
        ],
    },
)
