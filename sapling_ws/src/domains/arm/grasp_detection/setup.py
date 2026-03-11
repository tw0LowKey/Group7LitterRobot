from setuptools import setup
import os
from glob import glob

package_name = 'grasp_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'open3d', 'scipy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Robust grasp detection node for multi-object picking',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'grasp_node = grasp_detection.grasp_node:main',
            'angle_node = grasp_detection.angle:main'
        ],
    },
)
