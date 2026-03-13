from setuptools import setup
import os
from glob import glob

package_name = 'grasp_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'open3d', 'scipy'],
    zip_safe=True,
    maintainer='Ben Kirk',
    maintainer_email='ben.kirk.bk@gmail.com',
    description='Grasp Detection Node',
    license='No License',
    entry_points={
        'console_scripts': [
            'grasp_node = grasp_detection.grasp_node:main',
            'angle_node = grasp_detection.angle:main'
        ],
    },
)
