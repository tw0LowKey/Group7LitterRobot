from setuptools import find_packages
from setuptools import setup

setup(
    name='piper_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('piper_msgs', 'piper_msgs.*')),
)
