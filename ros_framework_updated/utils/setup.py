from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_framework'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/framework.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='achille.chiuchiarelli@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "broadcaster_node = ros_framework.transforms:main",
            "geometry_node = ros_framework.geometry_based_node:main",
            "appearance_node = ros_framework.appearance_based_node:main",
            "filter_node1 = ros_framework.filter1:main",
            "filter_node2 = ros_framework.filter2:main",
            "final_node = ros_framework.final_node:main",
            "publisher_node = ros_framework.publisher_node:main",
            "filter_trial = ros_framework.filter_test:main",
            "map_converter_node = ros_framework.map_converter:main",
            
        ],
    },
)
