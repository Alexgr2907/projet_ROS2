from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'projet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandre&Dimitri',
    maintainer_email='alexandre.guilletriconda@gmail.com',
    description='package projet ROS2 TurtleBot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = projet.vision_node:main',
            'control_node = projet.control_node:main',
            'lidar_node = projet.lidar_node:main',
        ],
    },
)
