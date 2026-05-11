from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'driver_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajayiyer2397',
    maintainer_email='ajayiyer2397@tamu.edu',
    description='OSM-based driver assistance system with ROS2 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_assistant_node = driver_assistant.driver_assistant_node:main',
            'csv_mapviz_player = driver_assistant.csv_mapviz_player_node:main',
        ],
    },
)
