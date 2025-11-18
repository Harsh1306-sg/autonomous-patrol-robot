from setuptools import setup
import os
from glob import glob

package_name = 'patrol_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), 
            glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@todo.todo',
    description='ROS2 Patrol Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_robot = patrol_robot_pkg.patrol_robot_node:main',
        ],
    },
)
