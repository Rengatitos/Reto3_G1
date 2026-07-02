import os
from glob import glob
from setuptools import setup

package_name = 'behavior_fsm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codeplai',
    maintainer_email='codeplaigamessac@gmail.com',
    description='Parte B del reto CapyTown: guardian con FSM reactiva al LiDAR.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_fsm   = behavior_fsm.behavior_fsm:main',
            'wall_follower  = behavior_fsm.wall_follower:main',
            'lidar_nav_node = behavior_fsm.lidar_nav_node:main',
        ],
    },
)
