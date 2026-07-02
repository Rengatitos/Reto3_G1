import os
from glob import glob
from setuptools import setup

package_name = 'box_detector_G1'

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
    description='PARTE A El Censo: detector de cajas con LiDAR 2D, clustering euclidiano y visualizacion 2D.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'box_detector_G1 = box_detector_G1.box_detector_G1:main',
            'web_monitor     = box_detector_G1.web_monitor:main',
        ],
    },
)
