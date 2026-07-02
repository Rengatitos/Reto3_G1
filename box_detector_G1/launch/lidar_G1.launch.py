"""
lidar_G1.launch.py
------------------
PARTE A – "El Censo"
Lanza el nodo box_detector_G1 (detector de cajas con LiDAR 2D).

Uso:
    ros2 launch box_detector_G1 lidar_G1.launch.py
    ros2 launch box_detector_G1 lidar_G1.launch.py rango_max_deteccion:=4.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('box_detector_G1')
    params_file = os.path.join(pkg_share, 'config', 'params_G1.yaml')

    # Argumento opcional para sobreescribir el rango máximo
    rango_arg = DeclareLaunchArgument(
        'rango_max_deteccion',
        default_value='3.5',
        description='Distancia máxima de detección en metros')

    detector = Node(
        package='box_detector_G1',
        executable='box_detector_G1',
        name='box_detector_G1',
        output='screen',
        parameters=[
            params_file,
            {'rango_max_deteccion': LaunchConfiguration('rango_max_deteccion')},
        ],
    )

    return LaunchDescription([
        rango_arg,
        detector,
    ])
