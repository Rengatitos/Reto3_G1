"""
monitor.launch.py
-----------------
Lanza el detector de cajas G1 + el monitor web en tiempo real.

    ros2 launch box_detector_G1 monitor.launch.py

Abre en el navegador:
    http://<IP_ROBOT>:8080
    http://localhost:8080   (si estas en el mismo equipo)

Argumentos:
    port         Puerto HTTP del monitor   (default: 8080)
    rango_max    Rango maximo de deteccion (default: 3.5 m)
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

    port_arg = DeclareLaunchArgument(
        'port', default_value='8080',
        description='Puerto HTTP del monitor web')

    rango_arg = DeclareLaunchArgument(
        'rango_max', default_value='3.5',
        description='Rango maximo de deteccion en metros')

    detector = Node(
        package='box_detector_G1',
        executable='box_detector_G1',
        name='box_detector_G1',
        output='screen',
        parameters=[
            params_file,
            {'rango_max_deteccion': LaunchConfiguration('rango_max')},
        ],
    )

    monitor = Node(
        package='box_detector_G1',
        executable='web_monitor',
        name='web_monitor',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
        ],
    )

    return LaunchDescription([
        port_arg,
        rango_arg,
        detector,
        monitor,
    ])
