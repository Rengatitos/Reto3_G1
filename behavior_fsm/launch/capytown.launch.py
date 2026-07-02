"""
capytown.launch.py
------------------
Lanzamiento MÍNIMO del reto "El Censo y el Guardián de las Cajas".

Solo arranca el monitor web (puerto 8080).
La Parte A (box_detector_G1) y la Parte B (behavior_fsm + wall_follower)
se activan desde la interfaz web mediante botones:

  [Iniciar escaneo]            → lanza Parte A (censa cajas con LiDAR)
  [Activar recorrido mejorado] → detiene Parte A, lanza Parte B (FSM guardián)
  [Guardar corrida]            → escribe SQLite + metricas_lidar.csv

    ros2 launch behavior_fsm capytown.launch.py

Interfaz web:
    http://<IP_ROBOT>:8080
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    monitor = Node(
        package='box_detector_G1',
        executable='web_monitor',
        name='web_monitor',
        output='screen',
        parameters=[{
            'port': 8080,
            'sector_frontal_deg': 45.0,
            'cajas_reales': 5,   # número real de cajas en la pista para métricas
        }],
    )

    return LaunchDescription([monitor])
