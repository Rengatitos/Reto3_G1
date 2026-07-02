#!/usr/bin/env python3
"""
box_detector_G1.py — PARTE A: "El Censo"

Detecta, cuenta y ubica cajas usando LaserScan 2D.

Pipeline completo:
    /scan  →  filtrar (inf/nan/rango/angulo)
           →  convertir a cartesiano (marco robot)
           →  clustering euclidiano 2D por distancia entre puntos consecutivos
           →  filtrar por ancho aparente ~ caja
           →  centroide (cx, cy) en marco robot
           →  radio inscrito = distancia minima centroide-punto (aproxima caja)
           →  componer con /odom (marco global)
           →  deduplicar censo
           →  publicar /cajas_G1 (PoseArray) + /cajas_markers_G1 (MarkerArray)
           →  imprimir plano 2D en consola cada N detecciones

Topicos publicados:
    /cajas_G1          geometry_msgs/PoseArray   — centroides en marco odom
    /cajas_markers_G1  visualization_msgs/MarkerArray — visualizacion RViz

Topicos suscritos:
    /scan   sensor_msgs/LaserScan
    /odom   nav_msgs/Odometry

ESAN — Robotica de Moviles 2026-I  |  Proyecto CapyTown — Grupo 1
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# — Tipos auxiliares ————————————————————————————————————————————————————
Punto2D = Tuple[float, float]   # (x, y) en metros


# ——————————————————————————————————————————————————————————————————————————
#  Funciones puras de procesamiento LiDAR
# ——————————————————————————————————————————————————————————————————————————

def yaw_desde_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extrae yaw desde quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def scan_a_cartesiano(msg: LaserScan,
                      angulo_min_excluir: float = -math.pi,
                      angulo_max_excluir: float = math.pi,
                      rango_max_proc: float = 5.0) -> List[Tuple[float, float, float]]:
    """Convierte LaserScan a lista de puntos 2D cartesianos con ángulo (marco robot).

    Descarta inf/nan, puntos fuera del rango físico y la zona ciega trasera
    definida por [angulo_min_excluir, angulo_max_excluir].

    Devuelve lista de (x, y, theta_normalizado).
    """
    puntos: List[Tuple[float, float, float]] = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r):
            continue
        if r < msg.range_min or r > min(msg.range_max, rango_max_proc):
            continue
        theta = msg.angle_min + i * msg.angle_increment
        # Normalizar a [-pi, pi]
        theta_n = math.atan2(math.sin(theta), math.cos(theta))
        # Excluir zona trasera si se configura
        if angulo_min_excluir <= angulo_max_excluir:
            if angulo_min_excluir <= theta_n <= angulo_max_excluir:
                continue
        puntos.append((r * math.cos(theta), r * math.sin(theta), theta_n))
    return puntos


def angular_span_cluster(cluster: List[Tuple[float, float, float]]) -> float:
    """Arco angular total del cluster (radianes).

    Una caja de ~20 cm a 1 m subtiende ~11° (~0.19 rad).
    Una pared larga subtiende decenas de grados (> 0.8 rad).
    Usar para discriminar cajas de maderas laterales.
    """
    if len(cluster) < 2:
        return 0.0
    angles = [p[2] for p in cluster]
    return abs(angles[-1] - angles[0])


def clustering_euclidiano(puntos: List[Tuple[float, float, float]],
                          umbral: float) -> List[List[Tuple[float, float, float]]]:
    """Agrupa puntos consecutivos cuya distancia euclidiana sea < umbral.

    Es equivalente a un single-linkage sobre puntos ordenados del barrido,
    lo que funciona bien para un LiDAR 2D.
    """
    if not puntos:
        return []
    clusters: List[List[Tuple[float, float, float]]] = [[puntos[0]]]
    for pt in puntos[1:]:
        px, py, _ = pt
        lx, ly, _ = clusters[-1][-1]
        if math.hypot(px - lx, py - ly) <= umbral:
            clusters[-1].append(pt)
        else:
            clusters.append([pt])
    return clusters


def centroide(cluster: List[Tuple[float, float, float]]) -> Punto2D:
    """Centroide (media aritmética) del cluster."""
    n = len(cluster)
    cx = sum(p[0] for p in cluster) / n
    cy = sum(p[1] for p in cluster) / n
    return cx, cy


def radio_inscrito(cluster: List[Tuple[float, float, float]], cx: float, cy: float) -> float:
    """Radio inscrito aproximado = distancia minima centroide-punto del cluster."""
    if not cluster:
        return 0.0
    return min(math.hypot(p[0] - cx, p[1] - cy) for p in cluster)


def ancho_cluster(cluster: List[Tuple[float, float, float]]) -> float:
    """Distancia entre los extremos del cluster (ancho aparente)."""
    if len(cluster) < 2:
        return 0.0
    return math.hypot(cluster[-1][0] - cluster[0][0],
                      cluster[-1][1] - cluster[0][1])


def componer_odom(px_robot: float, py_robot: float,
                  pose_x: float, pose_y: float, pose_yaw: float) -> Punto2D:
    """Transforma punto del marco robot al marco odom."""
    c = math.cos(pose_yaw)
    s = math.sin(pose_yaw)
    return (c * px_robot - s * py_robot + pose_x,
            s * px_robot + c * py_robot + pose_y)


# ——————————————————————————————————————————————————————————————————————————
#  Nodo principal
# ——————————————————————————————————————————————————————————————————————————

class BoxDetectorG1(Node):

    def __init__(self):
        super().__init__('box_detector_G1')

        # — Parámetros —————————————————————————————————————————————————
        self.declare_parameter('umbral_cluster',     0.12)   # m — salto que separa clusters
        self.declare_parameter('min_puntos',          4)     # puntos mínimos por cluster
        self.declare_parameter('ancho_caja',          0.20)  # m — lado nominal de la caja
        self.declare_parameter('tolerancia_ancho',    0.12)  # m — ±tolerancia ancho
        self.declare_parameter('rango_max_deteccion', 3.5)   # m — ignora cajas lejanas
        self.declare_parameter('dist_duplicado',      0.28)  # m — misma caja si < esto
        self.declare_parameter('imprimir_cada',        5)    # scans entre impresiones del plano
        self.declare_parameter('max_span_angular',    0.70)  # rad — máx arco para no ser pared (~40°)
        self.declare_parameter('min_span_angular',    0.04)  # rad — mín arco para no ser ruido (~2°)
        self.declare_parameter('min_dist_caja',       0.10)  # m — distancia mínima al centroide

        self._umb      = self.get_parameter('umbral_cluster').value
        self._min_pts  = int(self.get_parameter('min_puntos').value)
        self._ancho    = self.get_parameter('ancho_caja').value
        self._tol      = self.get_parameter('tolerancia_ancho').value
        self._rmax     = self.get_parameter('rango_max_deteccion').value
        self._dist_dup = self.get_parameter('dist_duplicado').value
        self._impr_c   = int(self.get_parameter('imprimir_cada').value)
        self._max_span = self.get_parameter('max_span_angular').value
        self._min_span = self.get_parameter('min_span_angular').value
        self._min_dist = self.get_parameter('min_dist_caja').value

        # — Estado ——————————————————————————————————————————————————————
        # pose robot en odom: (x, y, yaw)
        self._pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._tengo_odom = False
        # Censo: lista de (ox, oy, radio) en marco odom
        self._censo: List[Tuple[float, float, float]] = []
        self._scan_count = 0

        # — QoS para LiDAR ————————————————————————————————————————————
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        # — Suscriptores —————————————————————————————————————————————————
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos_sensor)
        self.create_subscription(Odometry,  '/odom', self._cb_odom, 10)

        # — Publicadores —————————————————————————————————————————————————
        self._pub_cajas   = self.create_publisher(PoseArray,   '/cajas_G1',          10)
        self._pub_cajas2  = self.create_publisher(PoseArray,   '/cajas_avistadas',   10)
        self._pub_markers = self.create_publisher(MarkerArray, '/cajas_markers_G1',  10)
        self._pub_markers2 = self.create_publisher(MarkerArray, '/cajas_markers',    10)

        self.get_logger().info(
            'box_detector_G1 listo | Esperando /scan y /odom …\n'
            f'  umbral_cluster={self._umb} m  ancho_caja={self._ancho}±{self._tol} m\n'
            f'  rango_max={self._rmax} m  dist_duplicado={self._dist_dup} m')

    # — Callbacks —————————————————————————————————————————————————————————

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_desde_quaternion(q.x, q.y, q.z, q.w)
        self._pose = (p.x, p.y, yaw)
        self._tengo_odom = True

    def _cb_scan(self, msg: LaserScan):
        if not self._tengo_odom:
            self.get_logger().warn('Sin odometría todavía, descartando scan.', once=True)
            return

        self._scan_count += 1
        pose_x, pose_y, pose_yaw = self._pose

        # 1) Convertir a cartesiano (solo sector frontal ±160° para ignorar trasero)
        puntos = scan_a_cartesiano(msg, rango_max_proc=self._rmax)
        if not puntos:
            return

        # 2) Clustering euclidiano 2D
        clusters = clustering_euclidiano(puntos, self._umb)

        # 3) Filtrar candidatos y calcular centroides + radio inscrito
        detecciones: List[Tuple[float, float, float]] = []  # (ox, oy, radio)

        for cl in clusters:
            if len(cl) < self._min_pts:
                continue

            # Filtro angular: paredes tienen arco grande, cajas arco pequeño
            span_ang = angular_span_cluster(cl)
            if span_ang > self._max_span:
                continue  # probablemente pared lateral
            if span_ang < self._min_span:
                continue  # ruido o punto suelto

            w = ancho_cluster(cl)
            if abs(w - self._ancho) > self._tol:
                continue

            cx, cy = centroide(cl)
            dist_robot = math.hypot(cx, cy)
            if dist_robot > self._rmax or dist_robot < self._min_dist:
                continue

            r_insc = radio_inscrito(cl, cx, cy)

            ox, oy = componer_odom(cx, cy, pose_x, pose_y, pose_yaw)
            detecciones.append((ox, oy, r_insc))

        # 4) Deduplicar y actualizar censo
        nuevas = 0
        for ox, oy, ri in detecciones:
            if not self._ya_censada(ox, oy):
                self._censo.append((ox, oy, ri))
                nuevas += 1
                self.get_logger().info(
                    f'  Caja #{len(self._censo):02d}  odom=({ox:.3f}, {oy:.3f}) m'
                    f'  radio_inscrito={ri:.3f} m')

        if nuevas:
            self.get_logger().info(
                f'[CENSO] Total cajas: {len(self._censo)}')

        # 5) Publicar y mostrar plano 2D en consola
        self._publicar(msg.header)
        if self._scan_count % self._impr_c == 0:
            self._imprimir_plano_2d()

    # — Helpers ——————————————————————————————————————————————————————————

    def _ya_censada(self, ox: float, oy: float) -> bool:
        for (cx, cy, _) in self._censo:
            if math.hypot(ox - cx, oy - cy) < self._dist_dup:
                return True
        return False

    def _publicar(self, header):
        """Publica PoseArray y MarkerArray con el censo actual."""
        # — PoseArray —
        pa = PoseArray()
        pa.header = header
        pa.header.frame_id = 'odom'
        for ox, oy, _ in self._censo:
            p = Pose()
            p.position.x = ox
            p.position.y = oy
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._pub_cajas.publish(pa)
        self._pub_cajas2.publish(pa)   # también en /cajas_avistadas

        # — MarkerArray —
        ma = MarkerArray()

        # Borrar marcadores previos
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.header = pa.header
        ma.markers.append(del_m)

        for idx, (ox, oy, ri) in enumerate(self._censo):
            # Círculo del radio inscrito
            circ = Marker()
            circ.header = pa.header
            circ.ns = 'radio_inscrito'
            circ.id = idx * 3
            circ.type = Marker.CYLINDER
            circ.action = Marker.ADD
            circ.pose.position.x = ox
            circ.pose.position.y = oy
            circ.pose.position.z = 0.01
            circ.pose.orientation.w = 1.0
            d = max(ri * 2.0, 0.04)
            circ.scale.x = d
            circ.scale.y = d
            circ.scale.z = 0.02
            circ.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.5)
            circ.lifetime.sec = 0
            ma.markers.append(circ)

            # Esfera centroide
            esf = Marker()
            esf.header = pa.header
            esf.ns = 'centroide'
            esf.id = idx * 3 + 1
            esf.type = Marker.SPHERE
            esf.action = Marker.ADD
            esf.pose.position.x = ox
            esf.pose.position.y = oy
            esf.pose.position.z = 0.05
            esf.pose.orientation.w = 1.0
            esf.scale.x = esf.scale.y = esf.scale.z = 0.08
            esf.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)
            esf.lifetime.sec = 0
            ma.markers.append(esf)

            # Texto con número de caja y coordenadas
            txt = Marker()
            txt.header = pa.header
            txt.ns = 'etiqueta'
            txt.id = idx * 3 + 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = ox
            txt.pose.position.y = oy
            txt.pose.position.z = 0.25
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.12
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'C{idx + 1:02d}\n({ox:.2f},{oy:.2f})\nr={ri:.2f}'
            txt.lifetime.sec = 0
            ma.markers.append(txt)

        self._pub_markers.publish(ma)
        self._pub_markers2.publish(ma)  # también en /cajas_markers

    def _imprimir_plano_2d(self):
        """Imprime en consola un plano ASCII 2D con las cajas censadas."""
        if not self._censo:
            return

        W, H = 40, 20           # caracteres del plano
        ESCALA = 0.15           # metros por celda

        # Centrar el plano en el robot (pose_x, pose_y)
        rx, ry, _ = self._pose
        ox0 = rx - (W // 2) * ESCALA
        oy0 = ry - (H // 2) * ESCALA

        grid = [['.' for _ in range(W)] for _ in range(H)]

        # Robot en el centro
        grid[H // 2][W // 2] = 'R'

        for i, (bx, by, _) in enumerate(self._censo):
            col = int((bx - ox0) / ESCALA)
            row = int((by - oy0) / ESCALA)
            row = H - 1 - row   # y crece hacia arriba
            if 0 <= row < H and 0 <= col < W:
                grid[row][col] = str((i + 1) % 10)

        lineas = [
            '',
            '┌' + '─' * W + '┐',
            f'│  PLANO 2D — CENSO  ({len(self._censo)} cajas)  │',
        ]
        for fila in grid:
            lineas.append('│' + ''.join(fila) + '│')
        lineas.append('└' + '─' * W + '┘')

        # Leyenda
        for i, (bx, by, ri) in enumerate(self._censo):
            lineas.append(
                f'  C{i+1:02d}: odom=({bx:+.3f}, {by:+.3f}) m  '
                f'r_inscrito={ri:.3f} m')
        lineas.append('')

        self.get_logger().info('\n'.join(lineas))


# ——————————————————————————————————————————————————————————————————————————
#  Entry point
# ——————————————————————————————————————————————————————————————————————————

def main(args=None):
    rclpy.init(args=args)
    nodo = BoxDetectorG1()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
