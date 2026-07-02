#!/usr/bin/env python3
"""
lidar_nav_node.py — Navegacion autonoma de pasillo para Parte A (escaneo).

Estrategia:
  1) CENTRADO: control P+D sobre error lateral (right - left) para mantenerse
     al centro del jiron usando distancias LiDAR izquierda/derecha.
  2) EVASION: si el sector frontal detecta un obstaculo < d_safety, busca el
     hueco mas ancho dentro de un cono frontal y gira hacia ese lado.
  3) RESPALDO odometria:
     - Scan muy degradado — sigue recto a v_slow con ultimo angulo.
     - Atasco detectado (se manda avanzar pero odom no cambia) —
       maniobra de recuperacion (retrocede + gira).

Topicos:
  Sub:  /scan  (sensor_msgs/LaserScan)
        /odom  (nav_msgs/Odometry)
  Pub:  /cmd_vel (geometry_msgs/Twist)

ESAN — Robotica de Moviles 2026-I  |  CapyTown Grupo 1
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class LidarNavNode(Node):

    def __init__(self):
        super().__init__('lidar_nav_node')

        # — Parametros ————————————————————————————————————————————————————
        self.declare_parameter('v_nom',             0.15)
        self.declare_parameter('v_curve',           0.10)
        self.declare_parameter('v_slow',            0.08)
        self.declare_parameter('v_recovery',       -0.08)
        self.declare_parameter('kp',                1.2)
        self.declare_parameter('kd',                0.4)
        self.declare_parameter('w_max',             0.8)
        self.declare_parameter('w_curve_threshold', 0.35)
        self.declare_parameter('w_avoid',           0.6)
        self.declare_parameter('d_safety',          0.28)
        self.declare_parameter('d_side_max',        1.20)
        self.declare_parameter('min_valid_range',   0.02)
        self.declare_parameter('front_half_angle_deg',  15.0)
        self.declare_parameter('side_start_deg',        60.0)
        self.declare_parameter('side_end_deg',         120.0)
        self.declare_parameter('avoid_cone_deg',        70.0)
        self.declare_parameter('avoid_gap_step_deg',     5.0)
        self.declare_parameter('control_period',    0.05)
        self.declare_parameter('scan_invalid_ratio', 0.5)
        self.declare_parameter('scan_timeout',       0.5)
        self.declare_parameter('stuck_check_period', 1.5)
        self.declare_parameter('stuck_dist_threshold', 0.03)
        self.declare_parameter('recovery_duration',  1.2)

        p = self.get_parameter
        self.v_nom             = p('v_nom').value
        self.v_curve           = p('v_curve').value
        self.v_slow            = p('v_slow').value
        self.v_recovery        = p('v_recovery').value
        self.kp                = p('kp').value
        self.kd                = p('kd').value
        self.w_max             = p('w_max').value
        self.w_curve_thresh    = p('w_curve_threshold').value
        self.w_avoid           = p('w_avoid').value
        self.d_safety          = p('d_safety').value
        self.d_side_max        = p('d_side_max').value
        self.min_valid        = p('min_valid_range').value
        self.front_half       = math.radians(p('front_half_angle_deg').value)
        self.side_start       = math.radians(p('side_start_deg').value)
        self.side_end         = math.radians(p('side_end_deg').value)
        self.avoid_cone       = math.radians(p('avoid_cone_deg').value)
        self.avoid_step       = math.radians(p('avoid_gap_step_deg').value)
        self.ctrl_period      = p('control_period').value
        self.scan_inv_ratio   = p('scan_invalid_ratio').value
        self.scan_timeout     = p('scan_timeout').value
        self.stuck_period     = p('stuck_check_period').value
        self.stuck_thr        = p('stuck_dist_threshold').value
        self.recovery_dur     = p('recovery_duration').value

        # — Estado ————————————————————————————————————————————————————————
        self.last_scan      = None
        self.last_scan_t    = None
        self.error_prev     = 0.0
        self.last_cmd       = Twist()
        self.odom_pos       = None
        self.odom_chk_pos   = None
        self.odom_chk_t     = None
        self.recovering_until = None

        # — ROS ——————————————————————————————————————————————————————————
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_timer(self.ctrl_period, self._loop)

        self.get_logger().info(
            f'lidar_nav_node listo  '
            f'd_safety={self.d_safety} m  kp={self.kp}  kd={self.kd}')

    # — Callbacks ——————————————————————————————————————————————————————————

    def _cb_scan(self, msg: LaserScan):
        self.last_scan   = msg
        self.last_scan_t = time.time()

    def _cb_odom(self, msg: Odometry):
        self.odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.odom_chk_pos is None:
            self.odom_chk_pos = self.odom_pos
            self.odom_chk_t   = time.time()

    # — Helpers LiDAR ——————————————————————————————————————————————————————

    def _sector_min(self, msg: LaserScan, a0: float, a1: float,
                    default: float = float('inf')) -> float:
        """Minimo de distancias validas en el arco [a0, a1] (rad, simetrico al frente=0)."""
        best = default
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            if a0 <= ang <= a1 and math.isfinite(r) and r > self.min_valid:
                best = min(best, r)
        return best

    def _validity(self, msg: LaserScan) -> float:
        n = len(msg.ranges)
        if n == 0:
            return 0.0
        ok = sum(1 for r in msg.ranges if math.isfinite(r) and r > self.min_valid)
        return ok / n

    def _best_gap(self, msg: LaserScan) -> float:
        """Angulo (rad) del hueco mas ancho dentro del cono frontal ±avoid_cone."""
        best_ang, best_score = 0.0, -1.0
        ang = -self.avoid_cone
        while ang <= self.avoid_cone:
            vals = []
            for i, r in enumerate(msg.ranges):
                a = msg.angle_min + i * msg.angle_increment
                if abs(a - ang) <= self.avoid_step and math.isfinite(r) and r > self.min_valid:
                    vals.append(r)
            score = min(vals) if vals else 0.0
            if score > best_score:
                best_score = score
                best_ang = ang
            ang += self.avoid_step
        return best_ang

    # — Recuperacion por atasco ——————————————————————————————————————————

    def _check_stuck(self, now: float, moving_forward: bool):
        if self.odom_chk_t is None or now - self.odom_chk_t < self.stuck_period:
            return
        if self.odom_pos and self.odom_chk_pos and moving_forward:
            dist = math.hypot(self.odom_pos[0] - self.odom_chk_pos[0],
                              self.odom_pos[1] - self.odom_chk_pos[1])
            if dist < self.stuck_thr:
                self.get_logger().warn('Atasco detectado — recuperacion')
                self.recovering_until = now + self.recovery_dur
        self.odom_chk_pos = self.odom_pos
        self.odom_chk_t   = now

    def _recovery_cmd(self, now: float) -> Twist:
        cmd = Twist()
        half = self.recovery_dur / 2.0
        remaining = self.recovering_until - now
        if remaining > half:                # Fase 1: retroceder
            cmd.linear.x  = self.v_recovery
            cmd.angular.z = 0.0
        else:                               # Fase 2: girar izquierda
            cmd.linear.x  = 0.0
            cmd.angular.z = self.w_avoid
        if now >= self.recovering_until:
            self.recovering_until = None
        return cmd

    # — Bucle de control ——————————————————————————————————————————————————

    def _loop(self):
        now = time.time()

        # Recuperacion con prioridad total
        if self.recovering_until is not None:
            cmd = self._recovery_cmd(now)
            self.pub_cmd.publish(cmd)
            self.last_cmd = cmd
            return

        # Sin scan — detener
        if self.last_scan is None or (now - self.last_scan_t) > self.scan_timeout:
            self.pub_cmd.publish(Twist())
            return

        msg = self.last_scan

        # Scan muy degradado — respaldo: recto lento
        if self._validity(msg) < (1.0 - self.scan_inv_ratio):
            cmd = Twist()
            cmd.linear.x  = self.v_slow
            cmd.angular.z = self.last_cmd.angular.z * 0.5
            self.pub_cmd.publish(cmd)
            self.last_cmd = cmd
            self._check_stuck(now, True)
            return

        # — Sectores ————————————————————————————————————————————————
        front = self._sector_min(msg, -self.front_half, self.front_half)
        left  = min(self._sector_min(msg,  self.side_start,  self.side_end,  self.d_side_max),
                    self.d_side_max)
        right = min(self._sector_min(msg, -self.side_end,   -self.side_start, self.d_side_max),
                    self.d_side_max)

        cmd = Twist()

        if front < self.d_safety:
            # — Modo EVASION ————————————————————————————————————————
            gap_ang = self._best_gap(msg)
            cmd.linear.x  = self.v_slow
            if gap_ang != 0.0:
                cmd.angular.z = _clamp(
                    math.copysign(self.w_avoid, gap_ang), -self.w_max, self.w_max)
            else:
                # sin hueco claro — girar hacia el lado mas abierto
                cmd.angular.z = self.w_avoid if left > right else -self.w_avoid
        else:
            # — Modo CENTRADO P+D ——————————————————————————————————
            # error > 0 — mas espacio derecha — girar derecha (w negativo)
            error  = right - left
            d_err  = (error - self.error_prev) / self.ctrl_period
            self.error_prev = error
            w = _clamp(-(self.kp * error + self.kd * d_err), -self.w_max, self.w_max)
            cmd.linear.x  = self.v_curve if abs(w) > self.w_curve_thresh else self.v_nom
            cmd.angular.z = w

        self.pub_cmd.publish(cmd)
        self.last_cmd = cmd
        self._check_stuck(now, cmd.linear.x > 0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())   # parar al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
