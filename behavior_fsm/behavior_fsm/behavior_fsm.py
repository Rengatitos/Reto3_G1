#!/usr/bin/env python3
"""
behavior_fsm.py — PARTE B: El Guardian (v3)

FSM NORMAL (por defecto):
    CRUCERO -> CAJA_DETECTADA -> PARAR -> ESPERAR_3S -> RODEAR -> CRUCERO

    Maniobra de rodeo:
        fase 0: girar +angulo_rodeo_deg (izquierda)
        fase 1: avanzar avance_rodeo_seg segundos
        fase 2: girar -angulo_rodeo_deg (derecha, reincorporarse)
        fase 3: rodeo completado -> CRUCERO

FSM MEJORADA (/enhanced_mode True):
    CRUCERO_M -> PARAR_M -> {ESQUINA_M | EVADIR_M} -> CRUCERO_M

    Al llegar a PARAR_M, el ancho angular del arco frontal (ver cb_scan)
    distingue CAJA (arco angosto) de ESQUINA (arco ancho):
        ESQUINA_M — retrocede un poco (corner_backup_seg) para no rozar la
                    pared/esquina con la parte trasera del chasis, luego
                    pivota ~90 grados (por odometria/yaw, o por tiempo si
                    /odom aun no publico) — reincorpora al tramo siguiente.
        EVADIR_M  — gira fijo a la izquierda hasta despejar el frente,
                    para esquivar la caja.
    En ambos casos, al volver a CRUCERO_M, wall_follower reengancha la
    distancia con la pared derecha.

Ambas FSM siguen la pared DERECHA como referencia (wall_follower.py) y,
al detectar una caja, rodean hacia la izquierda (se alejan de la pared)
para luego reincorporarse a la derecha. Las transiciones clave usan
persistencia de N lecturas /scan consecutivas (persist_frames) para no
reaccionar a un solo frame ruidoso.

Control via ROS2:
    /scan_active   (Bool) — habilita/deshabilita movimiento
    /enhanced_mode (Bool) — conmuta FSM normal <-> mejorada

Publica:
    /cmd_vel       geometry_msgs/Twist
    /fsm_state     std_msgs/String
    /parada_dist   std_msgs/Float32
    /fsm_metrics   std_msgs/String (JSON)

ESAN - Robotica de Moviles 2026-I | Proyecto CapyTown Grupo 1
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

# — Estados FSM Normal ————————————————————————————————————————————————————
CRUCERO        = 'CRUCERO'
CAJA_DETECTADA = 'CAJA_DETECTADA'
PARAR          = 'PARAR'
ESPERAR_3S     = 'ESPERAR_3S'
RODEAR         = 'RODEAR'

# — Estados FSM Mejorada ————————————————————————————————————————————————————
CRUCERO_M = 'CRUCERO_M'
PARAR_M   = 'PARAR_M'
EVADIR_M  = 'EVADIR_M'
ESQUINA_M = 'ESQUINA_M'


class BehaviorFSM(Node):

    def __init__(self):
        super().__init__('behavior_fsm')

        # — Parametros ————————————————————————————————————————————————
        self.declare_parameter('sector_frontal_deg',  45.0)
        self.declare_parameter('dist_alerta',          0.30)
        self.declare_parameter('dist_parada',          0.18)
        self.declare_parameter('dist_colision',        0.05)
        self.declare_parameter('vel_crucero',          0.15)
        self.declare_parameter('vel_precaucion',       0.07)
        self.declare_parameter('vel_giro',             0.50)
        self.declare_parameter('vel_min',              0.05)
        self.declare_parameter('espera_seg',           3.0)
        self.declare_parameter('angulo_rodeo_deg',     30.0)
        self.declare_parameter('avance_rodeo_seg',     2.0)
        self.declare_parameter('t_evasion_max',        12.0)
        # — Clasificacion frontal CAJA vs ESQUINA (arco angular) —————————
        self.declare_parameter('front_wall_deg',       45.0)   # ancho arco frontal >= esto -> ESQUINA
        self.declare_parameter('front_box_deg',        30.0)   # ancho arco frontal <= esto -> CAJA
        self.declare_parameter('front_arc_radial',     0.10)   # m — tolerancia radial para agrupar el arco
        self.declare_parameter('side_clear_dist',      0.60)   # m — izq despejada > esto desempata ambiguo -> ESQUINA
        self.declare_parameter('persist_frames',       4)      # N lecturas /scan consecutivas para confirmar transicion
        self.declare_parameter('corner_yaw_tol_deg',   3.5)    # grados — tolerancia para el giro de esquina por odometria
        self.declare_parameter('corner_turn_time',     3.2)    # s — respaldo si /odom aun no publico yaw (~90 deg a vel_giro)
        self.declare_parameter('corner_backup_seg',    0.8)    # s — retrocede antes de pivotar, para no rozar la pared/esquina
        self.declare_parameter('corner_backup_speed',  0.06)   # m/s — velocidad de ese retroceso
        # — Vueltas ————————————————————————————————————————————————————
        self.declare_parameter('num_vueltas',          1)     # vueltas a completar (0 = infinito)
        self.declare_parameter('dist_retorno',         0.40)  # m — dist al inicio para contar vuelta
        self.declare_parameter('min_dist_vuelta',      1.50)  # m — dist mínima antes de detectar retorno

        self.sector     = math.radians(self.get_parameter('sector_frontal_deg').value)
        self.d_alerta   = self.get_parameter('dist_alerta').value
        self.d_parada   = self.get_parameter('dist_parada').value
        self.d_colision = self.get_parameter('dist_colision').value
        self.v_crucero  = self.get_parameter('vel_crucero').value
        self.v_precauc  = self.get_parameter('vel_precaucion').value
        self.w_giro     = self.get_parameter('vel_giro').value
        self.v_min      = self.get_parameter('vel_min').value
        self.espera     = self.get_parameter('espera_seg').value
        self.ang_rodeo  = math.radians(self.get_parameter('angulo_rodeo_deg').value)
        self.t_avance   = self.get_parameter('avance_rodeo_seg').value
        self.t_ev_max   = self.get_parameter('t_evasion_max').value
        self.front_wall_ang   = math.radians(self.get_parameter('front_wall_deg').value)
        self.front_box_ang    = math.radians(self.get_parameter('front_box_deg').value)
        self.front_arc_radial = self.get_parameter('front_arc_radial').value
        self.side_clear       = self.get_parameter('side_clear_dist').value
        self.n_persist        = int(self.get_parameter('persist_frames').value)
        self.corner_yaw_tol   = math.radians(self.get_parameter('corner_yaw_tol_deg').value)
        self.corner_turn_time = self.get_parameter('corner_turn_time').value
        self.corner_backup_seg   = self.get_parameter('corner_backup_seg').value
        self.corner_backup_speed = self.get_parameter('corner_backup_speed').value
        self.num_vueltas   = int(self.get_parameter('num_vueltas').value)
        self.dist_retorno  = self.get_parameter('dist_retorno').value
        self.min_dist_v    = self.get_parameter('min_dist_vuelta').value

        # — Estado FSM ————————————————————————————————————————————————
        self.estado         = CRUCERO
        self.t_inicio       = self.get_clock().now()
        self.fase_rodeo     = 0
        self.modo_mejorado  = False
        self.scan_activo    = False  # espera activación desde web_monitor
        self._ultimo_log_espera = -1

        # — Sensores ————————————————————————————————————————————————————
        self.dist_frente     = float('inf')
        self.front_ang_width = 0.0    # rad — ancho angular del arco frontal continuo mas cercano
        self.left_min        = float('inf')  # m — min. distancia en el sector izquierdo (desempate)
        self.front_class     = 'NONE'  # 'NONE' | 'BOX' | 'CORNER'
        self._w_lateral  = 0.0

        # — Persistencia de transiciones (N frames consecutivos) ————————
        self._persist = {}

        # — Odometria: yaw (para el giro de esquina) —————————————————————
        self.yaw           = None
        self.have_odom_yaw = False
        self._corner_yaw_target = None
        self._corner_fase        = 0   # 0 = retroceder (ganar espacio), 1 = pivotar

        # — Censo de cajas (solo informativo en el log de rodeo) ————————
        self._cajas_censo: list = []   # [(ox, oy), ...]
        self._rodeo_dir   = 1          # siempre izquierda: se aleja de la pared derecha

        # — Odometría y vueltas ————————————————————————————————————————
        self._pos           = (0.0, 0.0)   # posición actual (x, y) en odom
        self._pos_inicio    = None         # posición al activar scan_active
        self._dist_max_v    = 0.0          # distancia máxima recorrida en vuelta actual
        self._vuelta_iniciada = False       # ya superó la dist mínima de vuelta
        self.vueltas        = 0            # vueltas completadas
        self.recorrido_done = False        # True cuando se alcanzaron num_vueltas

        # — Métricas ————————————————————————————————————————————————————
        self.dist_min_parada    = float('inf')
        self.colisiones         = 0
        self.rodeos_completados = 0
        self.rodeos_exitosos    = 0
        self._rodeo_colision    = False

        # — ROS I/O ————————————————————————————————————————————————————
        _qos = QoSProfile(depth=10)
        _qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, '/scan',               self.cb_scan,      _qos)
        self.create_subscription(Odometry,  '/odom',               self._cb_odom,     10)
        self.create_subscription(Float32,   '/lateral_correction', self._cb_lat,       10)
        self.create_subscription(Bool,      '/scan_active',        self._cb_active,    10)
        self.create_subscription(Bool,      '/enhanced_mode',      self._cb_enhanced,  10)
        self.create_subscription(PoseArray, '/cajas_G1',           self._cb_cajas,     10)

        self.pub_cmd     = self.create_publisher(Twist,   '/cmd_vel',      10)
        self.pub_estado  = self.create_publisher(String,  '/fsm_state',    10)
        self.pub_parada  = self.create_publisher(Float32, '/parada_dist',  10)
        self.pub_metrics = self.create_publisher(String,  '/fsm_metrics',  10)
        self.pub_vueltas = self.create_publisher(Int32,   '/vueltas',      10)

        self.create_timer(0.1, self.loop_control)
        self.create_timer(1.0, self._publish_metrics)

        self.get_logger().info(
            f'BehaviorFSM v2 listo\n'
            f'  dist_alerta={self.d_alerta} m  dist_parada={self.d_parada} m\n'
            f'  espera={self.espera} s  angulo_rodeo={math.degrees(self.ang_rodeo):.0f} deg')

    # — Callbacks ————————————————————————————————————————————————————————

    def cb_scan(self, msg: LaserScan):
        d_f = float('inf')
        front = []          # (angulo, rango) dentro del sector frontal
        left_min = float('inf')

        for i, r in enumerate(msg.ranges):
            raw   = msg.angle_min + i * msg.angle_increment
            valid = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            if not valid:
                continue

            # Sector frontal (normalizado a [-pi,pi])
            an = math.atan2(math.sin(raw), math.cos(raw))
            if abs(an) <= self.sector:
                d_f = min(d_f, r)
                front.append((an, r))
            # Sector izquierdo (45 a 135 deg) — desempate caja/esquina
            if math.pi / 4 <= an <= 3 * math.pi / 4:
                left_min = min(left_min, r)

        self.dist_frente     = d_f
        self.left_min        = left_min
        self.front_ang_width = self._arco_frontal(front)
        self.front_class     = self._clasificar_frente()

        # Detectar colision durante movimiento activo
        if self.scan_activo and d_f < self.d_colision:
            if self.estado not in (PARAR, ESPERAR_3S, PARAR_M):
                self.colisiones += 1
                self.get_logger().warn(
                    f'COLISION detectada! dist_frente={d_f:.3f} m '
                    f'colisiones_total={self.colisiones}')

    def _arco_frontal(self, front: list) -> float:
        """Ancho angular (rad) del arco frontal continuo que contiene el
        punto mas cercano. Un arco angosto = caja; uno ancho = pared/esquina."""
        if not front:
            return 0.0
        front.sort(key=lambda tr: tr[0])
        best_width, best_dmin = 0.0, float('inf')
        grupo = [front[0]]
        for k in range(1, len(front)):
            if abs(front[k][1] - front[k - 1][1]) < self.front_arc_radial:
                grupo.append(front[k])
            else:
                w  = grupo[-1][0] - grupo[0][0]
                dm = min(t[1] for t in grupo)
                if dm < best_dmin:
                    best_dmin, best_width = dm, w
                grupo = [front[k]]
        w  = grupo[-1][0] - grupo[0][0]
        dm = min(t[1] for t in grupo)
        if dm < best_dmin:
            best_dmin, best_width = dm, w
        return best_width

    def _clasificar_frente(self) -> str:
        """Distingue CAJA (arco angosto) de ESQUINA (arco ancho) usando el
        ancho angular del obstaculo frontal; en zona ambigua desempata con
        la distancia libre a la izquierda (despejada = esquina real)."""
        if self.dist_frente > self.d_alerta:
            return 'NONE'
        if self.front_ang_width >= self.front_wall_ang:
            return 'CORNER'
        if self.front_ang_width <= self.front_box_ang:
            return 'BOX'
        return 'CORNER' if self.left_min > self.side_clear else 'BOX'

    def _cb_lat(self, msg: Float32):
        self._w_lateral = msg.data

    def _cb_cajas(self, msg: PoseArray):
        """Actualiza el censo de cajas desde /cajas_G1."""
        self._cajas_censo = [(p.position.x, p.position.y) for p in msg.poses]

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.have_odom_yaw = True
        if self._pos_inicio is None:
            return
        d_inicio = math.hypot(p.x - self._pos_inicio[0], p.y - self._pos_inicio[1])
        # Acumular distancia máxima para saber que salimos del punto de inicio
        if d_inicio > self._dist_max_v:
            self._dist_max_v = d_inicio
        if self._dist_max_v >= self.min_dist_v:
            self._vuelta_iniciada = True
        # Detectar retorno al punto de inicio
        if self._vuelta_iniciada and d_inicio < self.dist_retorno:
            self._registrar_vuelta()

    def _registrar_vuelta(self):
        """Llamado cuando el robot regresa al punto de inicio."""
        self.vueltas += 1
        self._dist_max_v = 0.0
        self._vuelta_iniciada = False
        v_msg = Int32(); v_msg.data = self.vueltas
        self.pub_vueltas.publish(v_msg)
        self.get_logger().info(
            f'*** VUELTA {self.vueltas} completada '
            f'(objetivo={self.num_vueltas}) ***')
        if self.num_vueltas > 0 and self.vueltas >= self.num_vueltas:
            self.get_logger().info('Recorrido completo — deteniendo robot.')
            self.recorrido_done = True
            self.scan_activo = False
            self._pub(0.0, 0.0)

    def _cb_active(self, msg: Bool):
        self.scan_activo = msg.data
        if msg.data:
            # Registrar posición de inicio al activar
            self._pos_inicio = self._pos
            self._dist_max_v = 0.0
            self._vuelta_iniciada = False
            self.recorrido_done = False
            self.get_logger().info(
                f'scan_active=True — robot activo  '
                f'inicio=({self._pos[0]:.3f},{self._pos[1]:.3f})  '
                f'vueltas_objetivo={self.num_vueltas}')
        else:
            self._pub(0.0, 0.0)
            self.get_logger().info('scan_active=False — robot detenido')

    def _cb_enhanced(self, msg: Bool):
        self.modo_mejorado = msg.data
        nuevo = CRUCERO_M if msg.data else CRUCERO
        self._cambiar(nuevo)
        modo = 'MEJORADO (evasion fija)' if msg.data else 'NORMAL (RODEAR FSM)'
        self.get_logger().info(f'Modo cambiado a: {modo}')

    # — Helpers ————————————————————————————————————————————————————————

    def _pub(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        s = String(); s.data = self.estado
        self.pub_estado.publish(s)

    def _t_estado(self) -> float:
        return (self.get_clock().now() - self.t_inicio).nanoseconds * 1e-9

    def _persist_check(self, key: str, cond: bool) -> bool:
        """Debounce: exige n_persist lecturas /scan consecutivas con la
        condicion cumplida antes de confirmar la transicion. Evita que un
        solo frame ruidoso dispare cambios de estado (zigzag PARAR/EVADIR)."""
        c = self._persist.get(key, 0)
        c = c + 1 if cond else 0
        self._persist[key] = c
        return c >= self.n_persist

    def _persist_reset(self):
        self._persist.clear()

    def _corner_begin(self):
        """Prepara el giro de esquina: objetivo de yaw (~90 deg, siempre a
        la izquierda) si hay odometria, o un respaldo por tiempo si no.
        Debe llamarse justo despues de _cambiar(ESQUINA_M), que resetea
        t_inicio (usado por el respaldo de tiempo via _t_estado())."""
        self._persist_reset()
        self._corner_fase = 0   # primero retrocede para ganar espacio, luego pivota
        if self.have_odom_yaw and self.yaw is not None:
            target = self.yaw + math.pi / 2   # +90 deg, siempre izquierda
            self._corner_yaw_target = math.atan2(math.sin(target), math.cos(target))
        else:
            self._corner_yaw_target = None

    def _corner_step(self):
        """Un paso del giro de esquina. Devuelve (listo, w)."""
        if self._corner_yaw_target is not None:
            err = math.atan2(math.sin(self._corner_yaw_target - self.yaw),
                              math.cos(self._corner_yaw_target - self.yaw))
            if abs(err) < self.corner_yaw_tol:
                return True, 0.0
            return False, (self.w_giro if err > 0 else -self.w_giro)
        else:
            if self._t_estado() >= self.corner_turn_time:
                return True, 0.0
            return False, self.w_giro

    def _cambiar(self, nuevo: str):
        self.get_logger().info(
            f'[FSM] {self.estado} -> {nuevo}  '
            f'(d_frente={self.dist_frente:.3f} m)')
        self.estado   = nuevo
        self.t_inicio = self.get_clock().now()
        self._ultimo_log_espera = -1  # reset countdown

    def _publish_metrics(self):
        td = (self.rodeos_exitosos / self.rodeos_completados
              if self.rodeos_completados > 0 else 0.0)
        data = {
            'estado':               self.estado,
            'modo_mejorado':        self.modo_mejorado,
            'scan_activo':          self.scan_activo,
            'dist_frente_m':        round(self.dist_frente, 3)
                                    if math.isfinite(self.dist_frente) else None,
            'dist_min_parada_cm':   round(self.dist_min_parada * 100, 1)
                                    if math.isfinite(self.dist_min_parada) else None,
            'colisiones':           self.colisiones,
            'rodeos_completados':   self.rodeos_completados,
            'rodeos_exitosos':      self.rodeos_exitosos,
            'tasa_rodeo':           round(td, 3),
            'vueltas':              self.vueltas,
            'num_vueltas':          self.num_vueltas,
            'recorrido_done':       self.recorrido_done,
        }
        m = String(); m.data = json.dumps(data)
        self.pub_metrics.publish(m)

    # — FSM Normal ————————————————————————————————————————————————————

    def _fsm_normal(self):
        d = self.dist_frente

        if self.estado == CRUCERO:
            if d < self.d_parada:
                self._registrar_parada(d)
                self._cambiar(PARAR)
            elif d < self.d_alerta:
                self._cambiar(CAJA_DETECTADA)
            else:
                self._pub(self.v_crucero, self._w_lateral)

        elif self.estado == CAJA_DETECTADA:
            if d < self.d_parada:
                self._registrar_parada(d)
                self._cambiar(PARAR)
            elif d >= self.d_alerta:
                self._cambiar(CRUCERO)
            else:
                self._pub(self.v_precauc, 0.0)

        elif self.estado == PARAR:
            self._pub(0.0, 0.0)
            self._rodeo_colision = False
            self._cambiar(ESPERAR_3S)

        elif self.estado == ESPERAR_3S:
            self._pub(0.0, 0.0)
            t = self._t_estado()
            # Log cada segundo
            seg_actual = int(t)
            if not hasattr(self, '_ultimo_log_espera') or self._ultimo_log_espera != seg_actual:
                self._ultimo_log_espera = seg_actual
                self.get_logger().info(
                    f'[ESPERAR_3S] {t:.1f}/{self.espera:.0f} s ...')
            if t >= self.espera:
                self.fase_rodeo = 0
                self._cambiar(RODEAR)

        elif self.estado == RODEAR:
            self._ejecutar_rodeo()

    def _registrar_parada(self, d: float):
        self.dist_min_parada = min(self.dist_min_parada, d)
        dm = Float32(); dm.data = float(d)
        self.pub_parada.publish(dm)
        self.get_logger().info(
            f'PARADA registrada: {d:.3f} m  '
            f'(min_historico={self.dist_min_parada:.3f} m)')

    # — Maniobra de rodeo (3 subfases) ————————————————————————————————

    def _ejecutar_rodeo(self):
        t_giro = self.ang_rodeo / self.w_giro
        t = self._t_estado()

        # Marcar colision si ocurre durante el rodeo
        if self.dist_frente < self.d_colision:
            self._rodeo_colision = True

        if self.fase_rodeo == 0:         # Girar hacia el centro (izquierda), lejos de la pared derecha
            if t < 0.05:
                self.get_logger().info(
                    f'[RODEO] dir=IZQ (fijo, se aleja de la pared derecha)'
                    f'  cajas_censo={len(self._cajas_censo)}')
            if t < t_giro:
                self._pub(0.0, self.w_giro * self._rodeo_dir)
            else:
                self.fase_rodeo = 1
                self.t_inicio = self.get_clock().now()

        elif self.fase_rodeo == 1:       # Avanzar bordeando la caja
            if t < self.t_avance:
                self._pub(self.v_crucero, 0.0)
            else:
                self.fase_rodeo = 2
                self.t_inicio = self.get_clock().now()

        elif self.fase_rodeo == 2:       # Girar de vuelta (dirección contraria)
            if t < t_giro:
                self._pub(0.0, -self.w_giro * self._rodeo_dir)
            else:
                self.fase_rodeo = 3
                self.t_inicio = self.get_clock().now()

        else:                            # Rodeo completado
            self._pub(0.0, 0.0)
            self.rodeos_completados += 1
            if not self._rodeo_colision:
                self.rodeos_exitosos += 1
                self.get_logger().info(
                    f'Rodeo #{self.rodeos_completados} EXITOSO '
                    f'(exitosos={self.rodeos_exitosos})')
            else:
                self.get_logger().warn(
                    f'Rodeo #{self.rodeos_completados} con COLISION')
            self._cambiar(CRUCERO)

    # — FSM Mejorada (gap navigation) ————————————————————————————————

    def _fsm_mejorada(self):
        if self.estado == CRUCERO_M:
            if self._persist_check('parar_m', self.dist_frente <= self.d_parada):
                self._registrar_parada(self.dist_frente)
                self._cambiar(PARAR_M)
                return
            v = self.v_crucero if self.dist_frente >= self.d_alerta else self.v_min
            w = self._w_lateral if self.dist_frente >= self.d_alerta else 0.0
            self._pub(v, w)

        elif self.estado == PARAR_M:
            self._pub(0.0, 0.0)
            t = self._t_estado()
            if t >= self.espera:
                if self.front_class == 'CORNER':
                    self.get_logger().info(
                        f'[PARAR_M] frente=ESQUINA (ancho arco='
                        f'{math.degrees(self.front_ang_width):.0f} deg) -> giro 90 grados')
                    self._cambiar(ESQUINA_M)
                    self._corner_begin()
                else:
                    self.get_logger().info(
                        f'[PARAR_M] frente=CAJA (ancho arco='
                        f'{math.degrees(self.front_ang_width):.0f} deg) -> evasion')
                    self._persist_reset()
                    self._cambiar(EVADIR_M)

        elif self.estado == ESQUINA_M:
            # Fase 0: retrocede un poco para ganar espacio antes de pivotar
            # (si gira pegado a la pared/esquina, la parte trasera del
            # chasis la roza — el sensor frontal no detecta ese roce lateral).
            # Fase 1: pivota ~90 grados a la izquierda (odometria si esta
            # disponible, si no por tiempo). Al terminar, wall_follower
            # reengancha la pared derecha del nuevo tramo.
            if self._corner_fase == 0:
                if self._t_estado() < self.corner_backup_seg:
                    self._pub(-self.corner_backup_speed, 0.0)
                else:
                    self._corner_fase = 1
                    self.t_inicio = self.get_clock().now()
            else:
                listo, w = self._corner_step()
                self._pub(0.0, w)
                if listo:
                    self._persist_reset()
                    self._cambiar(CRUCERO_M)

        elif self.estado == EVADIR_M:
            # Gira siempre hacia la izquierda (se aleja de la pared derecha)
            # hasta despejar el frente; al volver a CRUCERO_M, wall_follower
            # reengancha la distancia con la pared derecha.
            if self._t_estado() > self.t_ev_max:
                self.get_logger().warn('Timeout evasion -> CRUCERO_M')
                self._persist_reset()
                self._cambiar(CRUCERO_M)
                return
            if self._persist_check('crucero_clear', self.dist_frente > self.d_alerta):
                self._persist_reset()
                self._cambiar(CRUCERO_M)
                return
            puede = self.dist_frente > self.d_parada
            self._pub(self.v_min if puede else 0.0, self.w_giro)

    # — Bucle principal ————————————————————————————————————————————————

    def loop_control(self):
        if not self.scan_activo:
            return
        if self.modo_mejorado:
            self._fsm_mejorada()
        else:
            self._fsm_normal()


def main(args=None):
    rclpy.init(args=args)
    nodo = BehaviorFSM()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo._pub(0.0, 0.0)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
