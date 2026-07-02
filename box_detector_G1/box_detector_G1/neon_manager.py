#!/usr/bin/env python3
"""
neon_manager.py — Almacenamiento en Neon PostgreSQL (cloud) para CapyTown G1.

Usa el HTTP API de Neon Serverless (HTTPS/443) para evitar restricciones
de red en el contenedor Docker (puerto 5432 bloqueado).

Tablas:
    corridas      — una fila por corrida (métricas globales)
    cajas_corrida — posiciones de las cajas detectadas para esa corrida

ESAN — Robotica de Moviles 2026-I  |  CapyTown Grupo 1
"""

import json
import threading
import urllib.request
import urllib.error

_NEON_CONN_STRING = (
    "postgresql://neondb_owner:npg_YtiMj8JVGF7g"
    "@ep-lingering-breeze-ato68uvj-pooler.c-9.us-east-1.aws.neon.tech"
    "/neondb?sslmode=require"
)
_NEON_HTTP_URL = (
    "https://ep-lingering-breeze-ato68uvj-pooler.c-9.us-east-1.aws.neon.tech/sql"
)
_NEON_HEADERS = {
    "Content-Type":         "application/json",
    "Neon-Connection-String": _NEON_CONN_STRING,
    "Neon-Raw-Text-Output": "false",
    "Neon-Array-Mode":      "false",
}


def _neon_exec(query: str, params: list = None) -> dict:
    """Ejecuta una consulta SQL vía el HTTP API de Neon.

    Devuelve el JSON de respuesta con campos:
        rows    — lista de dicts (filas)
        command — tipo de comando (SELECT, INSERT, …)
        rowCount
    """
    payload = json.dumps({"query": query, "params": params or []}).encode()
    req = urllib.request.Request(
        _NEON_HTTP_URL,
        data=payload,
        headers=_NEON_HEADERS,
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode())


class NeonRunDB:
    """Gestor de corridas en Neon PostgreSQL (cloud) vía HTTP API."""

    def __init__(self):
        self._lock = threading.Lock()

    # — API principal ————————————————————————————————————————————————————

    def save_run(self, data: dict) -> int:
        """
        Guarda una corrida en Neon DB.

        Parámetros esperados en data:
          cajas_reales (int)           — número real de cajas en la pista
          cajas_detectadas (int)       — detectadas por el censo
          error_pos_prom_cm (float)    — error de posición medio
          dist_min_parada_cm (float)   — distancia mínima al parar (en cm)
          colisiones (int)             — colisiones detectadas por FSM
          rodeo_exitoso (int)          — rodeos exitosos completados
          vueltas (int)                — vueltas completadas
          cajas_pos (list[(x,y,r)])    — posiciones del censo (odom frame)

        Devuelve el id de la corrida insertada, o -1 si falla.
        """
        cr  = int(data.get('cajas_reales',       5))
        cd  = int(data.get('cajas_detectadas',   0))
        ep  = float(data.get('error_pos_prom_cm',  0.0))
        dmp = float(data.get('dist_min_parada_cm', 0.0))
        col = int(data.get('colisiones',           0))
        re  = int(data.get('rodeo_exitoso',         0))
        vueltas = int(data.get('vueltas',           0))

        vp  = min(cr, cd)
        fp  = max(0, cd - cr)
        fn  = max(0, cr - cd)
        td  = vp / (vp + fn) if (vp + fn) > 0 else 0.0

        try:
            with self._lock:
                # Insertar corrida
                res = _neon_exec("""
                    INSERT INTO corridas
                    (cajas_reales, cajas_detectadas, vp, fp, fn,
                     error_pos_prom_cm, dist_min_parada_cm, colisiones,
                     rodeo_exitoso, tasa_deteccion, vueltas, modo)
                    VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,$12)
                    RETURNING id
                """, [cr, cd, vp, fp, fn, ep, dmp, col, re,
                      round(td, 4), vueltas, 'mejorado'])

                run_id = res['rows'][0]['id']

                # Insertar posiciones de cajas
                for bx, by, br in data.get('cajas_pos', []):
                    _neon_exec(
                        "INSERT INTO cajas_corrida (corrida, x_odom, y_odom, radio) "
                        "VALUES ($1,$2,$3,$4)",
                        [run_id, float(bx), float(by), float(br)])

                return run_id
        except Exception as exc:
            print(f'[NeonDB] ERROR al guardar corrida: {exc}')
            return -1

    def get_recent_runs(self, n: int = 10) -> list:
        """Devuelve las últimas n corridas como lista de dicts."""
        try:
            with self._lock:
                res = _neon_exec("""
                    SELECT id, timestamp, cajas_reales, cajas_detectadas,
                           vp, fp, fn, error_pos_prom_cm, dist_min_parada_cm,
                           colisiones, rodeo_exitoso, tasa_deteccion,
                           vueltas, modo
                    FROM corridas
                    ORDER BY id DESC LIMIT $1
                """, [n])
                return res.get('rows', [])
        except Exception as exc:
            print(f'[NeonDB] ERROR al leer corridas: {exc}')
            return []

    def count_runs(self) -> int:
        """Devuelve el número total de corridas guardadas."""
        try:
            with self._lock:
                res = _neon_exec("SELECT COUNT(*) AS total FROM corridas")
                return int(res['rows'][0]['total'])
        except Exception:
            return -1

    def ping(self) -> bool:
        """Verifica la conectividad con Neon DB."""
        try:
            res = _neon_exec("SELECT 1 AS ok")
            return res['rows'][0]['ok'] == 1
        except Exception:
            return False
