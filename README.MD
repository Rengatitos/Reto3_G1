# CapyTown G1 — Reto LiDAR

## El Censo y el Guardián de las Cajas

**ESAN — Robótica de Móviles 2026-I**
Robot **Yahboom Pi5** · LiDAR **MS200** · **ROS2 Humble** · Docker

---

## Descripción

Este proyecto implementa el reto clasificatorio LiDAR **“El Censo y el Guardián de las Cajas”**.

El robot debe recorrer el **Jirón Principal** usando únicamente el LiDAR `/scan` para:

* detectar cajas,
* contar cuántas cajas hay,
* estimar su posición,
* publicar markers para RViz,
* detenerse ante obstáculos,
* rodear cajas sin chocarlas,
* registrar métricas por corrida.

El sistema está dividido en dos partes:

* **Parte A — El Censo:** detección de cajas con LiDAR.
* **Parte B — El Guardián:** reacción y recorrido seguro con FSM.

---

# Estructura del proyecto

```text
yahboomcar_ws/
└── src/
    └── paquetes/
        ├── box_detector/
        │   └── # Paquete base / versión previa
        │
        ├── box_detector_G1/              # Parte A: Censo de cajas con LiDAR
        │   ├── box_detector_G1/
        │   │   ├── box_detector_G1.py    # Nodo principal: detecta, cuenta y ubica cajas
        │   │   ├── web_monitor.py        # Dashboard web en puerto 8080
        │   │   ├── web_monitor_readonly.py
        │   │   ├── neon_manager.py       # Persistencia en Neon DB / respaldo
        │   │   ├── viz_scan.py           # Visualizador/debug del /scan
        │   │   └── viz_lineas.py         # Visualizador/debug de líneas laterales
        │   │
        │   ├── config/
        │   │   └── params_G1.yaml        # Parámetros del detector de cajas
        │   │
        │   ├── launch/
        │   │   └── monitor.launch.py
        │   │
        │   ├── package.xml
        │   └── setup.py
        │
        └── behavior_fsm/                 # Parte B: Guardián con FSM
            ├── behavior_fsm/
            │   ├── behavior_fsm.py       # FSM normal y FSM mejorada
            │   └── wall_follower.py      # Seguimiento de pared lateral
            │
            ├── config/
            │   └── params.yaml           # Parámetros de recorrido y seguridad
            │
            ├── launch/
            │   └── capytown.launch.py    # Launch principal del sistema
            │
            ├── package.xml
            └── setup.py
```

---

# Parte A — Censo de cajas

Paquete:

```bash
box_detector_G1
```

Nodo principal:

```bash
box_detector_G1.py
```

Función:

* Se suscribe a `/scan`.
* Lee los puntos del LiDAR.
* Filtra lecturas inválidas.
* Convierte de polar a cartesiano.
* Agrupa puntos mediante clustering.
* Valida clusters que parecen cajas.
* Calcula centroides.
* Usa `/odom` para fijar la posición de cada caja en el mundo.
* Deduplica cajas repetidas.
* Publica las cajas detectadas.

Tópicos publicados:

```bash
/cajas_avistadas
/cajas_markers
/cajas_G1
/cajas_markers_G1
```

---

# Parte B — Guardián

Paquete:

```bash
behavior_fsm
```

Nodos:

```bash
behavior_fsm.py
wall_follower.py
```

La FSM normal sigue la lógica del reto:

```text
CRUCERO → CAJA_DETECTADA → PARAR → ESPERAR_3S → RODEAR → CRUCERO
```

También existe una FSM mejorada:

```text
CRUCERO_M → PARAR_M → ESQUINA_M / EVADIR_M → CRUCERO_M
```

La FSM mejorada intenta diferenciar entre:

* caja,
* pared,
* esquina,
* zona libre.

El nodo `wall_follower.py` ayuda a mantenerse dentro del jirón usando el LiDAR lateral y publica correcciones en:

```bash
/lateral_correction
```

---

# Entrar al Docker

Desde la Raspberry Pi:

```bash
bash /home/pi/ros2_humble.sh
```

Una vez dentro del contenedor, verificar ROS2:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

---

# Compilar el proyecto

Dentro del Docker:

```bash
cd /root/yahboomcar_ws
colcon build --packages-select box_detector box_detector_G1 behavior_fsm --symlink-install
source install/setup.bash
```

Si quieres compilar limpio:

```bash
cd /root/yahboomcar_ws
rm -rf build install log
colcon build --packages-select box_detector box_detector_G1 behavior_fsm --symlink-install
source install/setup.bash
```

---

# Lanzar el sistema principal

Para abrir el monitor web y controlar la Fase A / Fase B desde navegador:

```bash
ros2 launch behavior_fsm capytown.launch.py
```

Luego abrir en el navegador:

```text
http://<IP_RASPBERRY>:8080
```

Ejemplo:

```text
http://192.168.1.35:8080
```

Si se usa VSCode con SSH y port forwarding:

```text
http://localhost:8080
```

---

# Flujo normal de uso

```bash
# 1. Entrar al Docker
bash /home/pi/ros2_humble.sh

# 2. Ir al workspace
cd /root/yahboomcar_ws

# 3. Compilar
colcon build --packages-select box_detector box_detector_G1 behavior_fsm --symlink-install

# 4. Cargar entorno
source install/setup.bash

# 5. Lanzar monitor web
ros2 launch behavior_fsm capytown.launch.py
```

Abrir:

```text
http://<IP_RASPBERRY>:8080
```

Desde el monitor web:

1. Presionar **Iniciar escaneo**.
2. Esperar que se active la Parte A.
3. Ver las cajas detectadas.
4. Activar recorrido mejorado si corresponde.
5. Ver estado de la FSM.
6. Guardar la corrida.
7. Repetir hasta completar 10 corridas.

---

# Lanzar solo detector de cajas

Para debug de la Parte A:

```bash
cd /root/yahboomcar_ws
source install/setup.bash
ros2 run box_detector_G1 box_detector_G1
```

Ver cajas detectadas:

```bash
ros2 topic echo /cajas_avistadas
```

Ver markers:

```bash
ros2 topic echo /cajas_markers --once
```

---

# Lanzar solo FSM

Para debug de la Parte B:

```bash
cd /root/yahboomcar_ws
source install/setup.bash
ros2 run behavior_fsm behavior_fsm
```

Ver estado de la FSM:

```bash
ros2 topic echo /fsm_state
```

Ver distancia de parada:

```bash
ros2 topic echo /parada_dist
```

Ver métricas de la FSM:

```bash
ros2 topic echo /fsm_metrics
```

---

# Lanzar solo wall follower

Para debug del seguimiento de pared:

```bash
cd /root/yahboomcar_ws
source install/setup.bash
ros2 run behavior_fsm wall_follower
```

Ver corrección lateral:

```bash
ros2 topic echo /lateral_correction
```

---

# Tópicos principales

## Sensores

```bash
/scan
/odom
/odom_raw
/imu
```

## Movimiento

```bash
/cmd_vel
```

## Cajas detectadas

```bash
/cajas_avistadas
/cajas_markers
/cajas_G1
/cajas_markers_G1
```

## FSM y monitor

```bash
/fsm_state
/parada_dist
/fsm_metrics
/vueltas
/scan_active
/enhanced_mode
/lateral_correction
```

---

# Verificar LiDAR

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Si el LiDAR funciona, debe aparecer un mensaje con:

```text
angle_min
angle_max
angle_increment
range_min
range_max
ranges
intensities
```

---

# Verificar odometría

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
```

La odometría permite transformar las cajas desde el marco del robot al marco del mundo.

---

# Verificar nodos activos

```bash
ros2 node list
```

Nodos esperados durante una corrida:

```text
/web_monitor
/box_detector_G1
/behavior_fsm
/wall_follower
/YB_Car_Node
```

Si no aparece `/box_detector_G1`, probablemente la Fase A no está activa desde la web.

---

# Frenar el robot manualmente

Si el robot sigue avanzando:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Si no responde, detener el contenedor desde otra terminal:

```bash
docker stop <NOMBRE_CONTENEDOR>
```

O forzar:

```bash
docker kill <NOMBRE_CONTENEDOR>
```

Ejemplo:

```bash
docker kill stupefied_wu
```

---

# Actualizar código desde Git

Si el proyecto está en GitHub:

```bash
cd /root/yahboomcar_ws/src/paquetes
git pull
```

Luego recompilar:

```bash
cd /root/yahboomcar_ws
colcon build --packages-select box_detector box_detector_G1 behavior_fsm --symlink-install
source install/setup.bash
```

---

# Cuándo hacer `colcon build`

Solo necesitas hacer `colcon build` cuando cambias código Python, `setup.py`, `package.xml`, launch files o estructura del paquete.

Ejemplos:

```text
box_detector_G1.py
behavior_fsm.py
wall_follower.py
web_monitor.py
setup.py
package.xml
launch/*.py
```

Si solo cambias YAML y el launch usa el archivo fuente, normalmente basta con relanzar.

Pero si el launch usa el YAML instalado, puedes editar el YAML instalado directamente.

---

# Cambiar parámetros sin rebuild

Si el launch está leyendo el YAML instalado, editar:

```bash
nano /root/yahboomcar_ws/install/box_detector_G1/share/box_detector_G1/config/params_G1.yaml
```

o:

```bash
nano /root/yahboomcar_ws/install/behavior_fsm/share/behavior_fsm/config/params.yaml
```

Luego relanzar:

```bash
ros2 launch behavior_fsm capytown.launch.py
```

No hace falta `colcon build` si solo cambias esos valores.

---

# Parámetros clave de detección

Archivo fuente:

```bash
/root/yahboomcar_ws/src/paquetes/box_detector_G1/config/params_G1.yaml
```

Archivo instalado, si se usa el YAML instalado:

```bash
/root/yahboomcar_ws/install/box_detector_G1/share/box_detector_G1/config/params_G1.yaml
```

Parámetros importantes:

```yaml
umbral_cluster: 0.12
ancho_caja: 0.20
tolerancia_ancho: 0.12
min_puntos: 4
dist_duplicado: 0.28
```

## `umbral_cluster`

Define la distancia máxima entre puntos consecutivos para formar un mismo cluster.

* Muy alto: puede unir dos cajas.
* Muy bajo: puede dividir una caja.

## `ancho_caja`

Define el ancho esperado de una caja.

Ejemplo:

```yaml
ancho_caja: 0.20
```

Para cajas de 20 cm.

## `tolerancia_ancho`

Permite aceptar cajas un poco más pequeñas o grandes.

* Alta tolerancia: detecta más, pero puede confundir paredes.
* Baja tolerancia: evita falsos positivos, pero puede perder cajas.

## `min_puntos`

Cantidad mínima de puntos LiDAR para aceptar un cluster.

* Alto: más seguro.
* Bajo: más sensible al ruido.

## `dist_duplicado`

Distancia usada para no contar dos veces la misma caja.

* Alto: fusiona detecciones cercanas.
* Bajo: puede contar la misma caja varias veces.

---

# Parámetros clave del recorrido

Archivo fuente:

```bash
/root/yahboomcar_ws/src/paquetes/behavior_fsm/config/params.yaml
```

Archivo instalado:

```bash
/root/yahboomcar_ws/install/behavior_fsm/share/behavior_fsm/config/params.yaml
```

Valores recomendados para pruebas seguras:

```yaml
behavior_fsm:
  ros__parameters:
    sector_frontal_deg: 55.0
    dist_alerta: 0.45
    dist_parada: 0.30
    dist_colision: 0.14

    vel_crucero: 0.07
    vel_precaucion: 0.035
    vel_giro: 0.30
    vel_min: 0.025

    espera_seg: 3.0
    angulo_rodeo_deg: 45.0
    avance_rodeo_seg: 1.2

    persist_frames: 2
    side_clear_dist: 0.45
    t_evasion_max: 6.0
```

## `sector_frontal_deg`

Ángulo frontal que revisa el robot.

```yaml
sector_frontal_deg: 55.0
```

Si es muy pequeño, puede no ver cajas laterales.
Si es muy grande, puede confundir maderas laterales con obstáculos frontales.

## `dist_alerta`

Distancia donde el robot empieza a preocuparse.

```yaml
dist_alerta: 0.45
```

Si algo aparece a menos de 45 cm, el robot reduce velocidad o se prepara para parar.

## `dist_parada`

Distancia donde el robot debe detenerse.

```yaml
dist_parada: 0.30
```

Si es muy baja, el robot frena tarde y puede chocar.

## `dist_colision`

Distancia considerada como choque o casi choque.

```yaml
dist_colision: 0.14
```

Debe ser mayor o igual al rango mínimo real del LiDAR.

## `vel_crucero`

Velocidad normal de avance.

```yaml
vel_crucero: 0.07
```

Mientras más alta, más difícil es evitar colisiones.

## `vel_giro`

Velocidad angular del giro.

```yaml
vel_giro: 0.30
```

Si es muy alta, el robot gira brusco.
Si es muy baja, puede no esquivar a tiempo.

## `angulo_rodeo_deg`

Ángulo usado para rodear una caja.

```yaml
angulo_rodeo_deg: 45.0
```

Si es bajo, no esquiva suficiente.
Si es alto, puede irse contra la pared.

## `avance_rodeo_seg`

Tiempo que avanza mientras rodea.

```yaml
avance_rodeo_seg: 1.2
```

Si es muy corto, no pasa la caja.
Si es muy largo, puede chocar con otra caja o pared.

## `persist_frames`

Número de lecturas consecutivas necesarias para confirmar una decisión.

```yaml
persist_frames: 2
```

Sirve para evitar reaccionar por ruido de un solo frame del LiDAR.

---

# Orientación del LiDAR

Si el robot reacciona al revés o detecta obstáculos donde no debería, revisar la orientación del LiDAR.

En algunos Yahboom con MS200 el frente puede estar en:

```yaml
lidar_front_deg: 180.0
```

En otros montajes:

```yaml
lidar_front_deg: 0.0
```

Para verificar, coloca una caja justo al frente del robot y ejecuta:

```bash
ros2 topic echo /scan --once
```

O usa un script de diagnóstico para ver el ángulo de los puntos más cercanos.

Si la caja al frente aparece cerca de `180°`, usar:

```yaml
lidar_front_deg: 180.0
```

Si aparece cerca de `0°`, usar:

```yaml
lidar_front_deg: 0.0
```

---

# Ground truth de cajas

Para las métricas, medir con cinta las posiciones reales de las 5 cajas en el marco `odom`.

Formato:

```yaml
ground_truth: [x1, y1, x2, y2, x3, y3, x4, y4, x5, y5]
```

Ejemplo:

```yaml
ground_truth: [0.8, 0.3, 1.5, -0.2, 2.2, 0.4, 3.0, -0.3, 3.8, 0.2]
```

Este valor permite comparar cajas reales vs cajas detectadas.

---

# Métricas

El reto pide registrar 10 corridas.

Campos esperados:

```text
corrida
cajas_reales
cajas_detectadas
VP
FP
FN
error_pos_prom_cm
dist_min_parada_cm
colisiones
rodeo_exitoso
```

La tasa de detección se calcula así:

```text
tasa_deteccion = VP / (VP + FN)
```

Archivo esperado:

```bash
/root/metricas_lidar.csv
```

Si el sistema guarda en Neon DB o SQLite, se recomienda exportar también el CSV porque el reto lo pide explícitamente.

---

# Flujo entre corridas

Para cada corrida:

1. Colocar 5 cajas en la pista.
2. Separar las cajas al menos 40 cm.
3. Medir posiciones reales si se usará `ground_truth`.
4. Iniciar el monitor web.
5. Presionar **Iniciar escaneo**.
6. Revisar cajas detectadas.
7. Activar recorrido mejorado.
8. Verificar que el robot no choque.
9. Guardar la corrida.
10. Cambiar posiciones de cajas.
11. Repetir hasta completar 10 corridas.

---

# RViz por navegador con noVNC

Para cumplir con la captura de RViz, usar noVNC.

Instalar dependencias dentro del Docker:

```bash
apt update
apt install -y ros-humble-rviz2 xfce4 tigervnc-standalone-server novnc websockify dbus-x11
```

Iniciar VNC manualmente:

```bash
mkdir -p ~/.vnc

cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
startxfce4 &
EOF

chmod +x ~/.vnc/xstartup

vncserver :1 -geometry 1280x720 -depth 24
websockify --web=/usr/share/novnc/ 6080 localhost:5901 &
export DISPLAY=:1

source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash

rviz2
```

Abrir en navegador:

```text
http://<IP_RASPBERRY>:6080/vnc.html
```

Con VSCode SSH y port forwarding:

```text
http://localhost:6080/vnc.html
```

En RViz agregar:

```text
LaserScan      /scan
MarkerArray    /cajas_markers
PoseArray      /cajas_avistadas
MarkerArray    /cajas_markers_G1
PoseArray      /cajas_G1
TF
Grid
```

Fixed Frame recomendado:

```text
odom
```

Si no funciona:

```text
base_link
```

---

# Monitor visual LiDAR

Si se usa visualizador con matplotlib:

```bash
cd /root/yahboomcar_ws
source install/setup.bash
python3 /root/yahboomcar_ws/src/paquetes/box_detector_G1/box_detector_G1/viz_scan.py
```

Para líneas laterales:

```bash
python3 /root/yahboomcar_ws/src/paquetes/box_detector_G1/box_detector_G1/viz_lineas.py
```

Si falta matplotlib:

```bash
apt update
apt install -y python3-matplotlib python3-tk
```

---

# Errores comunes

## `source /opt/ros/humble/setup.bash: No such file or directory`

Estás fuera del Docker.

Solución:

```bash
bash /home/pi/ros2_humble.sh
```

---

## `colcon: command not found`

Probablemente estás fuera del Docker o no cargaste ROS2.

Solución:

```bash
source /opt/ros/humble/setup.bash
```

---

## `container is not running`

Iniciar el contenedor:

```bash
docker start <NOMBRE_CONTENEDOR>
docker exec -it <NOMBRE_CONTENEDOR> bash
```

Ejemplo:

```bash
docker start stupefied_wu
docker exec -it stupefied_wu bash
```

---

## El robot avanza aunque no debería

Frenar por ROS2:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Si no se detiene:

```bash
docker kill <NOMBRE_CONTENEDOR>
```

---

## No detecta cajas

Verificar LiDAR:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Verificar tópicos de cajas:

```bash
ros2 topic echo /cajas_avistadas
ros2 topic echo /cajas_G1
```

Revisar parámetros:

```yaml
umbral_cluster
ancho_caja
tolerancia_ancho
min_puntos
dist_duplicado
```

---

## Detecta cajas pero igual choca

Revisar parámetros de seguridad:

```yaml
dist_alerta
dist_parada
dist_colision
vel_crucero
vel_giro
angulo_rodeo_deg
avance_rodeo_seg
```

Valores más seguros para pruebas:

```yaml
dist_alerta: 0.45
dist_parada: 0.30
dist_colision: 0.14
vel_crucero: 0.07
vel_giro: 0.30
angulo_rodeo_deg: 45.0
avance_rodeo_seg: 1.2
```

---

# Comandos rápidos

## Entrar al Docker

```bash
bash /home/pi/ros2_humble.sh
```

## Compilar

```bash
cd /root/yahboomcar_ws
colcon build --packages-select box_detector box_detector_G1 behavior_fsm --symlink-install
source install/setup.bash
```

## Lanzar monitor

```bash
ros2 launch behavior_fsm capytown.launch.py
```

## Abrir web

```text
http://<IP_RASPBERRY>:8080
```

## Ver LiDAR

```bash
ros2 topic hz /scan
```

## Ver cajas

```bash
ros2 topic echo /cajas_avistadas
```

## Ver FSM

```bash
ros2 topic echo /fsm_state
```

## Frenar robot

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

# Entregables del reto

* Código de `box_detector_G1`.
* Código de `behavior_fsm`.
* Código de `wall_follower`.
* Monitor web funcional.
* `metricas_lidar.csv` con 10 corridas.
* Captura de RViz mostrando `/scan` y markers.
* Video del robot censando y rodeando cajas.
* README del proyecto.
* Explicación de parámetros YAML.
* Evidencia de que el robot no tumba ni roza cajas.

---

# Resumen para defensa

El sistema usa únicamente el LiDAR `/scan`.

Primero, `box_detector_G1` detecta cajas mediante clustering, calcula sus centroides y transforma sus posiciones al marco `/odom`.

Luego, `behavior_fsm` decide cómo moverse: avanzar, detenerse, esperar, rodear o girar si detecta una esquina.

El nodo `wall_follower` ayuda a mantenerse dentro del jirón usando las paredes laterales.

Los archivos YAML permiten calibrar el robot sin cambiar el código. Con ellos se ajusta qué se considera caja, cuándo frenar, qué tan rápido avanzar, cuánto girar y qué margen de seguridad usar.
