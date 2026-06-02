# Plan de Implementación — AMR Montacargas

**Reto:** TE3003B · ITESM FJ2026  
**Robot:** Diferencial con lifter tipo montacargas, controlado por FPGA  
**Misión:** Mover pallets entre camiones, racks y rollers en almacén a escala

---

## Estructura del repositorio

```
src/
├── bringup/            ✅ Renombrar desde puzzlebot_bringup   — launch files del sistema
├── controller/         ✅ Renombrar desde puzzlebot_controller — cinemática diferencial + odometría
├── description/        ✅ Renombrar desde puzzlebot_description — URDF + mundos Gazebo
├── localization/       ✅ Renombrar desde puzzlebot_localization_cpp — EKF C++ + ICP 2D
├── slam/               ✅ Renombrar desde puzzlebot_slam       — SLAM ICP + OccupancyGrid
│
├── navigation/         🔨 Crear — A* + Bug1 + PID path follower
├── mission_control/    🔨 Crear — máquina de estados YASMIN
├── perception/         🔨 Crear — Aruco, CNN tráiler, alineación PID
├── lifting/            🔨 Crear — GPIO → FPGA (real) | mock (simulación)
├── voice_control/      🔨 Crear — LPC + VQ, parser de comandos
└── dashboard/          🔨 Crear — Flask web: telemetría + streaming + misiones
```

> **Screaming architecture:** el árbol de `src/` grita lo que hace el robot —
> navega, percibe, levanta, escucha y reporta. Sin prefijo `puzzlebot_` redundante.

---

## Sim vs Real

Toda la pila soporta dos modos. El argumento `sim:=true/false` en el launch raíz
controla qué implementación de hardware se carga.

```bash
ros2 launch bringup full.launch.py sim:=true    # Gazebo
ros2 launch bringup full.launch.py sim:=false   # Jetson Nano + hardware real
```

### Hardware Abstraction Layer (HAL)

Los paquetes que tocan hardware exponen una interfaz Python común y cargan
la implementación según el parámetro ROS `use_sim_time`:

```
lifting/
└── lifting/
    ├── hal/
    │   ├── base.py          # interfaz abstracta GpioDriver
    │   ├── jetson.py        # Jetson.GPIO — solo carga en hardware real
    │   └── mock.py          # logging únicamente — carga en simulación
    └── lifting_node.py      # selecciona hal/ según use_sim_time

perception/
└── perception/
    ├── hal/
    │   ├── base.py          # interfaz abstracta CameraSource
    │   ├── ros_camera.py    # suscribe /cam_img (real o Gazebo, mismo topic)
    │   └── mock_camera.py   # genera imágenes sintéticas para pruebas unitarias
    └── ...
```

### Modos por paquete

| Paquete | Sim | Real |
|---|---|---|
| `controller` | Gazebo differential drive plugin | H-bridge + encoders físicos |
| `localization` | Odom de Gazebo como ground-truth | EKF con encoders reales |
| `slam` | Gazebo LiDAR plugin (`/scan`) | LiDAR físico |
| `navigation` | Misma lógica, mapa del SLAM sim | Mapa pre-guardado real |
| `perception` | Cámara Gazebo (`/cam_img`) | Cámara USB |
| `lifting` | `hal/mock.py` (no GPIO) | `hal/jetson.py` + FPGA |
| `voice_control` | Archivos .wav de prueba | Micrófono USB en Jetson |
| `dashboard` | Igual — consume topics ROS2 | Igual |

---

## Paquetes existentes — limpieza

Antes de crear los nuevos, renombrar:

```bash
# En src/
mv puzzlebot_bringup          bringup
mv puzzlebot_controller       controller
mv puzzlebot_description      description
mv puzzlebot_localization_cpp localization
mv puzzlebot_slam              slam

# Actualizar en cada package.xml: <name>puzzlebot_X</name> → <name>X</name>
# Actualizar referencias en CMakeLists.txt y setup.py/setup.cfg
# Actualizar imports internos en archivos Python
```

---

## Módulo 1 — `navigation`

**Evaluable:** MR Navegación (5%)  
**Responsabilidad:** Llevar el robot de su pose actual a un waypoint, evadiendo obstáculos.

### Estructura

```
navigation/
├── navigation/
│   ├── __init__.py
│   ├── astar.py               # A* puro sobre OccupancyGrid (sin ROS, testeable)
│   ├── bug1.py                # Bug1 reactivo: contornea obstáculo y retoma ruta
│   ├── path_follower.py       # PID heading + velocidad lineal → /cmd_vel
│   └── navigation_node.py    # Nodo ROS2: orquesta A* → follower → Bug1
├── config/
│   ├── nav_params.yaml        # Tolerancias, PID gains, velocidades máx
│   └── waypoints.yaml         # Poses fijas de cada zona
├── launch/
│   └── navigation.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Topics

| Topic | Tipo | Rol |
|---|---|---|
| `/robot_pose` | `PoseStamped` | Entrada — pose actual (EKF) |
| `/map` | `OccupancyGrid` | Entrada — mapa para A* |
| `/scan` | `LaserScan` | Entrada — obstáculos para Bug1 |
| `/cmd_vel` | `Twist` | Salida — velocidades |
| `/navigation/goal` | `PoseStamped` | Entrada — destino pedido por mission_control |
| `/navigation/status` | `String` | Salida — `NAVIGATING` · `ARRIVED` · `STUCK` |

### Lógica

```
mission_control publica /navigation/goal
    ↓
A* planifica sobre OccupancyGrid inflado (inflation_radius configurable)
    ↓
PID sigue ruta waypoint a waypoint (pure pursuit simplificado)
    ↓
LiDAR detecta obstáculo no mapeado → Bug1 toma control
    ↓
Bug1 contornea obstáculo → retoma ruta A*
    ↓
|error_pose| < tolerance → publica ARRIVED
```

### `waypoints.yaml`

```yaml
waypoints:
  truck_1:  {x: 1.50, y: 0.50, theta: 0.00}
  truck_2:  {x: 1.50, y: 1.50, theta: 0.00}
  truck_3:  {x: 1.50, y: 2.50, theta: 0.00}
  rack_1:   {x: 0.50, y: 0.50, theta: 1.57}
  rack_2:   {x: 0.50, y: 1.50, theta: 1.57}
  roller_1: {x: 2.00, y: 1.00, theta: 3.14}
  roller_2: {x: 2.00, y: 2.00, theta: 3.14}
```

---

## Módulo 2 — `mission_control`

**Evaluable:** Integración general del reto  
**Librería:** [YASMIN](https://github.com/uleroboticsgroup/yasmin)

### Estructura

```
mission_control/
├── mission_control/
│   ├── __init__.py
│   ├── states/
│   │   ├── waiting_command.py       # Espera misión de dashboard o voice_control
│   │   ├── navigating.py            # Publica goal → espera ARRIVED
│   │   ├── aligning.py              # PID visual, espera aligned
│   │   ├── picking_floor.py         # Lifter nivel 3 — pallet de piso
│   │   ├── picking_rack.py          # Lifter nivel 5 — rack nivel 2
│   │   ├── picking_truck.py         # Secuencia específica para camión
│   │   └── placing_pallet.py        # Baja lifter, deposita, retrocede
│   ├── mission_parser.py            # JSON → goals concretos
│   └── state_machine_node.py       # Construye y corre SM con YASMIN
├── config/
│   └── sm_params.yaml
├── launch/
│   └── mission_control.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Diagrama de estados

```
                   ┌────────────────────┐
                   │  WAITING_COMMAND   │◄─────────────────────┐
                   └───────┬────────────┘                      │
                   [mission_received]                          │
                            ↓                                  │
                   ┌────────────────────┐                      │
                   │  NAVIGATING_SOURCE │──[stuck]──► ERROR    │
                   └───────┬────────────┘                      │
                        [arrived]                              │
                            ↓                                  │
                   ┌────────────────────┐                      │
                   │  ALIGNING_TO_PALLET│                      │
                   └───────┬────────────┘                      │
                        [aligned]                              │
                            ↓                                  │
          ┌─────────────────┴──────────────────┐               │
          ▼                 ▼                  ▼               │
   PICKING_FLOOR      PICKING_RACK      PICKING_TRUCK          │
          └─────────────────┬──────────────────┘               │
                        [picked]                               │
                            ↓                                  │
                   ┌────────────────────┐                      │
                   │  NAVIGATING_DEST   │                      │
                   └───────┬────────────┘                      │
                        [arrived]                              │
                            ↓                                  │
                   ┌────────────────────┐                      │
                   │  ALIGNING_TO_DEST  │                      │
                   └───────┬────────────┘                      │
                        [aligned]                              │
                            ↓                                  │
                   ┌────────────────────┐                      │
                   │  PLACING_PALLET    │──────────────────────┘
                   └────────────────────┘

Cualquier estado → [pause_cmd] → PAUSED → [resume_cmd] → estado anterior
Cualquier estado → [stop_cmd]  → WAITING_COMMAND
```

### Formato de misión (JSON)

```json
{
  "pallet_id": "A3",
  "source": "rack_1",
  "source_level": 2,
  "destination": "truck_2"
}
```

---

## Módulo 3 — `perception`

**Evaluables:** MR Detección Aruco (5%), E80 Detección + alineación (5%)

### Estructura

```
perception/
├── perception/
│   ├── __init__.py
│   ├── hal/
│   │   ├── base.py              # CameraSource abstracta
│   │   ├── ros_camera.py        # Suscribe /cam_img (real + Gazebo)
│   │   └── mock_camera.py       # Genera frames sintéticos para tests
│   ├── aruco_detector.py        # cv2.aruco → PoseArray /aruco_poses
│   ├── trailer_detector.py      # CNN/YOLO → BoundingBox2D /trailer_detection
│   ├── pallet_detector.py       # HSV mask + contornos → /pallet_detection
│   └── alignment_node.py        # PID visual: centroide → /cmd_vel + /alignment_error
├── models/
│   └── trailer_model.pt         # Modelo entrenado (MobileNet/YOLO)
├── config/
│   ├── camera_params.yaml        # Matriz K, distorsión D
│   └── vision_params.yaml        # HSV ranges, PID gains alineación
├── launch/
│   └── perception.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Alineación con pallet

```
/cam_img → HSV filter → máscara binaria → findContours
→ bounding rect → centroide (cx, cy)
→ error_x = cx - img_width/2    → PID angular:  ω = Kp_ang · error_x
→ error_y = cy - img_height/2   → PID lineal:   v = Kp_lin · error_y
→ /cmd_vel
→ |error| < umbral → /alignment_error con aligned=True
```

### Aruco

- `cv2.aruco.detectMarkers()` con `DICT_4X4_50`
- `cv2.aruco.estimatePoseSingleMarkers()` con calibración de cámara
- Publica `PoseArray` en `/aruco_poses`

---

## Módulo 4 — `lifting`

**Evaluable:** M2 FPGA control servo (8%)

### Estructura

```
lifting/
├── lifting/
│   ├── __init__.py
│   ├── hal/
│   │   ├── base.py          # GpioDriver abstracto: set_level(n: int)
│   │   ├── jetson.py        # Jetson.GPIO — 3 pines, board numbering
│   │   └── mock.py          # Solo logging — para simulación y tests
│   └── lifting_node.py      # Suscribe /lifter_level (UInt8 0-7), llama hal.set_level()
├── config/
│   └── lifting_params.yaml  # pin_bit0, pin_bit1, pin_bit2, niveles operacionales
├── launch/
│   └── lifting.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### HAL — implementación real

```python
# hal/jetson.py
import Jetson.GPIO as GPIO

PINS = {0: 11, 1: 13, 2: 15}   # bit → pin board numbering

def set_level(level: int) -> None:   # level ∈ [0, 7]
    for bit, pin in PINS.items():
        GPIO.output(pin, GPIO.HIGH if (level >> bit) & 1 else GPIO.LOW)
```

### Niveles operacionales

| Operación | Nivel | Binario |
|---|---|---|
| Reposo | 0 | `000` |
| Transporte (viajando) | 1 | `001` |
| Pick pallet piso | 3 | `011` |
| Carry (cargando y viajando) | 4 | `100` |
| Pick rack nivel 2 | 5 | `101` |
| Dejar en destino | 3 | `011` |

---

## Módulo 5 — `voice_control`

**Evaluable:** M4 Comandos de voz (8%)  
**Referencia:** [JordanPalafox/Practica-1](https://github.com/JordanPalafox/Practica-1)

### Estructura

```
voice_control/
├── voice_control/
│   ├── __init__.py
│   ├── record.py              # Captura audio 16kHz con pyaudio
│   ├── endpoint.py            # VAD por energía adaptativa
│   ├── preprocess.py          # Pre-énfasis α=0.97 + ventana Hamming
│   ├── lpc.py                 # LPC orden 12 → LSF (Levinson-Durbin)
│   ├── vq.py                  # LBG clustering, distancia Itakura-Saito
│   ├── train.py               # Script offline: genera codebooks por palabra
│   ├── recognizer.py          # Clasificador en tiempo real
│   ├── command_parser.py      # Secuencia de palabras → misión JSON
│   └── voice_node.py          # Nodo ROS2: publica /voice_command + /mission
├── codebooks/                 # .pkl por palabra (generados por train.py)
├── config/
│   └── voice_params.yaml      # sample_rate, lpc_order, vq_k, umbral_silencio
├── scripts/
│   └── collect_training_data.py   # Graba 15 repeticiones por palabra
├── launch/
│   └── voice_control.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Vocabulario (13 palabras)

`start` `stop` `pause` `next` `ve` `rack` `camion` `roller` `nivel` `uno` `dos` `recoge` `deja`

### Pipeline de reconocimiento

```
Micrófono (16kHz)
    ↓
VAD — segmento de voz (energía > umbral adaptativo)
    ↓
Pre-énfasis (α=0.97) + Hamming windowing (25ms, stride 10ms)
    ↓
LPC orden 12 → coeficientes LSF por frame
    ↓
VQ: codebook más cercano (distancia Itakura-Saito promedio)
    ↓
Palabra reconocida → buffer de secuencia
    ↓
command_parser → gramática → misión JSON → /mission
```

### Gramática de comandos

```
START / STOP / PAUSE / NEXT → cmd directo
VE RACK NIVEL (UNO|DOS) RECOGE  → {"source": "rack_N", "action": "pick"}
VE CAMION DEJA                   → {"dest": "truck",    "action": "place"}
VE ROLLER (RECOGE|DEJA)         → {"zone": "roller",   "action": ...}
```

### Entrenamiento (una vez, offline)

```bash
ros2 run voice_control collect_training_data   # 15 repeticiones por palabra
ros2 run voice_control train                   # genera codebooks/*.pkl (k=32)
```

---

## Módulo 6 — `dashboard`

**Evaluable:** E80 Interfaz web (5%)

### Estructura

```
dashboard/
├── dashboard/
│   ├── __init__.py
│   ├── ros_bridge.py      # Thread suscriptor → expone datos vía queue thread-safe
│   └── dashboard_node.py  # Nodo ROS2: lanza Flask en thread separado
├── web/
│   ├── app.py             # Flask: REST API + WebSocket (flask-socketio)
│   ├── templates/
│   │   └── index.html     # Dashboard: mapa, cámara, estado, telemetría
│   └── static/
│       ├── main.js        # SocketIO client: actualiza UI en tiempo real
│       └── style.css
├── launch/
│   └── dashboard.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### REST API

| Método | Ruta | Descripción |
|---|---|---|
| `GET` | `/` | Dashboard HTML |
| `GET` | `/api/state` | Estado actual del robot |
| `GET` | `/api/map` | OccupancyGrid como PNG |
| `POST` | `/api/mission` | Enviar misión JSON → `/mission` topic |
| `GET` | `/video_feed` | MJPEG stream de `/cam_img` |

### WebSocket events

| Evento | Datos | Frecuencia |
|---|---|---|
| `robot_pose` | `{x, y, theta}` | 5 Hz |
| `robot_state` | `{state, mission}` | on change |
| `telemetry` | `{v, w, battery}` | 1 Hz |

---

## Simulación — Gazebo

### Mundo del almacén

Agregar en `description/worlds/warehouse.world`:
- Tres camiones (modelos SDF rectangulares)
- Cuatro racks (dos niveles, posiciones medidas del diseño real)
- Dos rollers (mesas planas)
- Pallets como cubos de colores (rojo, azul, verde para identificación HSV)
- Iluminación y paredes

### Plugins requeridos en URDF/SDF

```xml
<!-- Diferencial -->
<plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <publish_odom>true</publish_odom>
  <publish_tf>true</publish_tf>
</plugin>

<!-- LiDAR -->
<plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
  <ros><remapping>~/out:=/scan</remapping></ros>
</plugin>

<!-- Cámara -->
<plugin name="camera" filename="libgazebo_ros_camera.so">
  <ros><remapping>~/image_raw:=/cam_img</remapping></ros>
</plugin>
```

### Launch de simulación

`bringup/launch/sim.launch.py` levanta:
1. `gazebo` con `warehouse.world`
2. `robot_state_publisher` con el URDF
3. `slam` (modo simulación, usa `/scan` de Gazebo)
4. `localization` (EKF con odom de Gazebo)
5. `navigation`, `perception`, `mission_control`, `dashboard`
6. `lifting` con `use_mock_gpio:=true`
7. `rviz2` con config predefinida

---

## Launch principal

`bringup/launch/full.launch.py` — punto de entrada único:

```python
# Argumento: sim:=true/false
# sim=true  → lanza sim.launch.py  (Gazebo + mocks)
# sim=false → lanza real.launch.py (hardware Jetson)
```

### Stack completo (ambos modos)

```
slam_node           (slam)
ekf_node            (localization)
navigation_node     (navigation)
alignment_node      (perception)
lifting_node        (lifting)       ← mock o real según sim flag
voice_node          (voice_control) ← omitido en sim si no hay mic
state_machine_node  (mission_control)
dashboard_node      (dashboard)
```

---

## Dependencias

```bash
pip install yasmin               # State machine ROS2
pip install pyaudio              # Captura de audio
pip install scikit-learn         # VQ clustering (LBG via KMeans)
pip install flask flask-socketio # Dashboard web
pip install Jetson.GPIO          # GPIO Jetson Nano (solo hardware real)
# OpenCV, NumPy, SciPy — incluidos con ROS2 Humble
```

---

## Orden de implementación

### Fase 0 — Limpieza (1 día)
- [ ] Renombrar paquetes existentes (quitar prefijo `puzzlebot_`)
- [ ] Actualizar `package.xml` y `setup.py` en cada paquete renombrado
- [ ] Crear mundo Gazebo básico del almacén en `description/worlds/`
- [ ] Verificar build limpio: `colcon build --symlink-install`

### Fase 1 — Hardware crítico + Sim base (2-3 días)
- [ ] `lifting` — `hal/mock.py` primero, luego `hal/jetson.py`, validar GPIO → FPGA
- [ ] `perception` / `pallet_detector.py` + `alignment_node.py` — alineación básica en sim
- [ ] Launch de simulación funcional con Gazebo

### Fase 2 — Navegación (3-4 días)
- [ ] `navigation` / `astar.py` — unit test con mapa sintético, sin ROS
- [ ] `navigation` / `navigation_node.py` — integrar con mapa SLAM en sim
- [ ] `navigation` / `bug1.py` — evasión reactiva en sim con obstáculos dinámicos
- [ ] Medir waypoints reales y actualizar `waypoints.yaml`

### Fase 3 — Integración con mission_control (2-3 días)
- [ ] `mission_control` — implementar `WAITING → NAVIGATING → ARRIVED` primero
- [ ] Añadir `ALIGNING → PICKING → NAVIGATING_DEST → PLACING`
- [ ] Probar ciclo completo en sim (rack_1 → truck_1)
- [ ] Integrar `lifting` real en flujo de pick/place en hardware

### Fase 4 — Percepción avanzada (2 días)
- [ ] `perception` / `aruco_detector.py` — evaluable MR
- [ ] `perception` / `trailer_detector.py` — entrenar CNN para tráiler
- [ ] Integrar detección de tráiler en `picking_truck.py`

### Fase 5 — Voz y Dashboard (2-3 días)
- [ ] `voice_control` — grabar datos de entrenamiento (15 reps × 13 palabras)
- [ ] Entrenar codebooks, validar reconocimiento offline
- [ ] Integrar `voice_node` con `mission_control`
- [ ] `dashboard` — Flask + SocketIO, MJPEG stream, envío de misiones

### Fase 6 — Prueba integrada y ajuste (días finales)
- [ ] Ciclo completo en hardware real: voz / web → SM → navega → alinea → pick → navega → place
- [ ] Ajustar PID gains con datos reales
- [ ] Ajustar HSV ranges con iluminación real
- [ ] Ajustar waypoints con mediciones físicas de la pista

---

## Notas de diseño

- **No Nav2**: pila de navegación propia. A* sobre mapa del SLAM existente.
- **No Whisper**: LPC + VQ con entrenamiento offline de 15 grabaciones por palabra.
- **Mapa pre-guardado**: SLAM corre en modo corrección (localización), no remapea de cero.
- **Waypoints fijos**: posiciones medidas una vez, guardadas en YAML.
- **YASMIN**: mejor soporte ROS2 Humble que SMACH, tiene visualizador propio (`yasmin_viewer`).
- **Jetson.GPIO**: board numbering, 3 pines a FPGA para encoding binario del lifter.
- **Pallets por color/forma**: HSV mask + contornos — no requiere QR ni Aruco en pallet.
- **HAL pattern**: aísla el hardware para que todo sea simulable sin cambios de lógica.
