# Plan de Implementación — Puzzlebot AMR Montacargas

**Reto:** TE3003B · ITESM Campus Monterrey · FJ2026  
**Robot:** Puzzlebot (diferencial) con lifter tipo montacargas controlado por FPGA  

---

## Estructura final del repositorio

```
src/
├── puzzlebot_bringup/              ✅ Existente — launch del robot
├── puzzlebot_controller/           ✅ Existente — cinemática diferencial + odometría
├── puzzlebot_description/          ✅ Existente — URDF
├── puzzlebot_localization_cpp/     ✅ Existente — EKF C++ + ICP 2D
├── puzzlebot_slam/                 ✅ Existente — SLAM ICP + OccupancyGrid Python
│
├── puzzlebot_navigation/           🔨 Crear
├── puzzlebot_state_machine/        🔨 Crear
├── puzzlebot_vision/               🔨 Crear
├── puzzlebot_lifter/               🔨 Crear
├── puzzlebot_voice/                🔨 Crear
└── puzzlebot_web/                  🔨 Crear
```

---

## Módulo 1 — `puzzlebot_navigation`

**Evaluable:** MR Navegación (5%)  
**Responsabilidad:** Llevar el robot de su pose actual a un waypoint, evadiendo obstáculos.

### Archivos

```
puzzlebot_navigation/
├── puzzlebot_navigation/
│   ├── __init__.py
│   ├── astar.py               # Algoritmo A* puro (sin ROS)
│   ├── bug1.py                # Bug1 algorithm para evasión reactiva
│   ├── path_follower.py       # PID de seguimiento: error de heading → cmd_vel
│   └── navigation_node.py    # Nodo ROS2: subscribe /robot_pose + /map, publica /cmd_vel
├── config/
│   ├── nav_params.yaml        # Tolerancias, velocidades, PID gains
│   └── waypoints.yaml         # Poses fijas de cada zona (camiones, racks, rollers)
├── launch/
│   └── navigation.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Interfaces

| Topic | Tipo | Rol |
|---|---|---|
| `/robot_pose` | `PoseStamped` | Entrada — pose actual del robot |
| `/map` | `OccupancyGrid` | Entrada — mapa para A* |
| `/scan` | `LaserScan` | Entrada — obstáculos para Bug1 |
| `/cmd_vel` | `Twist` | Salida — velocidades |
| `/navigation/goal` | `PoseStamped` | Entrada — destino solicitado por SM |
| `/navigation/status` | `String` | Salida — `NAVIGATING`, `ARRIVED`, `STUCK` |

### Lógica

1. SM publica goal en `/navigation/goal`
2. A* planifica sobre el OccupancyGrid inflado (inflation layer manual)
3. PID sigue la ruta waypoint a waypoint (pure pursuit simplificado)
4. Si LiDAR detecta obstáculo no mapeado → Bug1 toma control
5. Bug1 sigue el contorno del obstáculo hasta retomar la ruta A*
6. Al llegar (error < tolerancia) publica `ARRIVED`

### `waypoints.yaml` (estructura)

```yaml
waypoints:
  truck_1:    {x: 1.50, y: 0.50, theta: 0.00}
  truck_2:    {x: 1.50, y: 1.50, theta: 0.00}
  truck_3:    {x: 1.50, y: 2.50, theta: 0.00}
  rack_1:     {x: 0.50, y: 0.50, theta: 1.57}
  rack_2:     {x: 0.50, y: 1.50, theta: 1.57}
  roller_1:   {x: 2.00, y: 1.00, theta: 3.14}
  roller_2:   {x: 2.00, y: 2.00, theta: 3.14}
```

---

## Módulo 2 — `puzzlebot_state_machine`

**Evaluable:** Integración general del reto  
**Librería:** [YASMIN](https://github.com/uleroboticsgroup/yasmin) — diseñada para ROS2 Python

### Archivos

```
puzzlebot_state_machine/
├── puzzlebot_state_machine/
│   ├── __init__.py
│   ├── states/
│   │   ├── waiting_command.py      # Espera misión de web o voz
│   │   ├── navigating.py           # Publica goal a navigation, espera ARRIVED
│   │   ├── aligning.py             # PID visual, espera aligned de vision
│   │   ├── picking_lower.py        # Lifter nivel 3, avanza, recoge pallet de piso
│   │   ├── picking_upper.py        # Lifter nivel 5, recoge pallet de rack nivel 2
│   │   ├── picking_from_truck.py   # Secuencia específica para recoger desde camión
│   │   └── placing_pallet.py       # Baja lifter, deposita pallet, retrocede
│   ├── mission_parser.py           # Convierte string JSON de misión a goals concretos
│   └── state_machine_node.py      # Construye y corre la SM con YASMIN
├── config/
│   └── sm_params.yaml
├── launch/
│   └── state_machine.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Diagrama de transiciones

```
                    ┌─────────────────────┐
                    │   WAITING_COMMAND   │◄──────────────────────┐
                    └────────┬────────────┘                       │
                    [mission_received]                            │
                             ↓                                    │
                    ┌─────────────────────┐                       │
                    │  NAVIGATING_SOURCE  │ ──[stuck]──► ERROR    │
                    └────────┬────────────┘                       │
                         [arrived]                                │
                             ↓                                    │
                    ┌─────────────────────┐                       │
                    │  ALIGNING_TO_PALLET │                       │
                    └────────┬────────────┘                       │
                         [aligned]                                │
                             ↓                                    │
               ┌─────────────┴──────────────┐                    │
               ▼                            ▼                    │
       [rack nivel 1]              [rack nivel 2 / camión]       │
    PICKING_LOWER              PICKING_UPPER / PICKING_FROM_TRUCK│
               └─────────────┬──────────────┘                    │
                          [picked]                                │
                             ↓                                    │
                    ┌─────────────────────┐                       │
                    │   NAVIGATING_DEST   │                       │
                    └────────┬────────────┘                       │
                         [arrived]                                │
                             ↓                                    │
                    ┌─────────────────────┐                       │
                    │   ALIGNING_TO_DEST  │                       │
                    └────────┬────────────┘                       │
                         [aligned]                                │
                             ↓                                    │
                    ┌─────────────────────┐                       │
                    │   PLACING_PALLET    │ ──────────────────────┘
                    └─────────────────────┘
                    
Cualquier estado → [pause_cmd] → PAUSED → [resume_cmd] → estado anterior
Cualquier estado → [stop_cmd]  → WAITING_COMMAND
```

### Formato de misión

```json
{
  "pallet_id": "A3",
  "source": "rack_1",
  "source_level": 2,
  "destination": "truck_2"
}
```

---

## Módulo 3 — `puzzlebot_vision`

**Evaluables:** MR Detección Aruco (5%), E80 Detección + alineación (5%)

### Archivos

```
puzzlebot_vision/
├── puzzlebot_vision/
│   ├── __init__.py
│   ├── aruco_detector.py      # Detecta markers, publica ID + pose 3D → /aruco_poses
│   ├── trailer_detector.py    # CNN/YOLO para detectar tráiler → /trailer_detection
│   ├── pallet_detector.py     # HSV color mask + contornos → /pallet_detection
│   └── alignment_node.py     # PID visual: error centroide → /alignment_error + /cmd_vel
├── models/
│   └── trailer_model.pt       # Modelo entrenado YOLO/MobileNet para tráiler
├── config/
│   ├── camera_params.yaml     # Matriz intrínseca K, distorsión D
│   └── vision_params.yaml     # HSV ranges, PID gains alineación
├── launch/
│   └── vision.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Alineación con pallet (color/forma)

```
Imagen → HSV filter → máscara binaria → findContours
→ bounding rect → centroide (cx, cy)
→ error_x = cx - img_width/2
→ error_y = cy - img_height/2
→ PID angular:  w = Kp_ang * error_x
→ PID lineal:   v = Kp_lin * error_y  (negativo = acercarse)
→ Publicar /cmd_vel
→ Si |error| < umbral: publicar aligned=True
```

### Aruco (evaluable MR)

- `cv2.aruco.detectMarkers()` con dict `DICT_4X4_50`
- `cv2.aruco.estimatePoseSingleMarkers()` con calibración de cámara
- Publica `PoseArray` en `/aruco_poses` con ID en header

---

## Módulo 4 — `puzzlebot_lifter`

**Evaluable:** M2 FPGA control servo (8%)

### Archivos

```
puzzlebot_lifter/
├── puzzlebot_lifter/
│   ├── __init__.py
│   └── lifter_node.py    # Suscribe /lifter_level (UInt8 0-7), escribe 3 pines GPIO
├── config/
│   └── lifter_params.yaml  # pin_bit0, pin_bit1, pin_bit2, nivel_pick_lower, nivel_pick_upper
├── launch/
│   └── lifter.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Lógica GPIO (Jetson.GPIO)

```python
import Jetson.GPIO as GPIO

PINS = {0: 11, 1: 13, 2: 15}   # pin_bit0, pin_bit1, pin_bit2 (board numbering)

def set_level(level: int):      # level ∈ [0, 7]
    for bit, pin in PINS.items():
        GPIO.output(pin, GPIO.HIGH if (level >> bit) & 1 else GPIO.LOW)
```

### Niveles por operación

| Operación | Nivel | Binario |
|---|---|---|
| Transporte (viajar) | 1 | `001` |
| Pick pallet piso | 3 | `011` |
| Carry (cargando) | 4 | `100` |
| Pick rack nivel 2 | 5 | `101` |
| Dejar en tráiler | 3 | `011` |
| Reposo | 0 | `000` |

---

## Módulo 5 — `puzzlebot_voice`

**Evaluable:** M4 Comandos de voz (8%)  
**Referencia:** [JordanPalafox/Practica-1](https://github.com/JordanPalafox/Practica-1) — LPC + VQ, sin Whisper

### Archivos

```
puzzlebot_voice/
├── puzzlebot_voice/
│   ├── __init__.py
│   ├── record.py          # Captura audio 16kHz con pyaudio
│   ├── endpoint.py        # VAD por energía adaptativa (detecta inicio/fin de palabra)
│   ├── preprocess.py      # Pre-énfasis + ventana Hamming
│   ├── lpc.py             # LPC orden 12 → LSF (Autocorrelación + Levinson-Durbin)
│   ├── vq.py              # LBG clustering, distancia Itakura-Saito
│   ├── train.py           # Script offline: genera codebooks por palabra
│   ├── recognizer.py      # Clasificador en tiempo real: frame → VQ dist → palabra
│   ├── command_parser.py  # Secuencia de palabras → misión JSON
│   └── voice_node.py     # Nodo ROS2: corre recognizer, publica /voice_command + /mission
├── codebooks/             # Archivos .pkl generados por train.py (uno por palabra)
├── config/
│   └── voice_params.yaml  # sample_rate, lpc_order, vq_k, umbral_silencio
├── scripts/
│   └── collect_training_data.py   # Script interactivo para grabar ejemplos
├── launch/
│   └── voice.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Vocabulario objetivo (13 palabras)

`start` `stop` `pause` `next` `ve` `rack` `camion` `roller` `nivel` `uno` `dos` `recoge` `deja`

### Pipeline de reconocimiento

```
Micrófono (16kHz)
    ↓
VAD — detecta segmento de voz (energía > umbral adaptativo)
    ↓
Pre-énfasis (α=0.97) + Hamming windowing (25ms, stride 10ms)
    ↓
LPC orden 12 → coeficientes LSF por frame
    ↓
VQ: busca codebook más cercano (distancia Itakura-Saito promedio)
    ↓
Palabra reconocida → buffer de secuencia
    ↓
command_parser: detecta patrón gramatical → misión JSON
```

### Gramática de comandos

```
START           → {"cmd": "start"}
STOP            → {"cmd": "stop"}
PAUSE           → {"cmd": "pause"}
NEXT            → {"cmd": "next"}
VE + RACK + NIVEL + (UNO|DOS) + RECOGE  → {"cmd": "mission", "source": "rack_N", "action": "pick"}
VE + CAMION + DEJA                       → {"cmd": "mission", "dest": "truck", "action": "place"}
VE + ROLLER + (RECOGE|DEJA)             → {"cmd": "mission", "zone": "roller", "action": ...}
```

### Entrenamiento (offline, una vez)

```bash
# 1. Grabar 15 repeticiones por palabra
ros2 run puzzlebot_voice collect_training_data

# 2. Entrenar codebooks (k=32 vectors)
ros2 run puzzlebot_voice train

# Los codebooks quedan en codebooks/*.pkl
```

---

## Módulo 6 — `puzzlebot_web`

**Evaluable:** E80 Interfaz web (5%)

### Archivos

```
puzzlebot_web/
├── puzzlebot_web/
│   ├── __init__.py
│   ├── ros_bridge.py      # Thread que suscribe topics ROS2 y expone datos vía queue
│   └── web_node.py       # Nodo ROS2 que lanza Flask en thread separado
├── web/
│   ├── app.py             # Flask app: REST API + WebSocket (flask-socketio)
│   ├── templates/
│   │   └── index.html     # Dashboard: mapa, cámara, estado, telemetría
│   └── static/
│       ├── main.js        # SocketIO client, actualiza UI en tiempo real
│       └── style.css
├── launch/
│   └── web.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

### Endpoints REST

| Método | Ruta | Descripción |
|---|---|---|
| `GET` | `/` | Dashboard HTML |
| `GET` | `/api/state` | Estado actual del robot |
| `GET` | `/api/map` | OccupancyGrid como PNG |
| `POST` | `/api/mission` | Enviar misión JSON → `/mission` topic |
| `GET` | `/video_feed` | MJPEG stream de `/cam_img` |

### WebSocket events (flask-socketio)

| Evento | Datos | Frecuencia |
|---|---|---|
| `robot_pose` | `{x, y, theta}` | 5 Hz |
| `robot_state` | `{state, mission}` | on change |
| `telemetry` | `{v, w, battery}` | 1 Hz |

---

## Launch principal

Crear `puzzlebot_bringup/launch/amr_full.launch.py` que levanta todo el stack:

```
slam_node          (puzzlebot_slam)
ekf_node           (puzzlebot_localization_cpp)
navigation_node    (puzzlebot_navigation)
vision_node        (puzzlebot_vision)
lifter_node        (puzzlebot_lifter)
voice_node         (puzzlebot_voice)
state_machine_node (puzzlebot_state_machine)
web_node           (puzzlebot_web)
```

---

## Dependencias externas a instalar

```bash
pip install yasmin           # State machine ROS2
pip install pyaudio          # Audio capture para voz
pip install scikit-learn     # VQ clustering (LBG via KMeans)
pip install flask flask-socketio  # Web interface
pip install Jetson.GPIO      # GPIO para FPGA lifter (solo en Jetson Nano)
# OpenCV ya viene con ROS2 Humble
# NumPy, SciPy ya vienen con ROS2
```

---

## Orden de implementación sugerido

### Fase 1 — Hardware crítico
1. `puzzlebot_lifter` — validar GPIO → FPGA en físico
2. `puzzlebot_vision` / `pallet_detector.py` + `alignment_node.py` — alineación básica

### Fase 2 — Navegación
3. `puzzlebot_navigation` / `astar.py` — probar en simulación con el mapa SLAM existente
4. `puzzlebot_navigation` / `bug1.py` — integrar evasión
5. Configurar `waypoints.yaml` con mediciones reales de la pista

### Fase 3 — Integración con SM
6. `puzzlebot_state_machine` — primero solo `WAITING → NAVIGATING → ARRIVED`
7. Expandir estados de pick/place con el lifter real
8. Integrar alineación visual en el flujo

### Fase 4 — Voz y Web
9. `puzzlebot_voice` — grabar datos, entrenar, integrar con SM
10. `puzzlebot_web` — interfaz + envío de misiones

### Fase 5 — Evaluables pendientes
11. `puzzlebot_vision` / `aruco_detector.py` — evaluable MR
12. `puzzlebot_vision` / `trailer_detector.py` — CNN para evaluable E80

---

## Notas de diseño

- **No Nav2**: toda la pila es propia. A* sobre el mapa del SLAM existente.
- **No Whisper**: LPC + VQ. Entrenamiento offline con 15 grabaciones por palabra.
- **Mapa pre-guardado**: SLAM corre en modo corrección/localización, no remapea de cero.
- **Zonas fijas**: los waypoints se miden una vez y quedan en YAML, no hay descubrimiento dinámico.
- **YASMIN** sobre SMACH: mejor soporte ROS2 Humble, visualizador incluido (`yasmin_viewer`).
- **Jetson.GPIO** para los 3 pines al FPGA — board numbering, setup como OUTPUT.
- La cámara detecta pallets por **color/forma** (HSV mask + contornos) — no requiere QR ni Aruco en pallet.
