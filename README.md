# Puzzlebot Control UI

Interfaz web para teleoperación y monitoreo de un robot Puzzlebot usando ROS 2 Humble.

---

## Arquitectura

```
ROS 2 (robot)
    │  /odom  /image_result  /cmd_vel
    ▼
ros2-grpc-wrapper.py   ← Terminal 1
    │  HTTP REST  :7043   (GET /odom, GET /image, POST /cmd_vel)
    │  gRPC       :7042   (GetMultCoords, GetImageResult)
    ▼
app.py  (Flask)        ← Terminal 2
    │  :8002
    ▼
Navegador  →  http://localhost:8002
```

| Componente | Archivo | Puerto |
|---|---|---|
| ROS 2 wrapper | `PY-RPC-Wrapper-Server-Linux/ros2-grpc-wrapper.py` | HTTP 7043 · gRPC 7042 |
| Servidor web Flask | `FLASK-REST-Call-Linux/app.py` | 8002 |
| UI React | `FLASK-REST-Call-Linux/templates/result.html` | — |

---

## Requisitos previos

- ROS 2 Humble instalado y con `source /opt/ros/humble/setup.bash`
- Python 3.10+
- Paquetes Python: `flask`, `requests`, `grpcio`, `grpcio-tools`, `opencv-python`, `cv_bridge`, `rclpy`

---

## Cómo lanzar

### Terminal 1 — ROS 2 Wrapper (debe correr en el entorno ROS 2)

> **Importante:** si tienes Miniconda/Anaconda activo, desactívalo primero.
> ROS 2 Humble requiere Python 3.10 del sistema; conda usa Python 3.12 y rompe `rclpy`.

```bash
conda deactivate
source /opt/ros/humble/setup.bash
cd /home/rosendorios/Desktop/Modulo3/UI/PY-RPC-Wrapper-Server-Linux
python3 ros2-grpc-wrapper.py
```

Este proceso:
- Crea un nodo ROS 2 `object_position_wrapper`
- Se suscribe a `/odom` y `/image_result`
- Publica en `/cmd_vel` a 20 Hz
- Expone un servidor HTTP en el puerto **7043** y un servidor gRPC en el puerto **7042**

### Terminal 2 — Servidor Flask (UI web)

```bash
cd /home/rosendorios/Desktop/Modulo3/UI/FLASK-REST-Call-Linux
python3 app.py
```

Este proceso:
- Sirve la interfaz web en `http://localhost:8002`
- Hace de proxy entre el navegador y el wrapper:
  - `GET /api/odom` → obtiene posición del robot
  - `POST /api/cmd_vel` → envía comandos de velocidad
  - `GET /api/camera` → stream MJPEG de la cámara

### Abrir la UI

```
http://localhost:8002
```

---

## Pestañas de la interfaz

| Pestaña | Descripción |
|---|---|
| **MAPA** | Vista de planta con LIDAR sintético, pose real del robot y waypoints. Haz clic en el mapa para añadir waypoints. |
| **CÁMARA** | Stream en vivo MJPEG desde `/image_result`. Muestra "Sin señal" si el wrapper no está corriendo. |
| **TELEOP** | Control por teclado, barras de velocidad, batería y E-Stop. |

---

## Controles de teclado (pestaña TELEOP)

| Tecla | Acción |
|---|---|
| `W` / `↑` | Avanzar |
| `S` / `↓` | Retroceder |
| `A` / `←` | Girar izquierda |
| `D` / `→` | Girar derecha |
| `Shift` | Boost (×1.6) |
| `Espacio` | E-Stop (alterna armado/detenido) |

---

## Topics ROS 2 utilizados

| Topic | Tipo | Dirección |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Suscripción |
| `/image_result` | `sensor_msgs/Image` | Suscripción |
| `/cmd_vel` | `geometry_msgs/Twist` | Publicación |

---

## Regenerar los protobuf (solo si se modifica el `.proto`)

```bash
cd /home/rosendorios/Desktop/Modulo3/UI/PY-RPC-Wrapper-Server-Linux
python3 -m grpc_tools.protoc -I./protos \
    --python_out=../generated_protos \
    --grpc_python_out=../generated_protos \
    ./protos/rpc-demo.proto
```
