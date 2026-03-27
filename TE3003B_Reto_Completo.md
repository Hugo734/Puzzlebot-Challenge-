# TE3003B — Integración de Robótica y Sistemas Inteligentes
## Planteamiento completo del Reto: Montacargas Autónomo
**ITESM Campus Monterrey · FJ 2026**  
Coordinador Grupo 1: Arturo Cerón (Dr. Eng.) · Coordinador Grupo 2: Alfredo Esquivel (Ph.D.)

---

## Índice

1. [Objetivo del Reto](#1-objetivo-del-reto)
2. [Entorno Físico](#2-entorno-físico)
3. [Arquitectura del Sistema](#3-arquitectura-del-sistema)
4. [Los 9 Evaluables del Reto](#4-los-9-evaluables-del-reto)
5. [Módulos Teóricos relacionados al Reto](#5-módulos-teóricos-relacionados-al-reto)
6. [Metodología Scrum y Sprints](#6-metodología-scrum-y-sprints)
7. [Ponderaciones y Calificación](#7-ponderaciones-y-calificación)
8. [Recursos, Logística y Reglas del Puzzlebot](#8-recursos-logística-y-reglas-del-puzzlebot)
9. [Bonus: Robot ANYmal D](#9-bonus-robot-anymal-d)

---

## 1. Objetivo del Reto

Construir un **montacargas autónomo** basado en el robot **Puzzlebot** capaz de:

- Detectar y localizar **pallets con carga** dentro de una pista que simula un almacén.
- Navegar de forma autónoma por el entorno, eludiendo obstáculos.
- Recoger los pallets usando un brazo tipo montacargas (gripper).
- Transportarlos y depositarlos en el interior de un **camión de carga (tráiler)** como destino final.
- Toda la operación debe ocurrir **sin intervención humana** durante la ejecución.

> La tarea replica un escenario industrial real de logística autónoma en almacenes, similar a los sistemas AGV (Automated Guided Vehicles) utilizados en empresas como Amazon o DHL.

---

## 2. Entorno Físico

### 2.1 La Pista

La pista es un área rectangular delimitada con **cajas de cartón y cinta amarilla/negra** que simula el piso de un almacén. Dentro de la pista se colocan:

- **Pallets** distribuidos aleatoriamente como objetos a recoger.
- **Obstáculos** que el robot debe esquivar.
- **Tráileres** posicionados en los bordes como destinos de entrega.

> La pista es **tentativa** — al momento del inicio del semestre se estaba adquiriendo una pista nueva.

### 2.2 Los Pallets

Se proveerán **pallets estándar en miniatura** (diseño de madera a escala). Cada pallet representa una unidad de carga que el robot debe identificar, recoger y transportar. Los pallets pueden estar en posiciones variables en cada corrida de evaluación.

### 2.3 Los Tráileres

Son estructuras que simulan la **vista trasera de un camión de carga** con la puerta trasera abierta. El robot debe identificar el tráiler asignado, alinearse correctamente con él y depositar el pallet en su interior. Se usan **CNNs para detectar el tráiler** y **QR codes** para identificar las cajas/pallets.

### 2.4 El Montacargas (Gripper)

El robot Puzzlebot necesita un accesorio mecánico tipo montacargas para levantar y depositar los pallets. Los equipos deben **diseñar y construir** este accesorio. Se pueden revisar los diseños de años anteriores como referencia (existe un prototipo con brazos verticales de material impreso en 3D, servo-motor y sistema de correa/piñón controlado por FPGA).

---

## 3. Arquitectura del Sistema

El sistema completo se articula en tres capas: **hardware físico**, **software en ROS2** y **servicios en la nube**.

### 3.1 Capa de Hardware (Puzzlebot)

| Componente | Función | Señal ROS2 |
|---|---|---|
| **Cámara** | Captura imagen para percepción visual | `/cam_img` (Image) |
| **LiDAR** | Escaneo del entorno para mapeo y evasión | `/scan` (LaserScan) |
| **Motores DC + Encoders** | Tracción y odometría | `/VelocityEncR`, `/VelocityEncL` (Float32) |
| **µC + H-bridge** | Interfaz entre software y motores | `/cmd_vel` (Twist) |
| **Gripper (FPGA + Servo)** | Mecanismo para levantar pallets | `/open_close` (Boolean) |
| **Micrófono** | Captura de comandos de voz | — |

> El gripper es controlado directamente desde una **FPGA**, que genera las señales de PWM para el servo-motor. Este es uno de los evaluables del módulo M2.

### 3.2 Capa de Software (ROS2 — Embedded Computer / Jetson Nano)

Todo el software del robot corre en **ROS2** sobre la computadora embebida. Se divide en cinco bloques funcionales:

#### 3.2.1 Percepción

| Nodo | Tecnología | Propósito |
|---|---|---|
| **Aruco Detection** | OpenCV | Detecta marcadores visuales para referencias de posición en 3D |
| **Camera Calibration** | OpenCV | Calibra la cámara para corregir distorsión y obtener medidas precisas |
| **Trailer Detection** | YOLO | Detecta el tráiler destino en la imagen de la cámara |
| **Robot Alignment** | QR + CNN + OpenCV | Alineación precisa del robot con el pallet y con el tráiler antes de operar el gripper |

#### 3.2.2 Localización

| Nodo | Tecnología | Propósito |
|---|---|---|
| **Dead Reckoning** | Cinemática diferencial | Estima posición a partir de velocidades de ruedas |
| **Kalman Filter** | Filtro de Kalman extendido | Fusiona odometría y sensores para una pose robusta del robot |

La salida combinada es la **Pose** estimada del robot (posición x, y + orientación θ), que alimenta tanto al planificador de rutas como a la interfaz web.

#### 3.2.3 Mapeo (SLAM)

| Nodo | Tecnología | Propósito |
|---|---|---|
| **SLAM** | CoreSLAM | Construye un mapa del entorno simultáneamente con la localización |

> El evaluable M3 exige que el mapa se construya con un **algoritmo propio**, no un paquete ROS preconfigurado. Se puede partir de CoreSLAM como base.

#### 3.2.4 Navegación

| Nodo | Tecnología | Propósito |
|---|---|---|
| **Path Planner** | A*, D* Lite, o RRT | Planificación global de la ruta desde el origen hasta el destino |
| **Obstacle Avoidance** | Bug Algorithms (Bug0, Bug1, Tangent Bug…) | Evasión reactiva de obstáculos no mapeados |

#### 3.2.5 Control

| Nodo | Propósito |
|---|---|
| **State Machine** | Orquesta el flujo completo del robot: detectar pallet → acercarse → recoger → navegar → entregar. Controlado también por comandos de voz. |
| **Steering Controller (PID)** | Controla la velocidad angular y lineal del robot para seguir la trayectoria planificada |

#### 3.2.6 Comandos de Voz (NLP)

Un nodo independiente procesa la entrada del micrófono y, usando técnicas de **NLP (Natural Language Processing)**, extrae el comando del operador para enviarlo a la máquina de estados:

- `Start` → inicia la misión autónoma
- `Pause` → congela el robot en su posición actual
- `Next` → pasa al siguiente pallet/destino
- `Stop` → detiene y reinicia el sistema

### 3.3 Capa Cloud (E80 Group)

Una **interfaz web en tiempo real** expuesta vía REST API que muestra:

- Streaming de la cámara del robot en vivo.
- Posición del robot superpuesta sobre el mapa.
- Estado de las entregas a cada tráiler (entregado / pendiente).
- Telemetría general del sistema.

**Stack tecnológico sugerido:** Flask o gRPC-Gateway + REST, Docker Compose para deployment, datos estructurados en JSON.

---

## 4. Los 9 Evaluables del Reto

Los evaluables se agrupan por **evaluador** y tienen pesos individuales. El total del componente reto suma **57%** de la calificación final.

---

### 4.1 Evaluables del Campus Monterrey (M1–M4 · 32% total)

#### Evaluable 1 — M1: Estándares industriales en robótica (8%)
**Evaluador:** ITESM Campus Monterrey (Prof. Muñoz)

**Descripción:** Documentar e implementar los estándares industriales relevantes para el sistema robótico desarrollado.

**Entregables esperados:**
- Ontologías aplicadas a la descripción del sistema robótico.
- Mapeo de normas **ISO** aplicables (ej. ISO 10218 para robots industriales, ISO/TS 15066 para robots colaborativos).
- Normas Mexicanas (**NOM**) relevantes si aplica.
- Justificación de cada estándar en el contexto del reto.

---

#### Evaluable 2 — M2: FPGA para controlar el servo del Montacargas (8%)
**Evaluador:** ITESM Campus Monterrey (Prof. Ávila)

**Descripción:** Diseñar e implementar en **FPGA** (usando HDL: VHDL o Verilog) el módulo de control del servo-motor que activa el gripper del montacargas.

**Entregables esperados:**
- Diseño HDL del módulo de generación de señal PWM para el servo.
- Entrada de datos desde ROS2 vía GPIO de la Jetson Nano (señal `/open_close`).
- Verificación funcional: el gripper debe abrirse y cerrarse de forma controlada desde software.
- El diseño debe pasar por síntesis e implementación en la FPGA física.

---

#### Evaluable 3 — M3: Creación automática de mapa (8%)
**Evaluador:** ITESM Campus Monterrey (Prof. Cerón)

**Descripción:** Implementar un algoritmo de **construcción automática de mapa** (SLAM) con código propio.

**Entregables esperados:**
- El robot debe recorrer la pista y generar un mapa 2D del entorno.
- Implementación basada en **CoreSLAM** o algoritmo equivalente, con comprensión profunda del código.
- El mapa generado debe ser utilizable por el planificador de rutas.
- Documentación técnica del algoritmo y sus parámetros.

---

#### Evaluable 4 — M4: Comandos de voz para controlar el robot (8%)
**Evaluador:** ITESM Campus Monterrey (Prof. Esquivel)

**Descripción:** Implementar un sistema de **reconocimiento de voz** que permita controlar el flujo de la misión del robot mediante comandos hablados.

**Entregables esperados:**
- Pipeline completo de procesamiento de voz: captura de audio → preprocesamiento → extracción de características (MFCCs) → reconocimiento.
- Reconocimiento al menos de los comandos: Start, Pause, Next, Stop.
- Integración con la máquina de estados del robot vía ROS2.
- El sistema debe funcionar en tiempo real con latencia aceptable para uso operativo.

---

### 4.2 Evaluables ITESM Nacional / Manchester Robotics MCR² (15% total)

Las sesiones de evaluación con Manchester Robotics son los **miércoles 13:00–15:00h** (en vivo + en línea).

#### Evaluable 5 — MR: Detección de referencias (5%)
**Evaluador:** ITESM Nacional / MCR²

**Descripción:** Detectar **marcadores Aruco** y estimar su posición y orientación en el espacio 3D.

**Entregables esperados:**
- Nodo ROS2 que detecta marcadores Aruco de la cámara del robot.
- Para cada marcador detectado: publicar su ID, posición (x, y, z) y orientación (cuaternión o ángulos de Euler) relativa a la cámara.
- La detección debe ser robusta a variaciones de iluminación y distancia dentro de la pista.
- Visualización en RViz o interfaz equivalente.

---

#### Evaluable 6 — MR: Localización (5%)
**Evaluador:** ITESM Nacional / MCR²

**Descripción:** Implementar el **Filtro de Kalman** desde cero para localizar el robot.

**Entregables esperados:**
- Implementación propia del Filtro de Kalman (EKF o UKF) sin usar librerías de alto nivel que lo abstraigan.
- Fusión de al menos dos fuentes: odometría (encoders) + otro sensor (LiDAR, Aruco, o IMU).
- El filtro debe publicar la pose estimada en `/robot_pose`.
- Demostración de que la localización es más precisa que Dead Reckoning solo.
- Justificación matemática del modelo de movimiento y modelo de observación usados.

---

#### Evaluable 7 — MR: Navegación (7%) → [**5% en el reto**]
**Evaluador:** ITESM Nacional / MCR²

**Descripción:** Implementar navegación autónoma con **planeación de rutas** y **evasión de obstáculos**.

**Entregables esperados:**
- Implementación de al menos un algoritmo de planeación global: **A***, D* Lite, o RRT.
- Implementación de al menos un **Bug Algorithm** (Bug0, Bug1, Tangent Bug) para evasión reactiva.
- El robot debe ir de un punto A a un punto B en el mapa sin colisionar con obstáculos.
- Demostración en la pista real con obstáculos colocados por los evaluadores.

---

### 4.3 Evaluables E80 Group (10% total)

Incluye visitas presenciales a las instalaciones de E80 Group (fechas por definir).

#### Evaluable 8 — E80: Detección de objetivos y alineación del robot (5%)
**Evaluador:** E80 Group

**Descripción:** Detectar visualmente los objetivos (pallets y tráileres) y alinear el robot con precisión para operar el gripper.

**Entregables esperados:**
- **QR codes** en los pallets: el robot debe leer el QR para identificar qué pallet está recogiendo.
- **CNN (red neuronal convolucional)** para detectar y clasificar los tráileres en la imagen de la cámara.
- Alineación precisa con **visión clásica (OpenCV)** + controlador **PID** para centrar el robot frente al pallet/tráiler antes de operar.
- La alineación debe ser lo suficientemente precisa para que el gripper funcione sin asistencia humana.

---

#### Evaluable 9 — E80: Interfaz Web con visualización de telemetría (5%)
**Evaluador:** E80 Group

**Descripción:** Desarrollar e implementar una **interfaz web en tiempo real** que muestre el estado del robot durante la misión.

**Entregables esperados:**
- **Streaming de cámara** del robot en vivo (baja latencia).
- **Mapa 2D** del entorno con la posición del robot superpuesta en tiempo real.
- **Estado de entregas**: qué tráileres han recibido pallets y cuáles están pendientes.
- **Telemetría**: velocidades, estado de la batería, modo de operación (autónomo/pausado/manual).
- La interfaz debe ser accesible desde un navegador en la misma red local o desde la nube.

---

## 5. Módulos Teóricos relacionados al Reto

Cada módulo tiene un **componente teórico (7%)** y un **componente reto (8%)**, para un total de 15% por módulo.

### M1 — Tecnologías Emergentes (Prof. Muñoz)
Temas relevantes al reto: estándares IEEE para robótica, ROS-Industrial, estructuras de datos y algoritmos, inteligencia artificial industrial, índice de innovación, CHIPS y computación cuántica.

### M2 — Sistemas Embebidos (Prof. Ávila)
Temas relevantes al reto: GPIOs baremetal en Jetson Nano, codesign hardware/software con HDL y FPGA, diagramas de estados, modelado y generación de código embebido, análisis estático, pruebas unitarias, seguridad en software embebido.

### M3 — Robots Autónomos (Prof. Cerón / Esquivel)
Temas relevantes al reto: SLAM (CoreSLAM, ORBSLAM3, Google Cartographer), Dead Reckoning, Filtro de Kalman, teorema de Bayes, navegación autónoma, planeación de rutas, cuaterniones, análisis de sensores, procesamiento de voz (MFCCs, HMM, LPC), niveles de autonomía.

### M4 — Interfaces Inteligentes (Prof. Esquivel / Cerón)
Temas relevantes al reto: Docker, DevOps, GitHub Actions, micro-servicios, gRPC + REST, ZeroMQ, MQTT, ROS2, Flask, procesamiento de lenguaje natural aplicado a robótica, verificación del hablante, LLMs, comandos de movimiento con voz, diseño MVVM, documentación con Doxygen.

---

## 6. Metodología Scrum y Sprints

El reto se desarrolla con **metodología Scrum**. Se trabaja en sprints de ~2 semanas con reuniones de retrospectiva y planeación al final de cada uno. Se usa **Jira** como herramienta de gestión (tablero Kanban + backlog de historias de usuario).

### Calendario de Sprints

| Sprint | Fechas | Sesión de Retrospectiva | Entrega de Actividad |
|---|---|---|---|
| **Sprint 1** | 6 – 17 abril | 17 de abril | 19 de abril |
| **Sprint 2** | 20 abril – 1 mayo | 1 de mayo | 3 de mayo |
| **Sprint 3** | 4 – 15 mayo | 15 de mayo | 17 de mayo |
| **Sprint 4** | 18 – 29 mayo | 29 de mayo | 31 de mayo |
| **Sprint Final** | 1 – 5 junio | — | 3 de junio (reporte + presentación) |
| **Ajustes finales** | 5 junio | — | 5 de junio |

### Contenido de cada entrega (Retillo)

Cada entrega de sprint debe incluir un reporte en **formato IEEE** (usando Overleaf con la plantilla de RAS PaperCept):

1. **Avances técnicos** — descripción de lo implementado (2 páginas).
2. **Reporte de retrospectiva** — qué salió bien, qué mejorar, planeación del siguiente sprint (1 página).
3. **Reflexiones individuales** — cada miembro del equipo escribe la suya (1 página total).
4. **Captura del Kanban** — tablero anterior (completado) y nuevo (planeado) (1 página).

**Meta mínima por entrega:** todos los mini-retos (minichallenges) completados hasta esa fecha + prototipo de al menos **3 incisos nuevos** del reto completo.

> Plantilla Overleaf: https://ras.papercept.net/conferences/support/tex.php

---

## 7. Ponderaciones y Calificación

### Estructura completa del curso

| Componente | Submódulos | % |
|---|---|---|
| **Teoría** | M1 (7%) + M2 (7%) + M3 (7%) + M4 (7%) | **28%** |
| **Reto Campus Mty** | M1-reto (8%) + M2-reto (8%) + M3-reto (8%) + M4-reto (8%) | **32%** |
| **Reto Manchester Robotics** | Detección (5%) + Localización (5%) + Navegación (5%) | **15%** |
| **Reto E80 Group** | Interfaz web (5%) + Detección/alineación (5%) | **10%** |
| **Ceneval** | Examen + Taller | **15%** |
| **TOTAL** | | **100%** |

### Resumen del Reto (57%)

| Evaluable | Evaluador | Peso |
|---|---|---|
| Estándares industriales (ontologías, ISO, NOM) | Campus Mty / M1 | 8% |
| FPGA → control servo del montacargas | Campus Mty / M2 | 8% |
| Creación automática de mapa | Campus Mty / M3 | 8% |
| Comandos de voz para control del robot | Campus Mty / M4 | 8% |
| Detección de marcadores Aruco (posición 3D) | ITESM Nacional / MCR² | 5% |
| Implementación del Filtro de Kalman | ITESM Nacional / MCR² | 5% |
| Navegación: Bug + planeación de rutas | ITESM Nacional / MCR² | 5% |
| Detección QR/CNN + alineación PID | E80 Group | 5% |
| Interfaz web con telemetría en tiempo real | E80 Group | 5% |
| **Total Reto** | | **57%** |

---

## 8. Recursos, Logística y Reglas del Puzzlebot

### 8.1 Recolección y devolución del Puzzlebot

- Los robots se recogen en **A7-438** con Marco, registrando matrículas.
- Hay **una semana** para verificar que todos los componentes estén funcionales. Revisar el manual de pruebas proporcionado por el profesor.
- Si durante el semestre se rompe alguna pieza por **uso indebido o negligencia**, el equipo cubre el costo de las refacciones.
- Al **final del semestre** el robot se devuelve desarmado y validado pieza por pieza con Marco. Sin devolución confirmada → no se libera la calificación del reto.

### 8.2 Baterías (compra propia obligatoria)

Cada equipo debe adquirir sus propias baterías powerbank:

| Capacidad | Descripción |
|---|---|
| **10,000 mAh** | Steren — con Turbo Charge QC y Power Delivery, 2 salidas USB + USB-C |
| **20,000 mAh** | Steren — mismas características, mayor autonomía |

### 8.3 Sesiones y comunicación

| Actividad | Día/Hora | Responsable |
|---|---|---|
| Sesiones MCR² (en vivo + en línea) | Miércoles 13:00–15:00h | Prof. Nacional |
| Sesiones de apoyo bloque | Viernes 13:00–15:00h | Prof. Bortoni + Coordinador |
| Visitas E80 Group | Fechas por definir | E80 |

### 8.4 Repositorio y comunicación

- **GitHub:** https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems_2026
- **Teams (sesiones MCR²):** Meeting ID: 273 212 111 466 44 / Passcode: Cs7g7uq9
- **Dudas Grupo 501:** arturo.ceron@tec.mx — Oficina A4-422-A
- **Dudas Grupo 502:** a.esquivel@tec.mx — Aulas III, 4to piso

---

## 9. Bonus: Robot ANYmal D

Los equipos que lleguen al **inicio del Sprint 4** con **más del 85% de avance** en al menos uno de los siguientes evaluables tendrán la oportunidad de probar su solución en el **robot cuadrúpedo ANYmal D de ANYbotics**:

| Evaluable | Criterio |
|---|---|
| M3 — Creación automática de mapa | Algoritmo propio funcionando |
| MR — Detección de referencias | Aruco con posición y orientación 3D |
| MR — Localización | Filtro de Kalman propio funcionando |
| E80 — Interfaz Web | Streaming + mapa + status en tiempo real |

> El ANYmal D es un robot de inspección cuadrúpedo industrial con capacidades de navegación avanzadas en terreno irregular. Es un incentivo significativo para los equipos más avanzados del semestre.

---

*Documento generado a partir del material oficial del curso TE3003B · ITESM Campus Monterrey · FJ2026*
