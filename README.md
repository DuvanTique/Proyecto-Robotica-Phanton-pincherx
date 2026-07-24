# Proyecto Final - Robótica 2026-I

Clasificación automatizada de figuras geométricas con el robot **Phantom X Pincher X100**, visión de máquina (API Roboflow), **MoveIt 2** y **ROS 2 Jazzy**.

## Equipo

- Duvan Tique

## Requisitos

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (desktop o ros-base)
- Python 3.12
- Git

## Instalación

### 1. Instalar ROS 2 Jazzy

Seguir la [guía oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) con la opción **Desktop Install**.

### 2. Instalar dependencias del sistema

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-tf-transformations \
  ros-jazzy-moveit* \
  ros-jazzy-dynamixel-sdk \
  ros-jazzy-control-msgs \
  python3-pip \
  python3-colcon-common-extensions \
  python3-tk
```

### 3. Instalar dependencias de Python

```bash
pip install --break-system-packages requests transforms3d python-dotenv pillow
```

### 4. Clonar el repositorio

```bash
mkdir -p ~/ros2_jazzy && cd ~/ros2_jazzy
git clone https://github.com/DuvanTique/Proyecto-Robotica-Phanton-pincherx.git
cd Proyecto-Robotica-Phanton-pincherx/phantom_ws
```

### 5. Compilar

```bash
source /opt/ros/jazzy/setup.bash
./build.sh
source install/setup.bash
```

### 6. Agregar source al .bashrc (recomendado)

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_jazzy/Proyecto-Robotica-Phanton-pincherx/phantom_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Ejecución

### Simulación (sin robot real)

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=false \
  start_clasificador:=true
```

En otra terminal, simular una detección de figura:

```bash
ros2 topic pub -1 /figure_type std_msgs/msg/String "{data: 'cubo'}"
ros2 topic pub -1 /figure_type std_msgs/msg/String "{data: 'cilindro'}"
ros2 topic pub -1 /figure_type std_msgs/msg/String "{data: 'pentagono'}"
ros2 topic pub -1 /figure_type std_msgs/msg/String "{data: 'rectangulo'}"
```

### Robot real

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true
```

> **Nota:** El robot debe estar conectado por USB. Verificar con `ls /dev/ttyUSB*`

### Interfaz gráfica (GUI)

En una terminal separada (después de lanzar el sistema):

```bash
ros2 run pincher_control pincher_gui
```

La GUI permite:
- Iniciar/detener ciclo automático
- Parada de emergencia
- Enviar robot a HOME
- Control manual del gripper
- Ver estado de la FSM y conteo de figuras

### Visión con Roboflow

```bash
export PINCHER_API_KEY="tu_api_key"
export PINCHER_MODEL_ID="tu-proyecto/1"

ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true \
  camera_device:=/dev/video0 \
  start_clasificador:=true
```

### Ejecución headless (Raspberry Pi)

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true \
  enable_rviz:=false
```

## Estructura del proyecto

```
phantom_ws/src/
├── phantomx_pincher_description/   # URDF/Xacro del robot y entorno (canecas, base, cámara)
├── phantomx_pincher_moveit_config/ # Configuración de MoveIt 2 (OMPL, controladores, SRDF)
├── phantomx_pincher_bringup/       # Launch files y configuración de poses
│   └── config/poses.yaml           # Poses calibradas (home, pick, pre_drop, drop)
├── phantomx_pincher_commander_cpp/ # Nodo C++ que conecta /pose_command con MoveIt
├── phantomx_pincher_interfaces/    # Mensaje personalizado PoseCommand
└── pincher_control/                # Nodos Python:
    ├── clasificador_node.py        # FSM de pick & place
    ├── follow_joint_trajectory_node.py  # Control hardware AX-12A
    ├── recognition_node.py         # Visión via API Roboflow
    ├── scene_objects_node.py       # Collision objects para MoveIt
    └── gui_node.py                 # Interfaz gráfica de usuario
```

## Tópicos principales

| Tópico | Tipo | Función |
|--------|------|---------|
| `/pose_command` | PoseCommand | Enviar pose objetivo al commander |
| `/figure_type` | String | Figura detectada → inicia pick & place |
| `/figure_state` | String | Estado continuo de la detección |
| `/joint_states` | JointState | Estado articular del robot |
| `/set_gripper` | Bool | Control directo del gripper (True=abrir) |
| `/routine_busy` | Bool | FSM ocupada (pausa visión) |
| `/emergency_stop` | Bool | Parada de emergencia |
| `/planning_scene` | PlanningScene | Collision objects del entorno |

## Clasificación de figuras

| Figura detectada | Destino |
|------------------|---------|
| cubo | caneca roja |
| cilindro | caneca verde |
| pentagono | caneca azul |
| rectangulo | caneca amarilla |

## Configuración del robot real

### Permisos del puerto USB

```bash
sudo usermod -aG dialout $USER
# Reiniciar sesión después
```

### Verificar conexión

```bash
ls /dev/ttyUSB*
```

### Si el puerto es diferente de /dev/ttyUSB0

Editar los parámetros en `phantomx_pincher.launch.py` (sección `follow_joint_trajectory_node`) o crear un symlink:

```bash
sudo ln -sf /dev/ttyUSB1 /dev/ttyUSB0
```

## Solución de problemas

### "No transform" en RViz (robot real)

El nodo `follow_joint_trajectory` no arrancó (puerto USB no disponible). Verificar:
```bash
ros2 node list | grep follow
ros2 topic hz /joint_states
```

### "No transform" en RViz (simulación)

El `joint_state_broadcaster` no se activó. Cerrar todo y relanzar.

### MoveIt no planifica / "invalid start state"

El robot está en una posición que la constraint del codo rechaza. Se reintenta automáticamente hasta 10 veces. Si persiste, enviar a HOME manualmente:
```bash
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.01745, 0.01745, 0.01745, 0.01745]}"
```

### El brazo se mueve "al revés"

Verificar `joint_signs` en `follow_joint_trajectory_node.py`. Para AX-12A:
- ID 1 (shoulder pan): +1
- ID 2 (shoulder lift): -1
- ID 3 (elbow flex): -1
- ID 4 (wrist flex): -1
- ID 5 (gripper): +1

## Configuración de Roboflow

Para entrenar el modelo de clasificación:
1. Crear proyecto en [Roboflow](https://roboflow.com) tipo **Classification → Single-Label**
2. Clases (exactamente): `cubo`, `cilindro`, `pentagono`, `rectangulo`, `vacio`
3. Subir imágenes del ROI de la cámara
4. Entrenar y obtener API key + Model ID
