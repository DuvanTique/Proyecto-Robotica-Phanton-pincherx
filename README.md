# Proyecto Final - Robótica 2026-I

Clasificación automatizada de figuras geométricas con el robot **Phantom X Pincher X100**, visión de máquina (API Roboflow), **MoveIt 2** y **ROS 2 Jazzy**.

## Requisitos

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (desktop o ros-base)
- Python 3.12
- Git

## Instalación

### 1. Instalar ROS 2 Jazzy

Seguir la [guía oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) con la opción **Desktop Install**.

### 2. Instalar dependencias

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
  python3-pip python3-colcon-common-extensions

pip install --break-system-packages requests transforms3d python-dotenv
```

### 3. Clonar el repositorio

```bash
cd ~/ros2_jazzy
git clone https://github.com/DuvanTique/Proyecto-Final-Robotica-2026-I.git
cd Proyecto-Final-Robotica-2026-I/phantom_ws
```

### 4. Compilar

```bash
./build.sh
source install/setup.bash
```

### 5. Agregar source al .bashrc (opcional)

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_jazzy/Proyecto-Final-Robotica-2026-I/phantom_ws/install/setup.bash' >> ~/.bashrc
```

## Ejecución

### Simulación (sin robot real)

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=false \
  start_clasificador:=true
```

En otra terminal, simular una detección:

```bash
ros2 topic pub -1 /figure_type std_msgs/msg/String "{data: 'cubo'}"
```

### Robot real

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true
```

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
├── phantomx_pincher_description/   # URDF/Xacro del robot y entorno
├── phantomx_pincher_moveit_config/ # Configuración de MoveIt 2
├── phantomx_pincher_bringup/       # Launch files y configuración de poses
├── phantomx_pincher_commander_cpp/ # Nodo C++ que conecta /pose_command con MoveIt
├── phantomx_pincher_interfaces/    # Mensaje PoseCommand
└── pincher_control/                # Nodos Python:
    ├── clasificador_node.py        # FSM de pick & place
    ├── follow_joint_trajectory_node.py  # Control hardware AX-12A
    ├── recognition_node.py         # Visión via API Roboflow
    └── scene_objects_node.py       # Collision objects para MoveIt
```

## Clasificación de figuras

| Figura detectada | Destino |
|------------------|---------|
| cubo | caneca roja |
| cilindro | caneca verde |
| pentagono | caneca azul |
| rectangulo | caneca amarilla |

## Permisos del puerto USB (robot real)

```bash
sudo usermod -aG dialout $USER
# Reiniciar sesión después
```
