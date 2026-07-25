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

### 4. Instalar Git LFS (necesario para los archivos STL/DAE del robot)

```bash
sudo apt install git-lfs
git lfs install
```

### 5. Clonar el repositorio

```bash
mkdir -p ~/ros2_jazzy && cd ~/ros2_jazzy
git clone https://github.com/DuvanTique/Proyecto-Robotica-Phanton-pincherx.git
cd Proyecto-Robotica-Phanton-pincherx/phantom_ws
```

> Si ya clonaste el repo antes de instalar Git LFS, ejecuta `git lfs pull` dentro del repositorio para descargar los archivos binarios correctamente.

### 6. Compilar

```bash
source /opt/ros/jazzy/setup.bash
./build.sh
source install/setup.bash
```

### 7. Agregar source al .bashrc (recomendado)

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

El puerto USB se puede seleccionar directamente desde el launch. Por ejemplo, si el U2D2 aparece como `/dev/ttyUSB1`:

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true \
  port:=/dev/ttyUSB1
```

Si no se indica `port`, el valor predeterminado es `/dev/ttyUSB0`.

> **Nota:** El robot debe estar conectado por USB. Verifica el dispositivo con `ls -l /dev/ttyUSB* /dev/ttyACM*`.

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

#### Botones Scan y Start (importante)

Para no saturar la API de Roboflow con llamadas continuas, `recognition_node` **no** consulta la API automáticamente. Solo lo hace bajo demanda:

- **📷 Scan**: consulta la API de Roboflow **una sola vez** y muestra el resultado en la GUI. **No mueve el robot.**
- **▶ Start**: usa la última detección mostrada (obtenida con Scan) para arrancar la secuencia de pick & place. **Esto es lo único que hace que el robot empiece a moverse.**

Al finalizar cada ciclo, el clasificador dispara automáticamente un nuevo escaneo (sin mover el robot), para que la siguiente figura ya esté detectada antes de presionar Start otra vez.

### Visión con Roboflow

#### Opción A (recomendada): archivo .env local, sin exportar variables cada vez

1. Copia la plantilla y complétala con tus credenciales reales:

```bash
cd ~/ros2_jazzy/robotica-proyecto-final/phantom_ws/src/pincher_control/config
cp .env.example .env
nano .env
```

2. Edita `.env` con tu API key y tu modelo:

```bash
PINCHER_API_KEY=tu_api_key_real
PINCHER_MODEL_ID=tu-proyecto/1
PINCHER_API_BACKEND=roboflow
```

3. Recompila para que el archivo quede disponible en `install/`:

```bash
cd ~/ros2_jazzy/robotica-proyecto-final/phantom_ws
./build.sh
source install/setup.bash
```

El nodo `recognition_node` carga automáticamente `.env` al iniciar. No necesitas exportar nada manualmente en cada terminal.

`.env` está protegido en `.gitignore` y nunca se sube al repositorio. Solo `.env.example` (sin credenciales reales) se versiona en git, para que cualquier persona que clone el proyecto sepa qué variables debe configurar.

#### Opción B: variables de entorno manuales (sesión actual)

```bash
export PINCHER_API_KEY="tu_api_key"
export PINCHER_MODEL_ID="tu-proyecto/1"
```

#### Lanzar el sistema de visión

```bash
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
| `/figure_type` | String | Figura confirmada por Start/Next → inicia pick & place (el robot se mueve) |
| `/figure_state` | String | Última detección conocida (actualizada solo tras un `/trigger_scan`) |
| `/trigger_scan` | Bool | Dispara una única consulta a la API de Roboflow (sin mover el robot) |
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

### Calibración global de articulaciones

La calibración de los motores reales se centraliza en:

```text
phantom_ws/src/phantomx_pincher_bringup/config/joint_calibration.yaml
```

El primer valor corresponde a la primera articulación, motor Dynamixel ID 1:

```yaml
joint_offsets_degrees: [-2.0, 0.0, 0.0, 0.0, 0.0]
```

El valor `-2.0` hace que la primera articulación reciba siempre 2° menos que la posición nominal solicitada por MoveIt. Esto afecta Home, pick, drop, el clasificador y cualquier trayectoria enviada al nodo `follow_joint_trajectory` en modo real. No es necesario modificar individualmente las poses ni los botones de la GUI.

Orden de los valores:

```text
[ID 1, ID 2, ID 3, ID 4, gripper]
```

Para modificar la calibración, cambia únicamente el valor correspondiente. Por ejemplo, `-1.5` aplica 1.5° menos al motor ID 1. Después recompila:

```bash
cd ~/ros2_jazzy/robotica-proyecto-final/phantom_ws
source /opt/ros/jazzy/setup.bash
./build.sh
source install/setup.bash
```

La calibración se aplica al hardware real; la simulación y las poses nominales de MoveIt permanecen sin offset para que la planificación y la visualización sigan usando el mismo modelo geométrico.

### Permisos del puerto USB

```bash
sudo usermod -aG dialout $USER
# Reiniciar sesión después
```

### Verificar conexión

```bash
ls /dev/ttyUSB*
```

### Seleccionar el puerto desde el comando

No es necesario renombrar `/dev/ttyUSB1` a `/dev/ttyUSB0`. El puerto se puede pasar al launch:

```bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py \
  use_real_robot:=true \
  start_clasificador:=true \
  port:=/dev/ttyUSB1
```

El valor predeterminado sigue siendo `/dev/ttyUSB0`.


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

Verificar `joint_sign` en `follow_joint_trajectory_node.py`. Convenio verificado en el robot real:
- ID 1 (shoulder pan): -1
- ID 2 (shoulder lift): +1
- ID 3 (elbow flex): +1
- ID 4 (wrist flex): +1
- ID 5 (gripper): +1

Si RViz muestra la misma orientación que el robot real pero las trayectorias se ejecutan hacia el lado contrario, el problema es el signo del motor (no el URDF ni las poses). Cambia únicamente `joint_sign` del ID correspondiente y recompila.

## Configuración de Roboflow

Para entrenar el modelo de clasificación:
1. Crear proyecto en [Roboflow](https://roboflow.com) tipo **Classification → Single-Label**
2. Clases (exactamente): `cubo`, `cilindro`, `pentagono`, `rectangulo`, `vacio`
3. Subir imágenes del ROI de la cámara
4. Entrenar y obtener API key + Model ID
