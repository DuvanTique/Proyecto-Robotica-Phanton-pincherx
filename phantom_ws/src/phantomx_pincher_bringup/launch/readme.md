# Launch Files — `phantomx_pincher_bringup`

## `phantomx_pincher.launch.py`

Lanzador principal del sistema. Soporta simulación y robot real.

```bash
# Simulación
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=false

# Robot real
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=true port:=/dev/ttyUSB1
```

## `vision_bringup.launch.py`

Lanzador del sistema de visión (cámara + nodo de reconocimiento).

```bash
ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true camera_device:=/dev/video4 start_clasificador:=true
```

## Argumentos Disponibles

Usar `ros2 launch --show-args phantomx_pincher_bringup <archivo>.launch.py` para ver todos los argumentos de cada launch file.
