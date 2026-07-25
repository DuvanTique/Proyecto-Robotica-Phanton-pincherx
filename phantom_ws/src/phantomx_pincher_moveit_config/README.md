# `phantomx_pincher_moveit_config` — Configuración de MoveIt 2

Paquete con la configuración completa de **MoveIt 2** para el PhantomX Pincher X100: SRDF, planificador OMPL, límites articulares, solver cinemático y controladores.

## Grupos de Planificación

| Grupo | Joints | Solver |
|---|---|---|
| `arm` | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex | KDL |
| `gripper` | gripper_finger1, gripper_finger2 | KDL |

## Named Targets (SRDF)

| Nombre | Grupo | Descripción |
|---|---|---|
| `up` | arm | Brazo completamente vertical |
| `rest` | arm | Posición compacta (shoulder_lift=-90°, elbow=130°) |
| `open` | gripper | Dedos completamente abiertos (0.0158 m) |
| `closed` | gripper | Dedos cerrados (0.001 m) |

## Archivos de Configuración

```
config/
├── controllers_position.yaml          # Controladores de posición (joint_trajectory_controller)
├── controllers_effort.yaml            # Controladores de esfuerzo (PID)
├── joint_limits.yaml                  # Límites de velocidad y aceleración
├── kinematics.yaml                    # Solver cinemático (KDL)
├── ompl_planning.yaml                 # Planificadores OMPL disponibles
├── servo.yaml                         # Configuración de MoveIt Servo (deshabilitado en Jazzy)
└── moveit_controller_manager_*.yaml   # Mapeo de controladores para MoveIt
```

## Launch Files

| Archivo | Uso |
|---|---|
| `move_group.launch.py` | Configura move_group con ros2_control, RViz y controladores |
| `move_group_external_control.launch.py` | Versión para control externo (MoveIt Servo) |

## Uso

```bash
# Lanzar MoveIt en simulación (modo standalone)
ros2 launch phantomx_pincher_moveit_config move_group.launch.py

# Ver argumentos disponibles
ros2 launch --show-args phantomx_pincher_moveit_config move_group.launch.py
```

## Planificador OMPL

Planificadores habilitados: RRT, RRTConnect, RRTstar, PRM, LBKPIECE, BKPIECE, KPIECE, EST, BiEST, SBL, entre otros.

Configuración por defecto:
- Tiempo de planificación: 5.0 s
- Intentos de planificación: 10
- Tolerancia de posición: 0.005 m
- Tolerancia de orientación: 3.14159 rad (permite cualquier orientación para 4 GDL)
