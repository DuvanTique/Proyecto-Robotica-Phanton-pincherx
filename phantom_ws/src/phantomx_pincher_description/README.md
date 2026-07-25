# `phantomx_pincher_description` — Descripción del Robot

Paquete que contiene el modelo URDF/Xacro y los meshes (STL, DAE) del robot PhantomX Pincher X100, incluyendo el entorno de trabajo (base fija, soporte de cámara).

## Contenido

- **URDF/Xacro**: Descripción completa del robot con joints revolute (brazo) y prismáticos (gripper).
- **Meshes**: Modelos 3D para visualización (DAE) y colisión (STL).
- **Configuración**: Posiciones iniciales para simulación (`initial_joint_positions.yaml`).
- **ros2_control**: Definición del hardware simulado (`mock_components/GenericSystem`).

## Estructura

```
├── config/
│   └── initial_joint_positions.yaml   # Posiciones de arranque (simulación)
├── launch/
│   ├── view.launch.py                 # Visualización URDF con RViz2
│   └── view_ign.launch.py            # Visualización SDF con Gazebo
├── meshes/
│   ├── collision/*.stl                # Mallas de colisión
│   ├── visual/*.dae                   # Mallas visuales
│   └── STL/                           # Piezas adicionales (soporte cámara, etc.)
├── urdf/
│   ├── phantomx_pincher.urdf.xacro    # Descriptor principal
│   ├── phantomx_pincher_arm.xacro     # Brazo (4 DOF)
│   ├── phantomx_pincher_gripper.xacro # Gripper (prismático)
│   ├── phantomx_pincher.ros2_control  # Hardware interface
│   └── kit.xacro                      # Entorno: base, soporte cámara
└── scripts/
    ├── xacro2urdf.bash                # Generación de URDF estático
    └── xacro2sdf_direct.bash          # Generación de SDF para Gazebo
```

## Uso

```bash
# Visualizar el robot en RViz
ros2 launch phantomx_pincher_description view.launch.py

# Generar URDF con argumentos personalizados
xacro phantomx_pincher.urdf.xacro name:="phantomx_pincher" ros2_control_plugin:=fake ros2_control:=true
```

## Joints del Robot

| Joint | Tipo | Eje | Límites |
|---|---|---|---|
| arm_shoulder_pan_joint | revolute | Z | ±2.62 rad |
| arm_shoulder_lift_joint | revolute | Z | ±2.62 rad |
| arm_elbow_flex_joint | revolute | Z | ±2.62 rad |
| arm_wrist_flex_joint | revolute | Z | ±2.62 rad |
| gripper_finger1_joint | prismatic | X | 0.001–0.0158 m |
| gripper_finger2_joint | prismatic | X | 0.001–0.0158 m |
