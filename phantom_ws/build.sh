#!/bin/bash
# Script para compilar forzando Python del sistema para evitar conflictos con PyEnv

echo "--- Iniciando compilación segura ---"

# 1. Asegurar que usamos /usr/bin/python3 (el del sistema) y no el de pyenv
export PATH=/usr/bin:$PATH

# 2. Detectar automáticamente la distribución de ROS 2 instalada
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "Usando ROS 2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "Usando ROS 2 Humble"
else
    echo "ERROR: No se encontró ninguna distribución de ROS 2 en /opt/ros/"
    exit 1
fi

# 3. Compilar forzando el ejecutable de Python correcto
# --symlink-install: Para no tener que recompilar si solo cambias Python (en algunos casos)
# -DPython3_EXECUTABLE: Fuerza CMake a usar el Python del sistema
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo "--- Compilación terminada ---"
echo "Para usar los cambios, ejecuta: source install/setup.bash"
