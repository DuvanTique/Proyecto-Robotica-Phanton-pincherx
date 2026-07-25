"""
Bringup de visión usando API de Roboflow para clasificación de figuras.

No requiere instalar ultralytics ni PyTorch localmente.

Variables de entorno necesarias:
  - PINCHER_API_KEY: Tu API key de Roboflow
  - PINCHER_MODEL_ID: ID del modelo (ej: "tu-proyecto/1" en Roboflow)

Uso:
  export PINCHER_API_KEY="tu_api_key_aqui"
  export PINCHER_MODEL_ID="tu-modelo/1"
  ros2 launch phantomx_pincher_bringup vision_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Variables de entorno
    api_key = os.environ.get("PINCHER_API_KEY", "")
    api_url = os.environ.get("PINCHER_API_URL", "")
    model_id = os.environ.get("PINCHER_MODEL_ID", "")
    api_backend = os.environ.get("PINCHER_API_BACKEND", "roboflow")
    image_topic_env = os.environ.get("PINCHER_IMAGE_TOPIC", "")

    # Argumentos de launch
    start_camera_arg = DeclareLaunchArgument(
        "start_camera", default_value="true",
        description="Iniciar nodo de cámara USB.",
    )
    camera_device_arg = DeclareLaunchArgument(
        "camera_device", default_value="/dev/video2",
        description="Dispositivo de vídeo.",
    )
    image_width_arg = DeclareLaunchArgument(
        "image_width", default_value="1280",
        description="Ancho de imagen.",
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height", default_value="720",
        description="Alto de imagen.",
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate", default_value="30.0",
        description="FPS de la cámara.",
    )
    # NOTA: recognition_node ya NO llama a la API a una frecuencia fija.
    # Solo consulta la API cuando recibe /trigger_scan (botón "Scan" de la
    # GUI o fin de ciclo del clasificador), para no saturar Roboflow.
    start_clasificador_arg = DeclareLaunchArgument(
        "start_clasificador", default_value="false",
        description="Iniciar nodo clasificador.",
    )

    # Argumentos de la Región de Interés (ROI) para la detección.
    # Por defecto delimita una zona cuadrada centrada en la pantalla.
    roi_x_min_arg = DeclareLaunchArgument(
        "roi_x_min_pct", default_value="0.35",
        description="Límite izquierdo del ROI (0.0-1.0, porcentaje del ancho).",
    )
    roi_x_max_arg = DeclareLaunchArgument(
        "roi_x_max_pct", default_value="0.65",
        description="Límite derecho del ROI (0.0-1.0, porcentaje del ancho).",
    )
    roi_y_min_arg = DeclareLaunchArgument(
        "roi_y_min_pct", default_value="0.35",
        description="Límite superior del ROI (0.0-1.0, porcentaje del alto).",
    )
    roi_y_max_arg = DeclareLaunchArgument(
        "roi_y_max_pct", default_value="0.65",
        description="Límite inferior del ROI (0.0-1.0, porcentaje del alto).",
    )

    # Launch configs
    start_camera = LaunchConfiguration("start_camera")
    camera_device = LaunchConfiguration("camera_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    start_clasificador = LaunchConfiguration("start_clasificador")
    roi_x_min_pct = LaunchConfiguration("roi_x_min_pct")
    roi_x_max_pct = LaunchConfiguration("roi_x_max_pct")
    roi_y_min_pct = LaunchConfiguration("roi_y_min_pct")
    roi_y_max_pct = LaunchConfiguration("roi_y_max_pct")

    # Nodo cámara
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        condition=IfCondition(start_camera),
        parameters=[{
            "video_device": camera_device,
            "framerate": framerate,
            "pixel_format": "yuyv",
            "image_width": image_width,
            "image_height": image_height,
            "auto_white_balance": True,
            "autoexposure": True,
            "autofocus": False,
        }],
    )

    # Nodo de reconocimiento vía API (Roboflow)
    api_recognition_node = Node(
        package="pincher_control",
        executable="recognition_node",
        name="recognition_node",
        output="screen",
        parameters=[{
            "api_key": api_key,
            "api_url": api_url,
            "model_id": model_id,
            "api_backend": api_backend,
            "image_topic": image_topic_env,
            "confidence_threshold": 0.70,
            "publish_roi": True,
            "roi_x_min_pct": roi_x_min_pct,
            "roi_x_max_pct": roi_x_max_pct,
            "roi_y_min_pct": roi_y_min_pct,
            "roi_y_max_pct": roi_y_max_pct,
        }],
    )

    # Nodo clasificador (opcional)
    clasificador_node = Node(
        package="pincher_control",
        executable="clasificador_node",
        name="clasificador_node",
        output="screen",
        parameters=[{
            "fsm_enabled": True,
            "pause_vision_during_execution": True,
        }],
        condition=IfCondition(start_clasificador),
    )

    return LaunchDescription([
        start_camera_arg,
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        start_clasificador_arg,
        roi_x_min_arg,
        roi_x_max_arg,
        roi_y_min_arg,
        roi_y_max_arg,
        camera_node,
        api_recognition_node,
        clasificador_node,
    ])
