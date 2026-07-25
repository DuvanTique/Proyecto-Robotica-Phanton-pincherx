# `recognition_node.py` — Nodo de Reconocimiento de Figuras (API Roboflow)

## Descripción

Nodo ROS 2 que realiza inferencia de detección de objetos mediante la **API REST de Roboflow** en modo **bajo demanda**. No consulta la API de forma continua; solo ejecuta una inferencia cuando recibe un disparo explícito en el tópico `/trigger_scan`.

## Rol en el Sistema

- Recibe imágenes de la cámara y las almacena internamente (sin enviarlas a la API).
- Al recibir `/trigger_scan`, extrae el ROI de la última imagen, lo envía a Roboflow y publica el resultado en `/figure_state`.
- La GUI y el clasificador consultan `/figure_state` para saber qué figura hay en la bandeja.
- El movimiento del robot **NO** se inicia desde este nodo; esa decisión la toma la GUI al publicar `/figure_type`.

## Suscripciones

| Tópico | Tipo | Descripción |
|---|---|---|
| `/image_raw` (configurable) | `sensor_msgs/Image` | Flujo de vídeo de la cámara |
| `/trigger_scan` | `std_msgs/Bool` | Dispara una única inferencia contra la API |
| `/routine_busy` | `std_msgs/Bool` | Si `True`, ignora triggers de escaneo |
| `/roi_config` | `std_msgs/Float32MultiArray` | Ajuste dinámico del ROI `[x_min, x_max, y_min, y_max]` |

## Publicaciones

| Tópico | Tipo | Descripción |
|---|---|---|
| `/figure_state` | `std_msgs/String` | Clase detectada tras cada escaneo (`cubo`, `cilindro`, `unknown`, etc.) |
| `/camera/debug` | `sensor_msgs/Image` | Imagen completa con overlay del ROI y detección |
| `/camera/roi` | `sensor_msgs/Image` | Recorte del ROI (solo la zona de detección) |

## Parámetros

| Parámetro | Defecto | Descripción |
|---|---|---|
| `api_key` | `""` (lee de `.env`) | API key de Roboflow |
| `model_id` | `""` (lee de `.env`) | ID del modelo (ej: `"mi-proyecto/1"`) |
| `api_backend` | `"roboflow"` | Backend de inferencia (`roboflow` o `ultralytics`) |
| `confidence_threshold` | `0.7` | Confianza mínima para aceptar una detección |
| `image_topic` | `"/image_raw"` | Tópico de imagen de entrada |
| `roi_x_min_pct` | `0.35` | Límite izquierdo del ROI (0.0–1.0) |
| `roi_x_max_pct` | `0.65` | Límite derecho del ROI (0.0–1.0) |
| `roi_y_min_pct` | `0.35` | Límite superior del ROI (0.0–1.0) |
| `roi_y_max_pct` | `0.65` | Límite inferior del ROI (0.0–1.0) |
| `publish_roi` | `True` | Si publica el recorte del ROI en `/camera/roi` |

## Carga Automática de Credenciales

El nodo busca un archivo `.env` al iniciar (usando `python-dotenv`):

1. `share/pincher_control/config/.env` (paquete instalado)
2. `<workspace>/src/pincher_control/config/.env` (desarrollo local)

No sobrescribe variables ya exportadas en el entorno.

## Lógica de Inferencia (Bajo Demanda)

```
image_callback() → Guarda frame, dibuja overlay → NO llama a la API
trigger_scan_callback() → Extrae ROI del último frame → Llama a la API → Publica /figure_state
```

Si la API devuelve múltiples detecciones (Object Detection), se selecciona la de **mayor confianza** y se emite un warning en los logs.

## Uso

```bash
# Se lanza automáticamente desde vision_bringup.launch.py
ros2 launch phantomx_pincher_bringup vision_bringup.launch.py \
  start_camera:=true camera_device:=/dev/video4

# Disparar un escaneo manualmente
ros2 topic pub -1 /trigger_scan std_msgs/msg/Bool "{data: true}"

# Ver el resultado
ros2 topic echo /figure_state
```
