#!/usr/bin/env python3

"""
Nodo de reconocimiento de figuras usando la API de Roboflow.

IMPORTANTE (modo bajo demanda, no continuo):
  Este nodo NO llama a la API automáticamente a una frecuencia fija.
  Solo realiza una inferencia cuando recibe un disparo explícito en:
    - /trigger_scan (std_msgs/Bool, data=true)
  Ese disparo llega desde:
    - El botón "Scan" de la GUI (solo consulta la API y muestra el
      resultado; NO mueve el robot).
    - El nodo clasificador, al finalizar un ciclo de pick & place
      (para tener una detección fresca lista para el siguiente ciclo).
  Esto evita saturar la API de Roboflow con llamadas continuas.

La interfaz con el resto del sistema:
  - Publica /figure_state con el resultado de cada inferencia bajo demanda.
  - El movimiento del robot NO depende de este nodo: la GUI decide cuándo
    publicar /figure_type (botón Start), que es lo que escucha
    clasificador_node para iniciar una trayectoria.
  - Se pausa con /routine_busy (no se atienden triggers mientras una
    rutina está en ejecución).

Configuración vía parámetros ROS o variables de entorno:
  - PINCHER_API_KEY: API key de Roboflow
  - PINCHER_API_URL: URL del endpoint (opcional, se construye con model_id)
  - PINCHER_MODEL_ID: ID del modelo en Roboflow (ej: "mi-modelo/1")

Las variables anteriores también se pueden definir en un archivo ".env"
(basado en "config/.env.example") dentro del paquete pincher_control, para
no tener que exportarlas manualmente en cada terminal. Ese archivo NUNCA
se sube a git (ver .gitignore).
"""

import os
import base64
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32MultiArray

try:
    from dotenv import load_dotenv
    HAS_DOTENV = True
except ImportError:
    HAS_DOTENV = False


def _load_pincher_env() -> None:
    """Carga PINCHER_API_KEY / PINCHER_MODEL_ID desde un archivo .env local.

    Busca ".env" en, en este orden:
      1. share/pincher_control/config/.env (paquete instalado)
      2. <workspace>/src/pincher_control/config/.env (desarrollo, sin instalar)

    No sobreescribe variables que ya existan en el entorno (por ejemplo, si
    el usuario las exportó manualmente antes de lanzar).
    """
    if not HAS_DOTENV:
        return

    candidate_paths = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("pincher_control")
        candidate_paths.append(Path(share_dir) / "config" / ".env")
    except Exception:
        pass

    # Fallback para ejecución directa sin instalar (p.ej. tests locales)
    candidate_paths.append(
        Path(__file__).resolve().parent.parent / "config" / ".env"
    )

    for env_path in candidate_paths:
        if env_path.is_file():
            load_dotenv(dotenv_path=env_path, override=False)
            break


_load_pincher_env()

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

try:
    from urllib.request import Request, urlopen
    from urllib.parse import urlencode
    import json
    HAS_URLLIB = True
except ImportError:
    HAS_URLLIB = False


class RecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__("recognition_node")

        # ----------------------------
        # Parámetros ROS
        # ----------------------------
        self.declare_parameter("image_topic", "")
        self.declare_parameter("api_key", "")
        self.declare_parameter("api_url", "")
        self.declare_parameter("model_id", "")
        self.declare_parameter("api_backend", "roboflow")
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("publish_roi", True)

        # ROI centrado en la pantalla por defecto (zona cuadrada al centro)
        self.declare_parameter("roi_x_min_pct", 0.35)
        self.declare_parameter("roi_x_max_pct", 0.65)
        self.declare_parameter("roi_y_min_pct", 0.35)
        self.declare_parameter("roi_y_max_pct", 0.65)

        # Resolver imagen topic
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value.strip()
        )
        if not image_topic:
            image_topic = os.environ.get("PINCHER_IMAGE_TOPIC", "/image_raw").strip() or "/image_raw"

        # API config
        self.api_key = (
            self.get_parameter("api_key").get_parameter_value().string_value.strip()
            or os.environ.get("PINCHER_API_KEY", "").strip()
        )
        self.api_url = (
            self.get_parameter("api_url").get_parameter_value().string_value.strip()
            or os.environ.get("PINCHER_API_URL", "").strip()
        )
        self.model_id = (
            self.get_parameter("model_id").get_parameter_value().string_value.strip()
            or os.environ.get("PINCHER_MODEL_ID", "").strip()
        )
        self.api_backend = (
            self.get_parameter("api_backend").get_parameter_value().string_value.strip()
            or os.environ.get("PINCHER_API_BACKEND", "roboflow").strip()
        ).lower()

        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_roi = bool(self.get_parameter("publish_roi").value)

        # Construir URL si no se proporcionó explícitamente
        if not self.api_url and self.model_id:
            if self.api_backend == "roboflow":
                # Roboflow Classification API
                self.api_url = f"https://classify.roboflow.com/{self.model_id}"
            elif self.api_backend == "ultralytics":
                # Ultralytics HUB API
                self.api_url = "https://predict.ultralytics.com"

        if not self.api_key:
            self.get_logger().error(
                "No se configuró API key. "
                "Setea PINCHER_API_KEY o el parámetro 'api_key'."
            )
        if not self.api_url:
            self.get_logger().error(
                "No se configuró API URL. "
                "Setea PINCHER_API_URL, PINCHER_MODEL_ID o los parámetros correspondientes."
            )

        # Suscripciones y publicadores
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        # NOTA: /figure_type ya NO lo publica este nodo. Ahora la GUI decide
        # cuándo publicarlo (botón Start), usando la última detección
        # conocida en /figure_state.
        self.figure_state_pub = self.create_publisher(String, "/figure_state", 10)
        self.debug_pub = self.create_publisher(Image, "/camera/debug", 10)
        self.roi_pub = self.create_publisher(Image, "/camera/roi", 10)

        self.vision_enabled = True
        self.busy_sub = self.create_subscription(
            Bool, "/routine_busy", self.busy_callback, 10
        )
        # Disparo explícito de una única inferencia (modo bajo demanda):
        #   - GUI → botón "Scan" (solo consulta la API, no mueve el robot).
        #   - clasificador_node → al finalizar un ciclo de pick & place.
        self.trigger_scan_sub = self.create_subscription(
            Bool, "/trigger_scan", self.trigger_scan_callback, 10
        )
        # Permite ajustar el ROI en vivo desde la GUI:
        # data = [x_min_pct, x_max_pct, y_min_pct, y_max_pct]
        self.roi_config_sub = self.create_subscription(
            Float32MultiArray, "/roi_config", self.roi_config_callback, 10
        )

        self.bridge = CvBridge()

        # ROI
        self.roi_x_min_pct = float(self.get_parameter("roi_x_min_pct").value)
        self.roi_x_max_pct = float(self.get_parameter("roi_x_max_pct").value)
        self.roi_y_min_pct = float(self.get_parameter("roi_y_min_pct").value)
        self.roi_y_max_pct = float(self.get_parameter("roi_y_max_pct").value)

        # Última imagen recibida de la cámara (se guarda en cada frame, pero
        # NO se envía a la API hasta que llegue un /trigger_scan explícito).
        self._latest_frame = None

        # Bandera para procesar como máximo un trigger pendiente a la vez.
        self._scan_pending = False

        # Estado para overlay
        self._last_detected_class = "unknown"
        self._last_confidence = 0.0

        self.get_logger().info(
            f"API Recognition Node inicializado (modo BAJO DEMANDA) | "
            f"backend={self.api_backend} | thr={self.confidence_threshold}"
        )
        self.get_logger().info(
            "La API solo se consulta al recibir /trigger_scan "
            "(botón Scan de la GUI o fin de ciclo del clasificador)."
        )
        self.get_logger().info(f"Suscrito a: {image_topic}")
        if self.api_url:
            self.get_logger().info(f"API URL: {self.api_url}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def busy_callback(self, msg: Bool) -> None:
        """Mientras la FSM está ejecutando una rutina, se ignoran los triggers
        de escaneo (no tiene sentido consultar la API mientras el robot se
        está moviendo hacia una figura que ya fue recogida)."""
        self.vision_enabled = not bool(msg.data)

    def roi_config_callback(self, msg: Float32MultiArray) -> None:
        """Actualiza el ROI en caliente. data = [x_min, x_max, y_min, y_max] (0.0-1.0)."""
        if len(msg.data) != 4:
            self.get_logger().warn(f"roi_config inválido: se esperaban 4 valores, llegaron {len(msg.data)}")
            return

        x_min, x_max, y_min, y_max = msg.data
        # Validación básica de rango
        x_min = max(0.0, min(1.0, x_min))
        x_max = max(0.0, min(1.0, x_max))
        y_min = max(0.0, min(1.0, y_min))
        y_max = max(0.0, min(1.0, y_max))
        if x_min >= x_max or y_min >= y_max:
            self.get_logger().warn(f"roi_config inválido: rangos min >= max ({msg.data})")
            return

        self.roi_x_min_pct = x_min
        self.roi_x_max_pct = x_max
        self.roi_y_min_pct = y_min
        self.roi_y_max_pct = y_max
        self.get_logger().info(
            f"ROI actualizado: X[{x_min:.2f}-{x_max:.2f}] Y[{y_min:.2f}-{y_max:.2f}]"
        )

    def image_callback(self, msg: Image) -> None:
        """Guarda el último frame y publica overlays de depuración.

        NO llama a la API aquí. La API solo se consulta en
        trigger_scan_callback(), para no saturarla con llamadas continuas.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        self._latest_frame = cv_image

        height, width, _ = cv_image.shape
        x_min = int(width * self.roi_x_min_pct)
        x_max = int(width * self.roi_x_max_pct)
        y_min = int(height * self.roi_y_min_pct)
        y_max = int(height * self.roi_y_max_pct)
        roi = cv_image[y_min:y_max, x_min:x_max]

        self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)

    def trigger_scan_callback(self, msg: Bool) -> None:
        """Realiza UNA inferencia contra la API cuando llega un trigger.

        Se dispara desde:
          - GUI (botón "Scan"): solo consulta y muestra el resultado.
          - clasificador_node: al finalizar un ciclo, para tener una
            detección fresca lista para el próximo Start.
        """
        if not msg.data:
            return

        if not self.vision_enabled:
            self.get_logger().warn(
                "Trigger de escaneo ignorado: hay una rutina en ejecución "
                "(/routine_busy = true)."
            )
            return

        if self._latest_frame is None:
            self.get_logger().warn(
                "Trigger de escaneo ignorado: aún no se ha recibido ninguna "
                "imagen de la cámara."
            )
            return

        if self._scan_pending:
            self.get_logger().warn(
                "Ya hay un escaneo en curso; se ignora este trigger."
            )
            return

        self._scan_pending = True
        try:
            cv_image = self._latest_frame
            height, width, _ = cv_image.shape
            x_min = int(width * self.roi_x_min_pct)
            x_max = int(width * self.roi_x_max_pct)
            y_min = int(height * self.roi_y_min_pct)
            y_max = int(height * self.roi_y_max_pct)
            roi = cv_image[y_min:y_max, x_min:x_max]

            self.get_logger().info("📷 Trigger de escaneo recibido: consultando API...")
            detected_class, confidence = self._infer_api(roi)
            self._last_detected_class = detected_class
            self._last_confidence = confidence

            # Estado continuo (para que la GUI muestre "última detección")
            state_msg = String()
            state_msg.data = detected_class
            self.figure_state_pub.publish(state_msg)

            self.get_logger().info(
                f"Resultado del escaneo: {detected_class} (confianza={confidence:.2f})"
            )

            self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)
        finally:
            self._scan_pending = False

    # ------------------------------------------------------------------
    # Inferencia via API
    # ------------------------------------------------------------------
    def _infer_api(self, roi: np.ndarray) -> tuple:
        """Envía el ROI a la API y retorna (clase, confianza)."""
        if not self.api_key or not self.api_url:
            return "unknown", 0.0

        try:
            # Codificar imagen como JPEG en base64
            _, buffer = cv2.imencode(".jpg", roi)
            img_base64 = base64.b64encode(buffer).decode("utf-8")

            if self.api_backend == "roboflow":
                return self._call_roboflow(img_base64)
            elif self.api_backend == "ultralytics":
                return self._call_ultralytics(roi)
            else:
                self.get_logger().error(f"Backend desconocido: {self.api_backend}")
                return "unknown", 0.0

        except Exception as e:
            self.get_logger().warn(f"Error en API inference: {e}")
            return "unknown", 0.0

    def _call_roboflow(self, img_base64: str) -> tuple:
        """
        Llama a la API de clasificación de Roboflow.
        Endpoint: https://classify.roboflow.com/{model_id}?api_key=XXX
        Body: imagen en base64.
        Respuesta: {"predictions": [{"class": "cubo", "confidence": 0.95}, ...]}
        """
        url = f"{self.api_url}?api_key={self.api_key}"

        if HAS_REQUESTS:
            resp = requests.post(
                url,
                data=img_base64,
                headers={"Content-Type": "application/x-www-form-urlencoded"},
                timeout=5,
            )
            result = resp.json()
        elif HAS_URLLIB:
            req = Request(
                url,
                data=img_base64.encode("utf-8"),
                headers={"Content-Type": "application/x-www-form-urlencoded"},
                method="POST",
            )
            with urlopen(req, timeout=5) as resp:
                result = json.loads(resp.read().decode("utf-8"))
        else:
            self.get_logger().error("No hay requests ni urllib disponible")
            return "unknown", 0.0

        # Parsear respuesta de Roboflow.
        # Puede venir en dos formatos:
        #   - Classification: {"top": "cubo", "confidence": 0.95}
        #   - Object Detection: {"predictions": [{"class": ..., "confidence": ...,
        #       "x", "y", "width", "height", "detection_id"}, ...]}
        # En Object Detection, Roboflow NO garantiza que predictions[0] sea la
        # de mayor confianza (puede haber varios objetos en el mismo ROI), así
        # que seleccionamos explícitamente la de mayor "confidence".
        predictions = result.get("predictions", [])
        if not predictions:
            # Puede venir como top/confidence directamente (Classification)
            top_class = result.get("top", "unknown")
            confidence = float(result.get("confidence", 0.0))
        else:
            if len(predictions) > 1:
                self.get_logger().warn(
                    f"La API devolvió {len(predictions)} detecciones en el ROI; "
                    "se usará la de mayor confianza. Verifica que solo haya una "
                    "figura en la zona de recolección."
                )
            best = max(predictions, key=lambda p: float(p.get("confidence", 0.0)))
            top_class = best.get("class", "unknown")
            confidence = float(best.get("confidence", 0.0))

        if confidence >= self.confidence_threshold:
            return top_class, confidence
        else:
            return "unknown", confidence

    def _call_ultralytics(self, roi: np.ndarray) -> tuple:
        """
        Llama a la API de Ultralytics HUB.
        Endpoint: https://predict.ultralytics.com
        Headers: x-api-key
        Body: multipart con imagen.
        """
        if not HAS_REQUESTS:
            self.get_logger().error("Se necesita 'requests' para Ultralytics HUB API")
            return "unknown", 0.0

        _, buffer = cv2.imencode(".jpg", roi)
        files = {"file": ("roi.jpg", buffer.tobytes(), "image/jpeg")}
        headers = {"x-api-key": self.api_key}
        data = {}
        if self.model_id:
            data["model"] = self.model_id

        resp = requests.post(
            self.api_url,
            headers=headers,
            files=files,
            data=data,
            timeout=10,
        )
        result = resp.json()

        # Parsear respuesta
        # La respuesta típica: {"data": [{"class": ..., "confidence": ...}]}
        # o {"results": [...]}
        predictions = result.get("data", result.get("results", []))
        if isinstance(predictions, list) and predictions:
            best = predictions[0]
            top_class = best.get("class", best.get("name", "unknown"))
            confidence = float(best.get("confidence", 0.0))
        else:
            return "unknown", 0.0

        if confidence >= self.confidence_threshold:
            return top_class, confidence
        else:
            return "unknown", confidence

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    def _draw_and_publish(self, cv_image, roi, x_min, y_min, x_max, y_max) -> None:
        detected_class = self._last_detected_class
        confidence = self._last_confidence

        color = (
            (0, 255, 0) if detected_class not in ("unknown", "vacio") else (0, 0, 255)
        )
        cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)
        label = f"{detected_class} ({confidence:.2f})"
        cv2.putText(
            cv_image, label, (x_min, y_min - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
        )

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

        if self.publish_roi:
            try:
                self.roi_pub.publish(self.bridge.cv2_to_imgmsg(roi, "bgr8"))
            except CvBridgeError as e:
                self.get_logger().error(f"Error publishing ROI image: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
