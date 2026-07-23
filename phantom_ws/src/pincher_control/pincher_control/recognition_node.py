#!/usr/bin/env python3

"""
Nodo de reconocimiento de figuras usando la API de Roboflow.

La interfaz con el resto del sistema:
  - Publica /figure_type cuando la detección es estable.
  - Publica /figure_state con el estado continuo.
  - Se pausa con /routine_busy.

Configuración vía parámetros ROS o variables de entorno:
  - PINCHER_API_KEY: API key de Roboflow
  - PINCHER_API_URL: URL del endpoint (opcional, se construye con model_id)
  - PINCHER_MODEL_ID: ID del modelo en Roboflow (ej: "mi-modelo/1")
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
from std_msgs.msg import String, Bool

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
        self.declare_parameter("inference_hz", 1.0)
        self.declare_parameter("publish_roi", True)

        self.declare_parameter("roi_x_min_pct", 0.45)
        self.declare_parameter("roi_x_max_pct", 0.60)
        self.declare_parameter("roi_y_min_pct", 0.62)
        self.declare_parameter("roi_y_max_pct", 0.77)

        # Estabilización
        self.declare_parameter("buffer_size", 5)
        self.declare_parameter("vacio_reset_count", 3)

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
        self.figure_pub = self.create_publisher(String, "/figure_type", 10)
        self.figure_state_pub = self.create_publisher(String, "/figure_state", 10)
        self.debug_pub = self.create_publisher(Image, "/camera/debug", 10)
        self.roi_pub = self.create_publisher(Image, "/camera/roi", 10)

        self.vision_enabled = True
        self._was_busy = False
        self.busy_sub = self.create_subscription(
            Bool, "/routine_busy", self.busy_callback, 10
        )

        self.bridge = CvBridge()

        # ROI
        self.roi_x_min_pct = float(self.get_parameter("roi_x_min_pct").value)
        self.roi_x_max_pct = float(self.get_parameter("roi_x_max_pct").value)
        self.roi_y_min_pct = float(self.get_parameter("roi_y_min_pct").value)
        self.roi_y_max_pct = float(self.get_parameter("roi_y_max_pct").value)

        # Buffer de estabilización
        self.detection_buffer = []
        self.buffer_size = int(self.get_parameter("buffer_size").value)
        self.last_published_figure = ""
        self.vacio_streak = 0
        self.vacio_reset_count = int(self.get_parameter("vacio_reset_count").value)

        # Control de frecuencia
        self.last_inference_time = 0.0
        inference_hz = float(self.get_parameter("inference_hz").value)
        inference_hz = max(inference_hz, 0.1)
        self.inference_interval = 1.0 / inference_hz

        # Estado para overlay
        self._last_detected_class = "unknown"
        self._last_confidence = 0.0

        self.get_logger().info(
            f"API Recognition Node inicializado | backend={self.api_backend} | "
            f"hz={inference_hz:.1f} | thr={self.confidence_threshold}"
        )
        self.get_logger().info(f"Suscrito a: {image_topic}")
        if self.api_url:
            self.get_logger().info(f"API URL: {self.api_url}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def busy_callback(self, msg: Bool) -> None:
        was_busy = self._was_busy
        is_busy = bool(msg.data)
        self.vision_enabled = not is_busy
        self._was_busy = is_busy

        if was_busy and not is_busy:
            self.get_logger().info("Rutina finalizada → rearmando detección API.")
            self.detection_buffer = []
            self.last_published_figure = ""
            self.vacio_streak = 0

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        height, width, _ = cv_image.shape
        x_min = int(width * self.roi_x_min_pct)
        x_max = int(width * self.roi_x_max_pct)
        y_min = int(height * self.roi_y_min_pct)
        y_max = int(height * self.roi_y_max_pct)
        roi = cv_image[y_min:y_max, x_min:x_max]

        if not self.vision_enabled:
            self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_inference_time >= self.inference_interval:
            self.last_inference_time = now

            detected_class, confidence = self._infer_api(roi)
            self._last_detected_class = detected_class
            self._last_confidence = confidence

            # Estado continuo
            state_msg = String()
            state_msg.data = detected_class
            self.figure_state_pub.publish(state_msg)

            # Buffer de estabilización
            if detected_class not in ("unknown", "vacio"):
                self._update_buffer(detected_class)
            else:
                self.detection_buffer = []

            # Rearmar con vacio
            if detected_class == "vacio":
                self.vacio_streak += 1
                if self.vacio_streak >= self.vacio_reset_count:
                    if self.last_published_figure:
                        self.get_logger().info("ROI 'vacio' estable → rearmando.")
                    self.last_published_figure = ""
            else:
                self.vacio_streak = 0

        self._draw_and_publish(cv_image, roi, x_min, y_min, x_max, y_max)

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

        # Parsear respuesta de Roboflow classify
        predictions = result.get("predictions", [])
        if not predictions:
            # Puede venir como top/confidence directamente
            top_class = result.get("top", "unknown")
            confidence = float(result.get("confidence", 0.0))
        else:
            # Lista ordenada por confianza
            best = predictions[0]
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

    def _update_buffer(self, shape: str) -> None:
        self.detection_buffer.append(shape)
        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        if len(self.detection_buffer) == self.buffer_size:
            if all(s == shape for s in self.detection_buffer):
                if shape != self.last_published_figure:
                    self.get_logger().info(f"Figura confirmada (API): {shape}")
                    msg = String()
                    msg.data = shape
                    self.figure_pub.publish(msg)
                    self.last_published_figure = shape


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
