#!/usr/bin/env python3
"""
Interfaz Gráfica de Usuario (GUI) para el sistema de clasificación
automatizada del PhantomX Pincher X100.

Visualización:
  - Imagen de la cámara con detecciones
  - Estado de la máquina de estados (FSM)
  - Conteo por color/figura clasificada
  - Figuras faltantes
  - Estado de MoveIt (plan válido / en ejecución / fallo)
  - Estado del gripper (abierto/cerrado)

Controles:
  - Start: iniciar ciclo automático
  - Stop: detener ciclo
  - Emergency Stop: parada de emergencia
  - Reset: limpiar fallas
  - Home: enviar robot a posición segura
  - Scan: capturar nueva imagen
  - Next Cube: ejecutar ciclo para siguiente figura (paso a paso)
  - Gripper Open / Close: control manual del gripper
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from PIL import Image as PILImage, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

import cv2
import numpy as np


# ============================================================================
# Estados posibles de la FSM
# ============================================================================
FSM_STATES = [
    "IDLE", "READY", "SCAN", "PLAN", "PICK",
    "VERIFY_GRIP", "DROP", "VERIFY_SORT", "FAULT", "DONE"
]

# Mapeo de figuras a canecas
FIGURE_MAP = {
    "cubo": "caneca_roja",
    "cilindro": "caneca_verde",
    "pentagono": "caneca_azul",
    "rectangulo": "caneca_amarilla",
}

# Receta: cuántos de cada figura se esperan
RECIPE = {
    "cubo": 3,
    "cilindro": 3,
    "pentagono": 3,
    "rectangulo": 3,
}


class GuiRosNode(Node):
    """Nodo ROS 2 ligero para la GUI."""

    def __init__(self):
        super().__init__("pincher_gui")

        # Publishers
        self.figure_pub = self.create_publisher(String, "/figure_type", 10)
        self.gripper_pub = self.create_publisher(Bool, "/set_gripper", 10)
        self.busy_pub = self.create_publisher(Bool, "/routine_busy", 10)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop", 10)

        # Publisher para enviar comandos directos de joints al commander
        from example_interfaces.msg import Float64MultiArray
        from phantomx_pincher_interfaces.msg import PoseCommand
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, "/joint_command", 10)
        self.pose_cmd_pub = self.create_publisher(PoseCommand, "/pose_command", 10)

        # Publisher para ajustar el ROI de detección en vivo
        self.roi_config_pub = self.create_publisher(Float32MultiArray, "/roi_config", 10)

        # Dispara una única consulta a la API de reconocimiento (botón Scan).
        # NO mueve el robot; solo actualiza /figure_state con el resultado.
        self.trigger_scan_pub = self.create_publisher(Bool, "/trigger_scan", 10)

        # Subscribers
        self.figure_state_sub = self.create_subscription(
            String, "/figure_state", self._figure_state_cb, 10
        )
        self.routine_busy_sub = self.create_subscription(
            Bool, "/routine_busy", self._busy_cb, 10
        )
        self.debug_image_sub = self.create_subscription(
            Image, "/camera/debug", self._image_cb, 10
        )
        self.torque_status_sub = self.create_subscription(
            Bool, "/torque_status", self._torque_status_cb, 10
        )

        self.bridge = CvBridge()

        # State
        self.current_figure_state = "unknown"
        self.is_busy = False
        self.latest_cv_image: Optional[np.ndarray] = None
        self.image_lock = threading.Lock()
        self.torque_enabled: Optional[bool] = None  # None = sin datos (sim o no reportado)
        self.torque_last_update = 0.0

    def _figure_state_cb(self, msg: String):
        self.current_figure_state = msg.data

    def _busy_cb(self, msg: Bool):
        self.is_busy = msg.data

    def _torque_status_cb(self, msg: Bool):
        self.torque_enabled = bool(msg.data)
        self.torque_last_update = time.time()

    def _image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.latest_cv_image = cv_img
        except Exception:
            pass

    def publish_figure(self, figure: str):
        """Publica /figure_type: esto es lo único que hace que el robot se
        mueva (clasificador_node arranca su FSM al recibir este mensaje)."""
        msg = String()
        msg.data = figure
        self.figure_pub.publish(msg)

    def publish_trigger_scan(self):
        """Dispara una única consulta a la API (botón Scan). No mueve el
        robot; solo actualiza /figure_state con el resultado."""
        msg = Bool()
        msg.data = True
        self.trigger_scan_pub.publish(msg)

    def publish_home(self):
        """Envía joint_command con articulaciones en posición home (1°)."""
        from example_interfaces.msg import Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [0.01745, 0.01745, 0.01745, 0.01745]
        self.joint_cmd_pub.publish(msg)

    def publish_roi_config(self, x_min, x_max, y_min, y_max):
        """Envía la nueva configuración de ROI al recognition_node."""
        msg = Float32MultiArray()
        msg.data = [float(x_min), float(x_max), float(y_min), float(y_max)]
        self.roi_config_pub.publish(msg)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        """Envía un PoseCommand al commander."""
        from phantomx_pincher_interfaces.msg import PoseCommand
        msg = PoseCommand()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        msg.cartesian_path = False
        self.pose_cmd_pub.publish(msg)

    def publish_gripper(self, open_gripper: bool):
        msg = Bool()
        msg.data = open_gripper
        self.gripper_pub.publish(msg)

    def publish_emergency_stop(self):
        """Publica señal de parada de emergencia."""
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)
        # También liberar busy
        busy_msg = Bool()
        busy_msg.data = False
        self.busy_pub.publish(busy_msg)


class PincherGUI:
    """Ventana principal de la interfaz gráfica."""

    def __init__(self, ros_node: GuiRosNode):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("PhantomX Pincher X100 - Clasificación de Figuras")
        self.root.geometry("1100x700")
        self.root.configure(bg="#2b2b2b")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Tracking
        self.classified_count = {"cubo": 0, "cilindro": 0, "pentagono": 0, "rectangulo": 0}
        self.fsm_state = "IDLE"
        self.is_fault = False
        self.auto_mode = False

        self._build_ui()

        # Periodic updates
        self.root.after(50, self._spin_ros)
        self.root.after(200, self._update_display)

    # =========================================================================
    # UI Construction
    # =========================================================================
    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Title.TLabel", font=("Segoe UI", 14, "bold"),
                        background="#2b2b2b", foreground="white")
        style.configure("Info.TLabel", font=("Segoe UI", 10),
                        background="#2b2b2b", foreground="#cccccc")
        style.configure("State.TLabel", font=("Segoe UI", 12, "bold"),
                        background="#2b2b2b", foreground="#00ff88")
        style.configure("Fault.TLabel", font=("Segoe UI", 12, "bold"),
                        background="#2b2b2b", foreground="#ff4444")
        style.configure("Count.TLabel", font=("Segoe UI", 11),
                        background="#333333", foreground="white")

        # ---- Top header ----
        header = tk.Frame(self.root, bg="#1a1a1a", height=50)
        header.pack(fill="x")
        ttk.Label(header, text="🤖 PhantomX Pincher - Sistema de Clasificación",
                  style="Title.TLabel", background="#1a1a1a").pack(pady=10)

        # ---- Main content ----
        main_frame = tk.Frame(self.root, bg="#2b2b2b")
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Left: Camera + detection
        left_frame = tk.Frame(main_frame, bg="#2b2b2b")
        left_frame.pack(side="left", fill="both", expand=True)

        ttk.Label(left_frame, text="📷 Cámara / Detección", style="Info.TLabel").pack(anchor="w")
        self.camera_canvas = tk.Canvas(left_frame, width=480, height=360, bg="#111111",
                                       highlightthickness=1, highlightbackground="#555")
        self.camera_canvas.pack(pady=5)

        # Detection info
        det_frame = tk.Frame(left_frame, bg="#333333", padx=10, pady=5)
        det_frame.pack(fill="x", pady=5)
        ttk.Label(det_frame, text="Última detección:", style="Info.TLabel",
                  background="#333333").pack(side="left")
        self.detection_var = tk.StringVar(value="---")
        ttk.Label(det_frame, textvariable=self.detection_var, style="Count.TLabel").pack(side="left", padx=10)

        # Right panel
        right_frame = tk.Frame(main_frame, bg="#2b2b2b", width=380)
        right_frame.pack(side="right", fill="y", padx=(10, 0))
        right_frame.pack_propagate(False)

        # --- FSM State ---
        state_frame = tk.LabelFrame(right_frame, text="Estado del Sistema",
                                    bg="#333333", fg="white", font=("Segoe UI", 10, "bold"))
        state_frame.pack(fill="x", pady=5)

        self.state_var = tk.StringVar(value="IDLE")
        self.state_label = ttk.Label(state_frame, textvariable=self.state_var, style="State.TLabel",
                                     background="#333333")
        self.state_label.pack(pady=8)

        # --- Conteo ---
        count_frame = tk.LabelFrame(right_frame, text="Clasificación (completado / total)",
                                    bg="#333333", fg="white", font=("Segoe UI", 10, "bold"))
        count_frame.pack(fill="x", pady=5)

        self.count_labels = {}
        colors = {"cubo": "#ff4444", "cilindro": "#44ff44",
                  "pentagono": "#4488ff", "rectangulo": "#ffff44"}
        for fig, color in colors.items():
            row = tk.Frame(count_frame, bg="#333333")
            row.pack(fill="x", padx=10, pady=2)
            tk.Label(row, text=f"● {fig.capitalize()}", bg="#333333", fg=color,
                     font=("Segoe UI", 10)).pack(side="left")
            lbl = tk.Label(row, text="0 / 3", bg="#333333", fg="white",
                           font=("Segoe UI", 10, "bold"))
            lbl.pack(side="right")
            self.count_labels[fig] = lbl

        # --- Faltantes ---
        self.remaining_var = tk.StringVar(value="Faltantes: 12")
        ttk.Label(right_frame, textvariable=self.remaining_var, style="Info.TLabel").pack(pady=5)

        # --- MoveIt status ---
        moveit_frame = tk.LabelFrame(right_frame, text="MoveIt / Gripper",
                                     bg="#333333", fg="white", font=("Segoe UI", 10, "bold"))
        moveit_frame.pack(fill="x", pady=5)

        self.moveit_var = tk.StringVar(value="Listo")
        ttk.Label(moveit_frame, textvariable=self.moveit_var, style="Info.TLabel",
                  background="#333333").pack(pady=3)

        self.gripper_var = tk.StringVar(value="Gripper: ---")
        ttk.Label(moveit_frame, textvariable=self.gripper_var, style="Info.TLabel",
                  background="#333333").pack(pady=3)

        self.torque_var = tk.StringVar(value="⚪ Torque: sin datos")
        self.torque_label = tk.Label(moveit_frame, textvariable=self.torque_var,
                                     bg="#333333", fg="#aaaaaa", font=("Segoe UI", 10, "bold"))
        self.torque_label.pack(pady=3)

        # --- ROI de detección (ajustable) ---
        roi_frame = tk.LabelFrame(right_frame, text="Zona de Detección (ROI)",
                                  bg="#333333", fg="white", font=("Segoe UI", 10, "bold"))
        roi_frame.pack(fill="x", pady=5)

        self.roi_sliders = {}
        roi_defaults = {"X min": 35, "X max": 65, "Y min": 35, "Y max": 65}
        for label, default in roi_defaults.items():
            row = tk.Frame(roi_frame, bg="#333333")
            row.pack(fill="x", padx=8, pady=2)
            tk.Label(row, text=label, bg="#333333", fg="white", width=6,
                     font=("Segoe UI", 9)).pack(side="left")
            var = tk.IntVar(value=default)
            scale = tk.Scale(row, from_=0, to=100, orient="horizontal", variable=var,
                             bg="#333333", fg="white", troughcolor="#555555",
                             highlightthickness=0, length=180,
                             command=lambda v, l=label: self._on_roi_change())
            scale.pack(side="left", fill="x", expand=True)
            self.roi_sliders[label] = var

        tk.Button(roi_frame, text="Aplicar ROI", command=self._on_apply_roi,
                  bg="#6B3FA0", fg="white", font=("Segoe UI", 9, "bold"),
                  relief="flat", padx=6, pady=4, cursor="hand2").pack(pady=6)

        # ---- Controls ----
        ctrl_frame = tk.Frame(self.root, bg="#1a1a1a")
        ctrl_frame.pack(fill="x", pady=5)

        buttons = [
            ("▶ Start", self._on_start, "#228B22"),
            ("⏹ Stop", self._on_stop, "#B8860B"),
            ("⚠ E-STOP", self._on_estop, "#cc0000"),
            ("🔄 Reset", self._on_reset, "#555555"),
            ("🏠 Home", self._on_home, "#336699"),
            ("📷 Scan", self._on_scan, "#6B3FA0"),
            ("➡ Next", self._on_next_cube, "#2E86AB"),
            ("🔓 Open", self._on_gripper_open, "#448844"),
            ("🔒 Close", self._on_gripper_close, "#884444"),
        ]

        for text, cmd, color in buttons:
            btn = tk.Button(ctrl_frame, text=text, command=cmd,
                            bg=color, fg="white", font=("Segoe UI", 9, "bold"),
                            relief="flat", padx=8, pady=6, cursor="hand2")
            btn.pack(side="left", padx=3, pady=5)

        # Status bar
        self.status_var = tk.StringVar(value="Sistema listo. Presione Start para iniciar.")
        status_bar = tk.Label(self.root, textvariable=self.status_var,
                              bg="#1a1a1a", fg="#aaaaaa", font=("Segoe UI", 9),
                              anchor="w", padx=10)
        status_bar.pack(fill="x", side="bottom")

    # =========================================================================
    # Control Callbacks
    # =========================================================================
    def _on_start(self):
        """Inicia el ciclo automático: el robot solo se mueve al presionar
        este botón. Usa la última detección disponible en /figure_state
        (obtenida con Scan o al finalizar el ciclo anterior)."""
        if self.is_fault:
            self.status_var.set("⚠️  Sistema en FAULT. Haga Reset primero.")
            return

        detected = self.node.current_figure_state
        if detected not in FIGURE_MAP:
            self.status_var.set(
                f"⚠️  No hay una detección válida (actual: {detected}). "
                "Presione Scan primero."
            )
            return

        self.auto_mode = True
        self.fsm_state = "PLAN"
        self._update_state_display()
        self.node.publish_figure(detected)
        self.status_var.set(f"▶ Start: iniciando pick & place para {detected}...")

    def _on_stop(self):
        self.auto_mode = False
        self.fsm_state = "IDLE"
        # Enviar señal al clasificador para que aborte la secuencia en curso
        msg = Bool()
        msg.data = True
        self.node.estop_pub.publish(msg)
        self.status_var.set("Ciclo detenido. Secuencia abortada.")
        self._update_state_display()

    def _on_estop(self):
        self.auto_mode = False
        self.is_fault = True
        self.fsm_state = "FAULT"
        self.node.publish_emergency_stop()
        self.node.publish_gripper(True)  # Abrir gripper por seguridad
        self.status_var.set("⚠️  PARADA DE EMERGENCIA ACTIVADA — Torque deshabilitado")
        self._update_state_display()

    def _on_reset(self):
        self.is_fault = False
        self.fsm_state = "IDLE"
        # Enviar robot a home al hacer reset
        self.node.publish_home()
        self.status_var.set("Fallas limpiadas. Robot enviado a HOME.")
        self._update_state_display()

    def _on_home(self):
        if self.is_fault:
            self.status_var.set("⚠️  Sistema en FAULT. Haga Reset primero.")
            return
        self.node.publish_home()
        self.status_var.set("🏠 Robot enviado a posición HOME.")

    def _on_scan(self):
        """Dispara una única consulta a la API de Roboflow y muestra el
        resultado. NO mueve el robot. Para iniciar el movimiento, use Start."""
        if self.is_fault:
            self.status_var.set("⚠️  Sistema en FAULT. Haga Reset primero.")
            return
        if self.node.is_busy:
            self.status_var.set("⚠️  Hay una rutina en ejecución. Espere a que termine.")
            return
        self.node.publish_trigger_scan()
        self.status_var.set("📷 Escaneando... consultando la API de Roboflow.")

    def _on_next_cube(self):
        """Modo paso a paso: clasifica la siguiente figura detectada."""
        if self.is_fault:
            self.status_var.set("⚠️  Sistema en FAULT. Haga Reset primero.")
            return
        detected = self.node.current_figure_state
        if detected in FIGURE_MAP:
            self.fsm_state = "PLAN"
            self._update_state_display()
            self.node.publish_figure(detected)
            self.status_var.set(f"Ejecutando pick & place para: {detected}")
        else:
            self.status_var.set(f"No hay figura válida detectada (actual: {detected})")

    def _on_roi_change(self):
        """Solo actualiza el label visual mientras se arrastra el slider."""
        pass  # El envío real ocurre al presionar "Aplicar ROI"

    def _on_apply_roi(self):
        x_min = self.roi_sliders["X min"].get() / 100.0
        x_max = self.roi_sliders["X max"].get() / 100.0
        y_min = self.roi_sliders["Y min"].get() / 100.0
        y_max = self.roi_sliders["Y max"].get() / 100.0

        if x_min >= x_max or y_min >= y_max:
            self.status_var.set("⚠️  ROI inválido: min debe ser menor que max.")
            return

        self.node.publish_roi_config(x_min, x_max, y_min, y_max)
        self.status_var.set(
            f"ROI actualizado: X[{x_min:.2f}-{x_max:.2f}] Y[{y_min:.2f}-{y_max:.2f}]"
        )

    def _on_gripper_open(self):
        self.node.publish_gripper(True)
        self.gripper_var.set("Gripper: ABIERTO")
        self.status_var.set("Gripper abierto manualmente.")

    def _on_gripper_close(self):
        self.node.publish_gripper(False)
        self.gripper_var.set("Gripper: CERRADO")
        self.status_var.set("Gripper cerrado manualmente.")

    # =========================================================================
    # Display Updates
    # =========================================================================
    def _update_state_display(self):
        self.state_var.set(self.fsm_state)
        if self.fsm_state == "FAULT":
            self.state_label.configure(style="Fault.TLabel")
        else:
            self.state_label.configure(style="State.TLabel")

    def _update_display(self):
        """Actualización periódica de la GUI."""
        # Update detection
        detected = self.node.current_figure_state
        self.detection_var.set(detected if detected else "---")

        # Update FSM state based on busy — but only si la GUI inició un ciclo
        if self.node.is_busy and self.fsm_state not in ("FAULT", "DONE", "IDLE"):
            self.fsm_state = "PICK"
        elif not self.node.is_busy and self.fsm_state == "PICK":
            # El ciclo terminó. clasificador_node ya disparó un nuevo
            # /trigger_scan automáticamente; aquí solo reflejamos el estado.
            self.fsm_state = "IDLE"
            self.auto_mode = False
            # Incrementar conteo si se completó
            if detected in FIGURE_MAP:
                self.classified_count[detected] = min(
                    self.classified_count[detected] + 1,
                    RECIPE[detected]
                )

        self._update_state_display()

        # Update counts
        total_remaining = 0
        for fig, lbl in self.count_labels.items():
            done = self.classified_count[fig]
            total = RECIPE[fig]
            lbl.config(text=f"{done} / {total}")
            total_remaining += (total - done)

        self.remaining_var.set(f"Faltantes: {total_remaining} de 12")

        # Check done
        if total_remaining == 0 and self.fsm_state != "FAULT":
            self.fsm_state = "DONE"
            self.auto_mode = False
            self.status_var.set("✅ ¡Clasificación completa! 12/12 figuras clasificadas.")

        # Update MoveIt status
        if self.node.is_busy:
            self.moveit_var.set("🔄 Trayectoria en ejecución...")
        elif self.is_fault:
            self.moveit_var.set("❌ FAULT — Revise la celda")
        else:
            self.moveit_var.set("✅ Listo para planificar")

        # Update torque status
        self._update_torque_display()

        # Update camera image
        self._update_camera_image()

        self.root.after(200, self._update_display)

    def _update_torque_display(self):
        """Actualiza el indicador de estado de torque de los motores."""
        # Si no hemos recibido /torque_status en los últimos 3s, asumimos
        # que estamos en simulación (no hay hardware real conectado).
        no_data = (
            self.node.torque_enabled is None
            or (time.time() - self.node.torque_last_update) > 3.0
        )
        if no_data:
            self.torque_var.set("⚪ Torque: sin datos (¿simulación?)")
            self.torque_label.config(fg="#aaaaaa")
        elif self.node.torque_enabled:
            self.torque_var.set("🟢 Torque: HABILITADO")
            self.torque_label.config(fg="#44ff44")
        else:
            self.torque_var.set("🔴 Torque: DESHABILITADO")
            self.torque_label.config(fg="#ff4444")

    def _update_camera_image(self):
        """Actualiza el canvas con la última imagen de la cámara."""
        with self.node.image_lock:
            img = self.node.latest_cv_image

        if img is None:
            return

        if not HAS_PIL:
            return

        # Resize to canvas
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = img_rgb.shape[:2]
        canvas_w, canvas_h = 480, 360
        scale = min(canvas_w / w, canvas_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        img_resized = cv2.resize(img_rgb, (new_w, new_h))

        pil_img = PILImage.fromarray(img_resized)
        self._tk_img = ImageTk.PhotoImage(pil_img)
        self.camera_canvas.delete("all")
        self.camera_canvas.create_image(
            canvas_w // 2, canvas_h // 2, image=self._tk_img, anchor="center"
        )

    # =========================================================================
    # ROS Spin & Lifecycle
    # =========================================================================
    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(50, self._spin_ros)

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = GuiRosNode()
    gui = PincherGUI(node)
    try:
        gui.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
