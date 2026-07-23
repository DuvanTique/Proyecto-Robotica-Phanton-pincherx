#!/usr/bin/env python3

"""
Nodo de clasificación para PhantomX Pincher.

Este nodo orquesta operaciones de pick-and-place basadas en el tipo de figura detectada.
Suscribe al tipo de figura y publica comandos de pose para mover el robot a través de
una secuencia completa de recolección y colocación en la caneca correcta.

Mapeo de figuras a canecas:
- cubo      → caneca_roja
- cilindro  → caneca_verde
- pentagono → caneca_azul
- rectangulo→ caneca_amarilla
"""

import os
from enum import Enum

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_msgs.msg import Bool
from example_interfaces.msg import Float64MultiArray

from phantomx_pincher_interfaces.msg import PoseCommand

import yaml


class SequenceState(Enum):
    """Estados de la secuencia de pick-and-place."""

    IDLE = 0
    MOVING_TO_HOME_START = 1
    OPENING_GRIPPER_START = 2
    MOVING_TO_PICKUP = 3
    CLOSING_GRIPPER = 4
    MOVING_TO_HOME_WITH_OBJECT = 5
    MOVING_TO_SAFE_POS_1 = 6        # recoleccion_1 (aproximación)
    MOVING_TO_SAFE_POS_2 = 12       # recoleccion_2 -> recoleccion_1 (retorno)
    MOVING_TO_SAFE_POS_3 = 14       # recoleccion_1 -> home (retorno)
    MOVING_TO_SAFE_POS_4 = 16
    MOVING_TO_BIN = 7
    OPENING_GRIPPER_DROP = 8
    PRE_DROP = 18               # Aproximación vertical sobre la caneca
    RETURNING_TO_SAFE_POS_4 = 17
    RETURNING_TO_SAFE_POS_3 = 15
    RETURNING_TO_SAFE_POS_2 = 13
    RETURNING_TO_SAFE_POS_1 = 9
    RETURNING_TO_HOME_END = 10
    COMPLETED = 11


class ClasificadorNode(Node):
    """Nodo que ejecuta secuencias de pick-and-place basadas en el tipo de figura."""

    def __init__(self) -> None:
        super().__init__("clasificador_node")

        # Mapeo de figura a caneca
        self.figure_to_bin = {
            "cubo": "caneca_roja",
            "cilindro": "caneca_verde",
            "pentagono": "caneca_azul",
            "hexagono": "caneca_azul", # Mapeamos hexagono a la misma caneca por ahora
            "rectangulo": "caneca_amarilla",
        }

        # Mapeo de caneca a su pose de aproximación (pre_drop)
        self.bin_to_pre_drop = {
            "caneca_roja": "pre_drop_roja",
            "caneca_verde": "pre_drop_verde",
            "caneca_azul": "pre_drop_azul",
            "caneca_amarilla": "pre_drop_amarilla",
        }

        # --- CONFIGURACIÓN DE TIEMPOS ---
        # IMPORTANTE:
        # Usamos tiempos suficientemente grandes para asegurarnos de que
        # cada movimiento (plane + execute) termine ANTES de mandar la
        # siguiente pose. Si estos tiempos son muy pequeños, MoveIt
        # recibe varios objetivos seguidos y solo ejecuta el último.
        self.TIME_MOVEMENT = 6.0  # Tiempo para movimientos del brazo
        self.TIME_GRIPPER = 2.0   # Tiempo para abrir/cerrar gripper
        # -------------------------------

        # Estado de la secuencia
        self.current_state = SequenceState.IDLE
        self.current_bin = None
        self.current_figure = None

        # Cargar poses desde el archivo YAML
        self.poses = self.load_poses()
        self.get_logger().info(f"Poses cargadas: {list(self.poses.keys())}")

        # Publisher para comandos de pose
        self.pose_pub = self.create_publisher(
            PoseCommand,
            "/pose_command",
            10,
        )

        # Gripper: control DIRECTO (0° abrir, -80° cerrar) vía follow_joint_trajectory_node
        # Publica Bool en /set_gripper:
        #   True  -> OPEN  (0°)
        #   False -> CLOSE (-80°)
        self.gripper_pub = self.create_publisher(Bool, "/set_gripper", 10)

        # Comandos directos a joints para forzar HOME (todos en 0 rad)
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            "joint_command",
            10,
        )

        # Subscriber para tipo de figura
        self.figure_sub = self.create_subscription(
            String,
            "/figure_type",
            self.figure_callback,
            10,
        )

        # Publicador de estado ocupado para pausar YOLO durante la rutina
        self.busy_pub = self.create_publisher(Bool, "/routine_busy", 10)

        # Timer para ejecutar la secuencia paso a paso
        self.sequence_timer = None

        self.get_logger().info("Nodo clasificador iniciado y listo para recibir comandos")
        self.get_logger().info(f"Mapeo de figuras: {self.figure_to_bin}")

    # ------------------------------------------------------------------
    # Carga de poses
    # ------------------------------------------------------------------
    def load_poses(self) -> dict:
        """Carga las poses desde phantomx_pincher_bringup/config/poses.yaml."""
        try:
            bringup_share = get_package_share_directory("phantomx_pincher_bringup")
            poses_path = os.path.join(bringup_share, "config", "poses.yaml")

            self.get_logger().info(f"Cargando poses desde: {poses_path}")

            with open(poses_path, "r") as f:
                data = yaml.safe_load(f)
                return data.get("poses", {})

        except Exception as e:  # pragma: no cover - ruta de error
            self.get_logger().error(f"Error cargando poses: {e}")
            # Poses mínimas por defecto
            return {
                "home": {
                    "x": 0.100,
                    "y": 0.0,
                    "z": 0.140,
                    "roll": 3.142,
                    "pitch": 0.0,
                    "yaw": 0.0,
                },
                "recoleccion": {
                    "x": 0.100,
                    "y": 0.0,
                    "z": 0.040,
                    "roll": 3.142,
                    "pitch": -0.007,
                    "yaw": 0.0,
                },
            }

    # ------------------------------------------------------------------
    # Publicar poses y gripper
    # ------------------------------------------------------------------
    def publish_pose(self, pose_name: str, cartesian_path: bool = False) -> bool:
        """Publica un comando de pose al tópico /pose_command."""
        if pose_name not in self.poses:
            self.get_logger().error(f'Pose "{pose_name}" no encontrada en configuración')
            return False

        pose = self.poses[pose_name]

        msg = PoseCommand()
        msg.x = float(pose["x"])
        msg.y = float(pose["y"])
        msg.z = float(pose["z"])
        msg.roll = float(pose["roll"])
        msg.pitch = float(pose["pitch"])
        msg.yaw = float(pose["yaw"])
        msg.cartesian_path = cartesian_path

        self.get_logger().info(
            f'📍 Publicando pose "{pose_name}": '
            f"x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, "
            f"roll={msg.roll:.3f}, pitch={msg.pitch:.3f}, yaw={msg.yaw:.3f}, "
            f"cartesian={msg.cartesian_path}"
        )

        self.pose_pub.publish(msg)
        return True

    def control_gripper(self, open_gripper: bool) -> None:
        """Controla el gripper (abrir/cerrar) con /set_gripper (control directo)."""
        action = "🔓 Abriendo" if open_gripper else "🔒 Cerrando"
        self.get_logger().info(f"{action} gripper (direct /set_gripper)...")

        msg = Bool()
        msg.data = bool(open_gripper)
        self.gripper_pub.publish(msg)

    def send_home_joint_command(self) -> None:
        """Envía joint_command [0,0,0,0] para asegurar HOME articular real."""
        msg = Float64MultiArray()
        # commander acepta >=4; enviamos q1..q4 en 0 rad
        msg.data = [0.01745, 0.01745, 0.01745, 0.01745]
        self.get_logger().info(
            "Enviando joint_command [1°,1°,1°,1°] para alinear joints a HOME real."
        )
        self.joint_cmd_pub.publish(msg)

    def set_busy(self, busy: bool) -> None:
        """Publica si la FSM de clasificación está ejecutando una rutina."""
        msg = Bool()
        msg.data = busy
        self.busy_pub.publish(msg)

    # ------------------------------------------------------------------
    # Máquina de estados de la secuencia
    # ------------------------------------------------------------------
    def execute_sequence_step(self) -> None:
        """Ejecuta el siguiente paso de la secuencia (llamado por el timer)."""
        if self.current_state == SequenceState.IDLE:
            return

        # 1. Ir a HOME (paso 1 de tu secuencia)
        if self.current_state == SequenceState.MOVING_TO_HOME_START:
            self.get_logger().info("🏠 [Paso 1/12] Ir a HOME...")
            if self.publish_pose("home", cartesian_path=False):
                # Luego ir a recoleccion_1 (aproximación)
                self.current_state = SequenceState.MOVING_TO_SAFE_POS_1
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error("❌ Error: No se pudo mover a HOME")
                self.abort_sequence()

        # 1.b Ir a RECOLECCION_1 (paso 2 de tu secuencia)
        elif self.current_state == SequenceState.MOVING_TO_SAFE_POS_1:
            self.get_logger().info("📍 [Paso 1b] Ir a RECOLECCION_1 (aproximación)...")
            if self.publish_pose("recoleccion_1", cartesian_path=False):
                self.current_state = SequenceState.OPENING_GRIPPER_START
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error("❌ Error: No se pudo mover a recoleccion_1")
                self.abort_sequence()

        # 1.5 Abrir Gripper (Inicio)
        elif self.current_state == SequenceState.OPENING_GRIPPER_START:
            self.control_gripper(True)
            self.current_state = SequenceState.MOVING_TO_PICKUP
            self.schedule_next_step(self.TIME_GRIPPER)

        # 2. Zona Recolección (paso 3 de tu secuencia: recoleccion_2)
        elif self.current_state == SequenceState.MOVING_TO_PICKUP:
            self.get_logger().info("📦 [Paso 2/12] Ir a RECOLECCIÓN...")
            if self.publish_pose("recoleccion_2", cartesian_path=False):
                self.current_state = SequenceState.CLOSING_GRIPPER
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error("❌ Error: No se pudo mover a RECOLECCIÓN")
                self.abort_sequence()

        # 2.5 Cerrar Gripper
        elif self.current_state == SequenceState.CLOSING_GRIPPER:
            self.control_gripper(False)

            # Retornar desde recoleccion_2 siguiendo la secuencia inversa:
            # recoleccion_2 -> recoleccion_1 -> home
            self.current_state = SequenceState.MOVING_TO_SAFE_POS_2
            self.schedule_next_step(self.TIME_GRIPPER)

        # 3.a recoleccion_2 -> recoleccion_1
        elif self.current_state == SequenceState.MOVING_TO_SAFE_POS_2:
            self.get_logger().info("⬆️  [Paso 3a] Volver a RECOLECCION_1 (desde recoleccion_2)...")
            if self.publish_pose("recoleccion_1", cartesian_path=False):
                # Anteriormente: ir a HOME con objeto y luego a la caneca.
                # Simplificado para seguir la misma lógica de routines.yaml:
                #   home -> recoleccion_1 -> recoleccion_2 -> recoleccion_1 -> caneca -> home
                self.current_state = SequenceState.MOVING_TO_BIN
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error("❌ Error: No se pudo mover a recoleccion_1 (retorno)")
                self.abort_sequence()

        # 7. Ir a Caneca (primero pre_drop, luego drop)
        elif self.current_state == SequenceState.MOVING_TO_BIN:
            pre_drop_pose = self.bin_to_pre_drop.get(self.current_bin)
            if pre_drop_pose and pre_drop_pose in self.poses:
                self.get_logger().info(f"📐 [Paso 6/12] Aproximación sobre {self.current_bin.upper()}...")
                if self.publish_pose(pre_drop_pose, cartesian_path=False):
                    self.current_state = SequenceState.PRE_DROP
                    self.schedule_next_step(self.TIME_MOVEMENT)
                else:
                    self.get_logger().warn("⚠️  Pre-drop falló, intentando ir directo a caneca...")
                    self.current_state = SequenceState.PRE_DROP
                    self.schedule_next_step(0.5)
            else:
                # Si no hay pre_drop definido, ir directo
                self.current_state = SequenceState.PRE_DROP
                self.schedule_next_step(0.1)

        # 7.b Bajar a la caneca (drop)
        elif self.current_state == SequenceState.PRE_DROP:
            self.get_logger().info(f"🎯 [Paso 7/12] Ir a {self.current_bin.upper()}...")
            if self.publish_pose(self.current_bin, cartesian_path=False):
                self.current_state = SequenceState.OPENING_GRIPPER_DROP
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error(f"❌ Error: No se pudo mover a {self.current_bin}")
                self.abort_sequence()

        # 7.5 Abrir Gripper (Soltar)
        elif self.current_state == SequenceState.OPENING_GRIPPER_DROP:
            self.control_gripper(True)
            self.current_state = SequenceState.RETURNING_TO_HOME_END
            self.schedule_next_step(self.TIME_GRIPPER)

        # 12. Ir a HOME (Final)
        elif self.current_state == SequenceState.RETURNING_TO_HOME_END:
            self.get_logger().info("🏠 [Paso final] Ir a HOME (Final)...")
            if self.publish_pose("home", cartesian_path=False):
                self.current_state = SequenceState.COMPLETED
                self.schedule_next_step(self.TIME_MOVEMENT)
            else:
                self.get_logger().error("❌ Error: No se pudo regresar a HOME final")
                self.abort_sequence()

        elif self.current_state == SequenceState.COMPLETED:
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ SECUENCIA COMPLETADA EXITOSAMENTE")
            self.get_logger().info("=" * 60)
            # Forzar HOME articular exacto (todas las joints en 0 rad)
            self.send_home_joint_command()
            # Liberar la rutina para que YOLO pueda volver a detectar nuevas figuras
            self.set_busy(False)
            self.current_state = SequenceState.IDLE
            if self.sequence_timer:
                self.sequence_timer.cancel()
                self.sequence_timer = None

    def schedule_next_step(self, delay_seconds: float) -> None:
        """Programa el siguiente paso de la secuencia (one-shot timer)."""
        if self.sequence_timer:
            self.sequence_timer.cancel()
            self.sequence_timer = None

        self.sequence_timer = self.create_timer(
            delay_seconds,
            self._one_shot_step_callback,
        )

    def _one_shot_step_callback(self) -> None:
        """Wrapper que cancela el timer después de disparar (one-shot behavior)."""
        if self.sequence_timer:
            self.sequence_timer.cancel()
            self.sequence_timer = None
        self.execute_sequence_step()

    def abort_sequence(self) -> None:
        """Aborta la secuencia actual e intenta ir a pose de recovery."""
        self.get_logger().error("❌ Secuencia abortada debido a un error")
        self.current_state = SequenceState.IDLE
        self.set_busy(False)
        if self.sequence_timer:
            self.sequence_timer.cancel()
            self.sequence_timer = None
        # Intentar mover a pose de recovery (segura)
        if "recovery" in self.poses:
            self.get_logger().info("🔄 Intentando mover a pose de RECOVERY...")
            self.publish_pose("recovery", cartesian_path=False)

    # ------------------------------------------------------------------
    # Inicio de secuencia y callback
    # ------------------------------------------------------------------
    def start_sequence(self, figure_type: str) -> None:
        """Inicia una nueva secuencia de pick-and-place."""
        if figure_type not in self.figure_to_bin:
            self.get_logger().error(
                f'Tipo de figura "{figure_type}" no reconocido. '
                f"Tipos válidos: {list(self.figure_to_bin.keys())}"
            )
            return

        if self.current_state != SequenceState.IDLE:
            self.get_logger().warn(
                "⚠️  Ya hay una secuencia en curso. Ignorando comando."
            )
            return

        self.current_bin = self.figure_to_bin[figure_type]
        self.current_figure = figure_type

        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 INICIANDO SECUENCIA DE PICK & PLACE")
        self.get_logger().info(
            f"📋 Figura: {figure_type} → Caneca: {self.current_bin}"
        )
        self.get_logger().info("=" * 60)

        # Marca la rutina como ocupada para pausar la detección YOLO
        self.set_busy(True)

        self.current_state = SequenceState.MOVING_TO_HOME_START
        self.execute_sequence_step()

    def figure_callback(self, msg: String) -> None:
        """Callback para el tópico /figure_type."""
        figure_type = msg.data.lower().strip()
        self.get_logger().info(f'📨 Recibido tipo de figura: "{figure_type}"')
        self.start_sequence(figure_type)


def main(args=None) -> None:
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = ClasificadorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

