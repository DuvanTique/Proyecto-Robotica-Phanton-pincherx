#!/usr/bin/env python3
"""
Nodo que agrega objetos de colisión a la planning scene de MoveIt.

Publica en /planning_scene las geometrías de:
  - Mesa de trabajo (plano)
  - Bandeja de recolección
  - Canecas (4)
  - Soporte de cámara

Esto permite que MoveIt valide colisiones y evite trayectorias que choquen
con el entorno real.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
import time


class SceneObjectsNode(Node):
    def __init__(self):
        super().__init__("scene_objects_node")

        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        # Esperar a que MoveIt esté listo
        self.get_logger().info("Esperando 5s para que MoveIt inicialice...")
        self.create_timer(5.0, self._publish_scene_once)

    def _publish_scene_once(self):
        """Publica todos los objetos de colisión una vez."""
        scene = PlanningScene()
        scene.is_diff = True

        # Frame de referencia
        frame_id = "phantomx_pincher_base_link"

        # --- Mesa de trabajo (caja grande debajo del robot) ---
        mesa = CollisionObject()
        mesa.header.frame_id = frame_id
        mesa.id = "mesa"
        mesa.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.60, 0.60, 0.02]  # 60x60cm, 2cm de grosor
        pose_mesa = Pose()
        pose_mesa.position = Point(x=0.0, y=0.0, z=0.0)  # justo debajo de la base
        pose_mesa.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        mesa.primitives.append(box)
        mesa.primitive_poses.append(pose_mesa)
        scene.world.collision_objects.append(mesa)

        # --- Bandeja de recolección ---
        bandeja = CollisionObject()
        bandeja.header.frame_id = frame_id
        bandeja.id = "bandeja_recoleccion"
        bandeja.operation = CollisionObject.ADD
        box_bandeja = SolidPrimitive()
        box_bandeja.type = SolidPrimitive.CYLINDER
        box_bandeja.dimensions = [0.005, 0.08]  # 15cm radio, 1cm de alto
        pose_bandeja = Pose()
        pose_bandeja.position = Point(x=0.10, y=0.0, z=0.02)
        pose_bandeja.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bandeja.primitives.append(box_bandeja)
        bandeja.primitive_poses.append(pose_bandeja)
        scene.world.collision_objects.append(bandeja)

        # --- Canecas (cajas con orientación) ---
        import math
        # Formato: (nombre, x, y, z, rotacion_z_radianes)
        canecas = [
            ("caneca_roja", -0.009, 0.120, 0.04, 0.0),
            ("caneca_verde", 0.196, 0.091, 0.04, 0.433+1.571),    # rotada ~25° en Z
            ("caneca_azul", 0.192, -0.088, 0.04, -0.433-1.571),   # rotada ~-25° en Z
            ("caneca_amarilla", -0.010, -0.120, 0.04, 0.0),
        ]
        for name, cx, cy, cz, yaw in canecas:
            obj = CollisionObject()
            obj.header.frame_id = frame_id
            obj.id = name
            obj.operation = CollisionObject.ADD
            cyl = SolidPrimitive()
            cyl.type = SolidPrimitive.BOX
            cyl.dimensions = [0.18, 0.12, 0.06]  # 18x12cm, 8cm de alto
            pose_c = Pose()
            pose_c.position = Point(x=cx, y=cy, z=cz)
            pose_c.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            )
            obj.primitives.append(cyl)
            obj.primitive_poses.append(pose_c)
            scene.world.collision_objects.append(obj)

        # --- Soporte de cámara (cilindro vertical) ---
        camara = CollisionObject()
        camara.header.frame_id = frame_id
        camara.id = "soporte_camara"
        camara.operation = CollisionObject.ADD
        cyl_cam = SolidPrimitive()
        cyl_cam.type = SolidPrimitive.CYLINDER
        cyl_cam.dimensions = [0.60, 0.03]  # 30cm alto, 2cm radio
        pose_cam = Pose()
        pose_cam.position = Point(x=0.26, y=0.0, z=0.30)
        pose_cam.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        camara.primitives.append(cyl_cam)
        camara.primitive_poses.append(pose_cam)
        scene.world.collision_objects.append(camara)

        # Publicar
        self.scene_pub.publish(scene)
        self.get_logger().info(
            f"✅ Objetos de colisión publicados: "
            f"{[o.id for o in scene.world.collision_objects]}"
        )

        # Solo publicar una vez, luego destruir el timer
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SceneObjectsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
