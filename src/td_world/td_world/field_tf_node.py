"""Нода `field_tf` — публикует привязку поля к рабочим фреймам роботов.

Без этой ноды «клетка A3» ничем не отличается от произвольной точки: именно
здесь field-фрейм связывается с картой ровера (`map`) и с локальным фреймом
дрона. Числа берутся из `field.yaml` и получаются процедурой калибровки
(td_bringup/tools/calibrate_field.py).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from .field import load_field
from .grid import Transform2D


def _to_transform(parent: str, child: str, tf2d: Transform2D, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(tf2d.tx)
    msg.transform.translation.y = float(tf2d.ty)
    msg.transform.translation.z = 0.0
    msg.transform.rotation.z = math.sin(tf2d.yaw / 2.0)
    msg.transform.rotation.w = math.cos(tf2d.yaw / 2.0)
    return msg


class FieldTfNode(Node):
    def __init__(self) -> None:
        super().__init__("field_tf")
        self.declare_parameter("field_config", "")
        self.declare_parameter("publish_map", True)
        self.declare_parameter("publish_drone", False)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("drone_frame", "drone_local")
        self.declare_parameter("field_frame", "field")

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        field = load_field(config)

        self._broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        field_frame = self.get_parameter("field_frame").value

        transforms = []
        if bool(self.get_parameter("publish_map").value):
            transforms.append(
                _to_transform(
                    self.get_parameter("map_frame").value,
                    field_frame,
                    field.field_to_map,
                    stamp,
                )
            )
        if bool(self.get_parameter("publish_drone").value):
            transforms.append(
                _to_transform(
                    self.get_parameter("drone_frame").value,
                    field_frame,
                    field.field_to_drone,
                    stamp,
                )
            )

        if not transforms:
            self.get_logger().warn("Нечего публиковать: обе привязки выключены")
        else:
            self._broadcaster.sendTransform(transforms)
            for tf in transforms:
                self.get_logger().info(
                    f"{tf.header.frame_id} -> {tf.child_frame_id}: "
                    f"x={tf.transform.translation.x:.3f} "
                    f"y={tf.transform.translation.y:.3f}"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
