#!/usr/bin/env python3
"""Имитатор ровера противника.

Публикует детекции `ENEMY`, будто дрон видит чужую машину, и гоняет её по
заданному маршруту. Нужен, чтобы проверить на земле самое неудобное:
объезд занятой клетки, ожидание освобождения и реакцию диспетчера — не
дожидаясь соперника на площадке.

    ros2 run td_bringup ghost_enemy --ros-args \
        -p route:="['C1','C2','C3','C4']" -p period_s:=3.0
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from td_interfaces.msg import Detection
from td_world.grid import CellRef, Grid
from td_world.ros_convert import cell_to_msg


class GhostEnemy(Node):
    def __init__(self) -> None:
        super().__init__("ghost_enemy")
        self.declare_parameter("route", ["C1", "C2", "C3", "C4", "C3", "C2"])
        self.declare_parameter("period_s", 3.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("confidence", 0.9)
        self.declare_parameter("rows", 6)
        self.declare_parameter("cols", 6)
        self.declare_parameter("cell_size_m", 0.8)

        self.grid = Grid(
            rows=int(self.get_parameter("rows").value),
            cols=int(self.get_parameter("cols").value),
            cell_size=float(self.get_parameter("cell_size_m").value),
        )
        self.route: list[CellRef] = [
            self.grid.cell(str(name)) for name in self.get_parameter("route").value
        ]
        if not self.route:
            raise RuntimeError("Маршрут пуст")

        self._index = 0
        self._pub = self.create_publisher(Detection, "/td/detections", 10)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(float(self.get_parameter("period_s").value), self._advance)

        self.get_logger().warn(
            "Имитатор противника запущен — на реальной попытке он должен быть выключен! "
            f"Маршрут: {', '.join(c.name for c in self.route)}"
        )

    def _advance(self) -> None:
        self._index = (self._index + 1) % len(self.route)
        self.get_logger().info(f"Противник в клетке {self.route[self._index].name}")

    def _publish(self) -> None:
        cell = self.route[self._index]
        x, y = self.grid.center(cell)

        msg = Detection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.kind = Detection.KIND_ENEMY
        msg.cell = cell_to_msg(cell)
        msg.confidence = float(self.get_parameter("confidence").value)
        msg.source = "ghost/simulator"
        msg.has_pose = True
        msg.pose.header = msg.header
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GhostEnemy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
