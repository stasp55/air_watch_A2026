"""Нода `keepout_publisher` — запрет клеток для планировщика Nav2.

Слушает `blocked_cells` из картины мира и перерисовывает маску фильтра
запретных зон. Дальше работает штатный механизм Nav2: `KeepoutFilter` в
глобальном костмапе поднимает стоимость закрашенных клеток, и планировщик
строит объезд без единой строки специального кода в логике миссии.

Публикует (оба топика — latched, как того требует Nav2):
    ~/keepout_mask   nav_msgs/OccupancyGrid
    ~/costmap_filter_info  nav2_msgs/CostmapFilterInfo

Как включить на ровере — см. td_bringup/config/nav2_keepout.yaml.
"""

from __future__ import annotations

import math

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from td_interfaces.msg import WorldState
from td_interfaces.srv import SetBlockedCells
from td_world.field import load_field
from td_world.grid import CellRef
from td_world.ros_convert import msg_to_cell

from .keepout import build_mask


class KeepoutPublisher(Node):
    def __init__(self) -> None:
        super().__init__("keepout_publisher")

        self.declare_parameter("field_config", "")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("margin_m", 0.1)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("mask_topic", "/keepout_mask")
        self.declare_parameter("filter_info_topic", "/costmap_filter_info")
        self.declare_parameter("follow_world_state", True)

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)
        self._cells: set[CellRef] = set()

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mask_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("mask_topic").value), latched
        )
        self._info_pub = self.create_publisher(
            CostmapFilterInfo, str(self.get_parameter("filter_info_topic").value), latched
        )

        if bool(self.get_parameter("follow_world_state").value):
            self.create_subscription(WorldState, "/td/world_state", self._on_world, 5)
        self.create_service(SetBlockedCells, "~/set_blocked_cells", self._on_service)

        self._publish_filter_info()
        self._publish_mask()
        self.get_logger().info("Маска запретных зон опубликована (пустая)")

    # --- входы -------------------------------------------------------------

    def _on_world(self, msg: WorldState) -> None:
        cells = {msg_to_cell(c, self.field.grid) for c in msg.blocked_cells}
        cells = {c for c in cells if c is not None}
        if cells != self._cells:
            self._cells = cells
            self._publish_mask()
            names = ", ".join(sorted(c.name for c in cells)) or "нет"
            self.get_logger().info(f"Запрещённые клетки: {names}")

    def _on_service(self, request: SetBlockedCells.Request, response):
        cells = {msg_to_cell(c, self.field.grid) for c in request.cells}
        cells = {c for c in cells if c is not None}
        if request.inflate_neighbours:
            cells = set(self.field.grid.expand(cells))
        self._cells = cells
        self._publish_mask()
        response.success = True
        response.message = f"запрещено клеток: {len(cells)}"
        self.get_logger().info(response.message)
        return response

    # --- выходы ------------------------------------------------------------

    def _publish_filter_info(self) -> None:
        info = CostmapFilterInfo()
        info.header.frame_id = str(self.get_parameter("map_frame").value)
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = 0            # 0 = keepout / preferred lanes
        info.filter_mask_topic = str(self.get_parameter("mask_topic").value)
        info.base = 0.0
        info.multiplier = 1.0
        self._info_pub.publish(info)

    def _publish_mask(self) -> None:
        resolution = float(self.get_parameter("resolution").value)
        mask, geometry = build_mask(
            self.field.grid,
            self._cells,
            resolution=resolution,
            margin_m=float(self.get_parameter("margin_m").value),
        )

        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = str(self.get_parameter("map_frame").value)
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = resolution
        grid_msg.info.width = geometry.width
        grid_msg.info.height = geometry.height

        # Начало маски — угол поля, пересчитанный в карту ровера; поворот
        # берётся из той же калибровки, поэтому маска ложится ровно на сетку.
        origin_x, origin_y = self.field.field_to_map.apply(0.0, 0.0)
        yaw = self.field.field_to_map.yaw
        grid_msg.info.origin.position.x = float(origin_x)
        grid_msg.info.origin.position.y = float(origin_y)
        grid_msg.info.origin.orientation.z = math.sin(yaw / 2.0)
        grid_msg.info.origin.orientation.w = math.cos(yaw / 2.0)
        grid_msg.data = mask.flatten().tolist()

        self._mask_pub.publish(grid_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeepoutPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
