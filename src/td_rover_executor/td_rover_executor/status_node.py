"""Нода `rover_status` — отчёт ровера о себе.

Переводит позу ровера в клетку поля и публикует `AgentStatus`, из которого
диспетчер понимает, где ровер и жив ли он. Здесь же — сигнальная мигалка
светодиодной лентой, если она включена в конфигурации.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from td_interfaces.msg import AgentStatus
from td_world.field import load_field
from td_world.ros_convert import cell_to_msg


class RoverStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_status")

        self.declare_parameter("field_config", "")
        self.declare_parameter("agent_id", "rover-01")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("battery_topic", "/battery/state")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("pose_timeout_s", 3.0)

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        self._position: tuple[float, float] | None = None
        self._position_stamp = 0.0
        self._battery = -1.0
        self._source = ""

        self.create_subscription(
            PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value),
            self._on_amcl, 5,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 5
        )
        self.create_subscription(
            BatteryState, str(self.get_parameter("battery_topic").value),
            self._on_battery, 5,
        )
        self._pub = self.create_publisher(AgentStatus, "/td/agent_status", 10)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

    # --- входы -------------------------------------------------------------

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._set_position(msg.pose.pose.position.x, msg.pose.pose.position.y, "amcl")

    def _on_odom(self, msg: Odometry) -> None:
        # Одометрия — запасной источник: используется, пока нет локализации.
        if self._source == "amcl" and self._fresh():
            return
        self._set_position(msg.pose.pose.position.x, msg.pose.pose.position.y, "odom")

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = float(msg.percentage) if msg.percentage == msg.percentage else -1.0

    def _set_position(self, x: float, y: float, source: str) -> None:
        self._position = (x, y)
        self._position_stamp = self._now()
        self._source = source

    # --- выход -------------------------------------------------------------

    def _publish(self) -> None:
        msg = AgentStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.agent_id = str(self.get_parameter("agent_id").value)
        msg.battery = float(self._battery)

        cell = None
        if self._position is not None and self._fresh():
            cell = self.field.cell_from_map(*self._position)
        msg.cell = cell_to_msg(cell)
        msg.localized = cell is not None
        msg.state = "IDLE" if cell is not None else "ERROR"
        msg.message = (
            f"позиция по {self._source}"
            if cell is not None
            else "нет свежей позиции или ровер вне поля"
        )
        self._pub.publish(msg)

    def _fresh(self) -> bool:
        timeout = float(self.get_parameter("pose_timeout_s").value)
        return (self._now() - self._position_stamp) < timeout

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
