"""Нода `drone_skills` — навыки дрона.

Предоставляет:
    действие /takeoff, /land   — взлёт и посадка;
    действие /goto_cell        — выйти на клетку поля и зависнуть;
    подписку hover_target      — следование за подвижной целью;
    публикацию AgentStatus     — где дрон и что он делает.

Взлёт и посадка оформлены действиями, а не сервисами, чтобы диспетчер
управлял ими через тот же мост, что и остальными навыками.

Полёт выполняется через адаптер `DroneApi`, поэтому логику можно отлаживать
на земле (`backend: mock`) без риска поднять машину в воздух.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from td_interfaces.action import GoToCell, Land, Takeoff
from td_interfaces.msg import AgentStatus
from td_world.field import load_field
from td_world.ros_convert import cell_to_msg, msg_to_cell

from .drone_api import create_drone_api


class DroneSkillsNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_skills")

        self.declare_parameter("field_config", "")
        self.declare_parameter("agent_id", "drone-01")
        self.declare_parameter("backend", "sverk")       # sverk | mock
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("cruise_altitude_m", 1.6)
        self.declare_parameter("speed_mps", 0.5)
        self.declare_parameter("hover_target_topic", "/td/drone/hover_target")
        self.declare_parameter("hover_update_period_s", 0.5)
        self.declare_parameter("hover_enabled", True)
        self.declare_parameter("status_rate_hz", 4.0)

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        backend = str(self.get_parameter("backend").value)
        self.drone = create_drone_api(
            backend, **({} if backend == "mock" else {
                "frame_id": str(self.get_parameter("frame_id").value)
            })
        )
        self.get_logger().info(f"Управление дроном: backend={backend}")

        self._group = ReentrantCallbackGroup()
        self._current_action = ""
        self._last_hover = 0.0

        self._status = self.create_publisher(AgentStatus, "/td/agent_status", 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("hover_target_topic").value),
            self._on_hover_target, 5, callback_group=self._group,
        )
        self._servers = [
            ActionServer(
                self, GoToCell, "/goto_cell",
                execute_callback=self._execute_goto,
                goal_callback=lambda _g: GoalResponse.ACCEPT,
                cancel_callback=lambda _g: CancelResponse.ACCEPT,
                callback_group=self._group,
            ),
            ActionServer(
                self, Takeoff, "/takeoff",
                execute_callback=self._execute_takeoff,
                goal_callback=lambda _g: GoalResponse.ACCEPT,
                cancel_callback=lambda _g: CancelResponse.ACCEPT,
                callback_group=self._group,
            ),
            ActionServer(
                self, Land, "/land",
                execute_callback=self._execute_land,
                goal_callback=lambda _g: GoalResponse.ACCEPT,
                cancel_callback=lambda _g: CancelResponse.ACCEPT,
                callback_group=self._group,
            ),
        ]

        rate = max(1.0, float(self.get_parameter("status_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_status)

    # --- взлёт и посадка ---------------------------------------------------

    def _execute_takeoff(self, goal_handle):
        request = goal_handle.request
        altitude = request.altitude or float(self.get_parameter("cruise_altitude_m").value)
        speed = request.speed or float(self.get_parameter("speed_mps").value)

        self._current_action = "takeoff"
        feedback = Takeoff.Feedback()
        feedback.phase = "CLIMBING"
        feedback.current_altitude = self._altitude()
        goal_handle.publish_feedback(feedback)

        ok, message = self.drone.takeoff(altitude, speed)
        self._current_action = ""

        result = Takeoff.Result()
        result.success = bool(ok)
        result.message = message
        result.reached_altitude = self._altitude()
        if ok:
            goal_handle.succeed()
            self.get_logger().info(f"Взлёт: {message}")
        else:
            goal_handle.abort()
            self.get_logger().error(f"Взлёт не удался: {message}")
        return result

    def _execute_land(self, goal_handle):
        timeout = goal_handle.request.timeout or 15.0
        self._current_action = "land"
        ok, message = self.drone.land(timeout=timeout)
        self._current_action = ""

        result = Land.Result()
        result.success = bool(ok)
        result.message = message
        if ok:
            goal_handle.succeed()
            self.get_logger().info(f"Посадка: {message}")
        else:
            goal_handle.abort()
            self.get_logger().error(f"Посадка не удалась: {message}")
        return result

    def _altitude(self) -> float:
        position = self.drone.position()
        return float(position[2]) if position is not None else 0.0

    # --- навык перемещения -------------------------------------------------

    def _execute_goto(self, goal_handle):
        request = goal_handle.request
        result = GoToCell.Result()

        cell = msg_to_cell(request.goal, self.field.grid)
        if cell is None:
            goal_handle.abort()
            result.message = f"клетка {request.goal.name!r} не разобрана или вне поля"
            return result

        x, y = self.field.center_in_drone(cell)
        altitude = float(self.get_parameter("cruise_altitude_m").value)
        yaw = None if math.isnan(request.yaw_deg) else math.radians(request.yaw_deg)

        self._current_action = f"goto_cell {cell.name}"
        self._publish_feedback(goal_handle, cell, "MOVING")
        ok, message = self.drone.go_to(
            x, y, altitude, yaw,
            float(self.get_parameter("speed_mps").value), wait=True,
        )
        self._current_action = ""

        result.final_cell = cell_to_msg(cell)
        result.success = bool(ok)
        result.message = message
        if ok:
            goal_handle.succeed()
            self.get_logger().info(f"Дрон над клеткой {cell.name}")
        else:
            goal_handle.abort()
            self.get_logger().warn(f"Не удалось выйти на {cell.name}: {message}")
        return result

    def _publish_feedback(self, goal_handle, cell, phase: str) -> None:
        feedback = GoToCell.Feedback()
        feedback.current_cell = cell_to_msg(self._current_cell())
        feedback.distance_remaining = 0.0
        feedback.phase = phase
        goal_handle.publish_feedback(feedback)

    # --- сопровождение -----------------------------------------------------

    def _on_hover_target(self, msg: PoseStamped) -> None:
        """Следовать за целью, которую публикует трекер противника."""
        if not bool(self.get_parameter("hover_enabled").value):
            return
        period = float(self.get_parameter("hover_update_period_s").value)
        now = time.time()
        if now - self._last_hover < period:
            return  # не заваливаем полётный контроллер сетпоинтами
        self._last_hover = now

        x, y = self.field.field_to_drone.apply(msg.pose.position.x, msg.pose.position.y)
        altitude = msg.pose.position.z or float(self.get_parameter("cruise_altitude_m").value)
        ok, message = self.drone.go_to(
            x, y, altitude, None,
            float(self.get_parameter("speed_mps").value), wait=False,
        )
        if not ok:
            self.get_logger().warn(f"Сопровождение: {message}", throttle_duration_sec=5.0)
        else:
            self._current_action = "track_target"

    # --- статус ------------------------------------------------------------

    def _current_cell(self):
        position = self.drone.position()
        if position is None:
            return None
        try:
            x, y = float(position[0]), float(position[1])
            if not math.isfinite(x) or not math.isfinite(y):
                return None
            return self.field.cell_from_drone(x, y)
        except (IndexError, TypeError, ValueError, OverflowError):
            # Telemetry is often NaN before the flight controller localizes.
            return None

    def _publish_status(self) -> None:
        cell = self._current_cell()
        msg = AgentStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"
        msg.agent_id = str(self.get_parameter("agent_id").value)
        msg.cell = cell_to_msg(cell)
        msg.localized = cell is not None
        msg.current_action = self._current_action
        msg.state = "BUSY" if self._current_action else "IDLE"
        msg.battery = -1.0
        msg.message = "" if cell is not None else "нет позиции или дрон вне поля"
        self._status.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneSkillsNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.drone.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
