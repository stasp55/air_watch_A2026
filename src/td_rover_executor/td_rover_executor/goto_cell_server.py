"""Нода `goto_cell` — навык ровера «приехать в клетку».

Переводит клетку поля в позу на карте и отдаёт её Nav2. Наверх отдаётся
понятная миссии обратная связь: текущая клетка, остаток пути, фаза.

Объезд противника здесь НЕ программируется: запрещённые клетки закрашивает
`keepout_publisher`, и Nav2 планирует обход сам. Эта нода лишь отказывается
ехать в клетку, которая запрещена, и прерывает движение, если запрет появился
уже в пути.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from td_interfaces.action import GoToCell
from td_interfaces.msg import WorldState
from td_world.field import load_field
from td_world.grid import CellRef
from td_world.ros_convert import cell_to_msg, msg_to_cell

from .alignment import wall_yaw


class GoToCellServer(Node):
    def __init__(self) -> None:
        super().__init__("goto_cell")

        self.declare_parameter("field_config", "")
        self.declare_parameter("nav_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("default_timeout_s", 90.0)
        self.declare_parameter("respect_blocked_cells", True)
        self.declare_parameter("wait_when_blocked_s", 8.0)

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        self._group = ReentrantCallbackGroup()
        self._blocked: set[CellRef] = set()
        self._nav_feedback = None

        self.create_subscription(
            WorldState, "/td/world_state", self._on_world, 5, callback_group=self._group
        )
        self._nav = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action").value),
            callback_group=self._group,
        )
        self._server = ActionServer(
            self, GoToCell, "/goto_cell",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._group,
        )
        self.get_logger().info("Навык goto_cell готов")

    def _on_world(self, msg: WorldState) -> None:
        cells = {msg_to_cell(c, self.field.grid) for c in msg.blocked_cells}
        self._blocked = {c for c in cells if c is not None}

    # --- навык -------------------------------------------------------------

    def _execute(self, goal_handle):
        request = goal_handle.request
        result = GoToCell.Result()

        target = msg_to_cell(request.goal, self.field.grid)
        if target is None:
            goal_handle.abort()
            result.message = f"клетка {request.goal.name!r} не разобрана или вне поля"
            self.get_logger().error(result.message)
            return result

        if self._respect_blocked() and target in self._blocked:
            waited = self._wait_until_free(target, goal_handle)
            if not waited:
                goal_handle.abort()
                result.message = f"клетка {target.name} занята противником"
                self.get_logger().warn(result.message)
                return result

        pose = self._cell_to_pose(target, request)
        if not self._nav.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.message = "Nav2 недоступен"
            self.get_logger().error(result.message)
            return result

        self._publish(goal_handle, target, 0.0, "PLANNING")
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        send_future = self._nav.send_goal_async(
            nav_goal, feedback_callback=self._on_nav_feedback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            goal_handle.abort()
            result.message = "Nav2 отклонил цель"
            return result

        result_future = nav_handle.get_result_async()
        timeout = request.timeout or float(self.get_parameter("default_timeout_s").value)
        deadline = time.time() + timeout

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
                goal_handle.canceled()
                result.message = "движение отменено"
                return result
            if self._respect_blocked() and target in self._blocked:
                nav_handle.cancel_goal_async()
                goal_handle.abort()
                result.message = f"клетка {target.name} стала занята противником"
                self.get_logger().warn(result.message)
                return result
            if time.time() > deadline:
                nav_handle.cancel_goal_async()
                goal_handle.abort()
                result.message = f"таймаут {timeout:.0f} с"
                return result
            self._publish(goal_handle, target, self._distance(), "MOVING")
            time.sleep(0.2)

        wrapped = result_future.result()
        succeeded = wrapped is not None and wrapped.status == 4  # STATUS_SUCCEEDED
        result.final_cell = cell_to_msg(target)
        if not succeeded:
            goal_handle.abort()
            result.success = False
            result.message = f"Nav2 не довёл до {target.name} (статус {getattr(wrapped, 'status', '?')})"
            self.get_logger().warn(result.message)
            return result

        goal_handle.succeed()
        result.success = True
        result.message = f"прибыл в {target.name}"
        self.get_logger().info(result.message)
        return result

    # --- вспомогательное ---------------------------------------------------

    def _respect_blocked(self) -> bool:
        return bool(self.get_parameter("respect_blocked_cells").value)

    def _wait_until_free(self, target: CellRef, goal_handle) -> bool:
        """Подождать, пока противник освободит клетку назначения."""
        wait_s = float(self.get_parameter("wait_when_blocked_s").value)
        deadline = time.time() + wait_s
        self.get_logger().info(
            f"Клетка {target.name} занята, жду до {wait_s:.0f} с"
        )
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if target not in self._blocked:
                return True
            self._publish(goal_handle, target, 0.0, "BLOCKED")
            time.sleep(0.3)
        return target not in self._blocked

    def _cell_to_pose(self, cell: CellRef, request) -> PoseStamped:
        x, y = self.field.center_in_map(cell)

        if not math.isnan(request.yaw_deg):
            yaw_field = math.radians(float(request.yaw_deg))
        elif request.face_wall:
            yaw_field = wall_yaw(self.field.grid, cell)
        else:
            yaw_field = 0.0
        yaw_map = self.field.field_to_map.apply_yaw(yaw_field)

        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("map_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw_map / 2.0)
        pose.pose.orientation.w = math.cos(yaw_map / 2.0)
        return pose

    def _on_nav_feedback(self, feedback_msg) -> None:
        self._nav_feedback = feedback_msg.feedback

    def _distance(self) -> float:
        return float(getattr(self._nav_feedback, "distance_remaining", 0.0) or 0.0)

    def _publish(self, goal_handle, cell: CellRef, distance: float, phase: str) -> None:
        feedback = GoToCell.Feedback()
        feedback.current_cell = cell_to_msg(cell)
        feedback.distance_remaining = float(distance)
        feedback.phase = phase
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToCellServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
