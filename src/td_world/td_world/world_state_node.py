"""Нода `world_state` — единая картина мира.

Единственное место в системе, где сходятся все детекции и статусы агентов.
Диспетчер читает только её выход, поэтому «кто первый крикнул» больше не
влияет на поведение миссии.

Подписки:
    /td/detections     td_interfaces/Detection    — от любых нод восприятия
    /td/agent_status   td_interfaces/AgentStatus  — от исполнителей

Публикации:
    /td/world_state    td_interfaces/WorldState

Сервисы:
    ~/resolve_cell     td_interfaces/ResolveCell  — клетка <-> метры
    ~/forget           std_srvs/Trigger           — сбросить закреплённые факты
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from td_interfaces.msg import AgentStatus, Detection, WorldState
from td_interfaces.srv import ResolveCell

from .field import load_field
from .fusion import BeliefStore, DEFAULT_POLICIES, Fact, KindPolicy
from .grid import CellRef
from .ros_convert import cell_to_msg, detection_to_fact, msg_to_cell


class WorldStateNode(Node):
    def __init__(self) -> None:
        super().__init__("world_state")

        self.declare_parameter("field_config", "")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("enemy_ttl_s", 2.5)
        self.declare_parameter("block_enemy_neighbours", False)
        self.declare_parameter("restrict_target_to_candidates", True)

        field_config = self.get_parameter("field_config").value
        if not field_config:
            raise RuntimeError(
                "Параметр field_config обязателен: путь к field.yaml "
                "(см. td_bringup/config/field.yaml)"
            )
        self.field = load_field(field_config)
        self.get_logger().info(
            f"Поле {self.field.grid.rows}x{self.field.grid.cols}, "
            f"клетка {self.field.grid.cell_size} м, старт "
            f"{self.field.start_cell.name}"
        )

        policies = dict(DEFAULT_POLICIES)
        policies["ENEMY"] = KindPolicy(
            min_confidence=policies["ENEMY"].min_confidence,
            ttl_s=float(self.get_parameter("enemy_ttl_s").value),
            latch=False,
        )
        allowed = {}
        if (
            bool(self.get_parameter("restrict_target_to_candidates").value)
            and self.field.target_candidates
        ):
            allowed["TARGET"] = self.field.target_candidates
        self.beliefs = BeliefStore(policies=policies, allowed_cells=allowed)

        self._agents: dict[str, AgentStatus] = {}
        self._rationale: list[str] = []

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(WorldState, "/td/world_state", latched)
        self.create_subscription(Detection, "/td/detections", self._on_detection, 20)
        self.create_subscription(AgentStatus, "/td/agent_status", self._on_status, 20)

        self.create_service(ResolveCell, "~/resolve_cell", self._on_resolve)
        self.create_service(Trigger, "~/forget", self._on_forget)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

    # --- входящие данные ---------------------------------------------------

    def _on_detection(self, msg: Detection) -> None:
        fact = detection_to_fact(msg, self.field.grid)
        if fact.stamp <= 0.0:
            fact = Fact(
                kind=fact.kind,
                cell=fact.cell,
                confidence=fact.confidence,
                source=fact.source,
                stamp=self._now(),
                payload=fact.payload,
            )
        if self.beliefs.update(fact):
            self._note(
                f"принято {fact.kind}="
                f"{fact.cell.name if fact.cell else fact.payload or '?'} "
                f"({fact.confidence:.2f}, {fact.source})"
            )

    def _on_status(self, msg: AgentStatus) -> None:
        self._agents[msg.agent_id] = msg

    # --- сервисы -----------------------------------------------------------

    def _on_resolve(self, request: ResolveCell.Request, response: ResolveCell.Response):
        try:
            if request.use_point:
                cell = self.field.grid.cell_at(request.point.x, request.point.y)
                if cell is None:
                    raise ValueError("Точка вне поля")
            else:
                cell = msg_to_cell(request.cell, self.field.grid)
                if cell is None:
                    raise ValueError("Клетка не разобрана или вне поля")
            x, y = self.field.grid.center(cell)
            response.success = True
            response.cell = cell_to_msg(cell)
            response.point.x, response.point.y = float(x), float(y)
            response.message = ""
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_forget(self, _request, response):
        for kind in ("TARGET", "FINISH_LABEL", "ENEMY"):
            self.beliefs.forget(kind)
        self._note("оператор сбросил закреплённые факты")
        response.success = True
        response.message = "facts cleared"
        return response

    # --- выход -------------------------------------------------------------

    def _publish(self) -> None:
        now = self._now()
        msg = WorldState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "field"

        target = self.beliefs.get("TARGET", now)
        finish = self.beliefs.get("FINISH_LABEL", now)
        enemy = self.beliefs.get("ENEMY", now)

        msg.target_cell = cell_to_msg(target.cell if target else None)
        msg.finish_cell = cell_to_msg(finish.cell if finish else None)
        msg.enemy_cell = cell_to_msg(enemy.cell if enemy else None)
        msg.target_confidence = target.confidence if target else 0.0
        msg.finish_confidence = finish.confidence if finish else 0.0
        msg.enemy_confidence = enemy.confidence if enemy else 0.0
        msg.target_age_s = _age(self.beliefs.age("TARGET", now))
        msg.finish_age_s = _age(self.beliefs.age("FINISH_LABEL", now))
        msg.enemy_age_s = _age(self.beliefs.age("ENEMY", now))

        rover = self._agents.get("rover-01")
        drone = self._agents.get("drone-01")
        msg.rover_cell = cell_to_msg(
            msg_to_cell(rover.cell, self.field.grid) if rover else None
        )
        msg.drone_cell = cell_to_msg(
            msg_to_cell(drone.cell, self.field.grid) if drone else None
        )
        msg.rover_age_s = self._status_age(rover, now)
        msg.drone_age_s = self._status_age(drone, now)

        msg.blocked_cells = [cell_to_msg(c) for c in self._blocked_cells(enemy)]
        msg.phase = ""  # фазу проставляет диспетчер в своей публикации
        msg.rationale = self._collect_rationale()
        self._pub.publish(msg)

    def _blocked_cells(self, enemy: Fact | None) -> list[CellRef]:
        if enemy is None or enemy.cell is None:
            return []
        cells = [enemy.cell]
        if bool(self.get_parameter("block_enemy_neighbours").value):
            cells = self.field.grid.expand(cells)
        return cells

    def _collect_rationale(self) -> list[str]:
        out = self._rationale + self.beliefs.drain_rejections()
        self._rationale.clear()
        return out[-10:]

    def _status_age(self, status: AgentStatus | None, now: float) -> float:
        if status is None:
            return _age(float("inf"))
        stamp = float(status.header.stamp.sec) + float(status.header.stamp.nanosec) * 1e-9
        return _age(max(0.0, now - stamp))

    def _note(self, text: str) -> None:
        self._rationale.append(text)
        self.get_logger().info(text)
        del self._rationale[:-10]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _age(value: float) -> float:
    """Бесконечность не сериализуется дружелюбно — отдаём большое число."""
    return 1e6 if value == float("inf") else float(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
