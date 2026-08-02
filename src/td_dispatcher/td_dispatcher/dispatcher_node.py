"""Нода `dispatcher` — агент-диспетчер.

Читает картину мира, спрашивает у конечного автомата миссии, что делать
дальше, и вызывает навыки агентов как обычные ROS-действия. Где физически
находится агент и через какой транспорт до него идут команды, диспетчер не
знает — этим занимается `td_link`.

Публикации:
    /td/mission_state   td_interfaces/WorldState (с фазой и объяснениями)

Сервисы:
    ~/start   std_srvs/Trigger — начать миссию
    ~/abort   std_srvs/Trigger — прервать миссию и остановить агентов
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from td_interfaces.msg import WorldState
from td_link.skills import skill_action_type
from td_world.field import load_field
from td_world.ros_convert import cell_to_msg, msg_to_cell

from .decision_log import DecisionLog
from .mission import Command, MissionConfig, MissionPlanner, Outcome, Phase, Snapshot


class DispatcherNode(Node):
    def __init__(self) -> None:
        super().__init__("dispatcher")

        self.declare_parameter("field_config", "")
        self.declare_parameter("rover_id", "rover-01")
        self.declare_parameter("drone_id", "drone-01")
        self.declare_parameter("action_prefix", "/td")
        self.declare_parameter("tick_period_s", 0.5)
        self.declare_parameter("autostart", False)
        self.declare_parameter("log_path", "")
        # Параметры сценария
        self.declare_parameter("candidates", [])
        self.declare_parameter("scan_retries", 1)
        self.declare_parameter("label_retries", 2)
        self.declare_parameter("move_retries", 2)
        self.declare_parameter("escort_enemy", True)
        self.declare_parameter("land_when_done", True)
        self.declare_parameter("takeoff_first", True)

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        candidates = [str(c) for c in self.get_parameter("candidates").value]
        if not candidates:
            candidates = [c.name for c in self.field.target_candidates]
        self.planner = MissionPlanner(
            MissionConfig(
                candidates=candidates,
                scan_retries=int(self.get_parameter("scan_retries").value),
                label_retries=int(self.get_parameter("label_retries").value),
                move_retries=int(self.get_parameter("move_retries").value),
                escort_enemy=bool(self.get_parameter("escort_enemy").value),
                land_when_done=bool(self.get_parameter("land_when_done").value),
                takeoff_first=bool(self.get_parameter("takeoff_first").value),
            )
        )
        self.log = DecisionLog(str(self.get_parameter("log_path").value) or None)

        self._agents = {
            "rover": str(self.get_parameter("rover_id").value),
            "drone": str(self.get_parameter("drone_id").value),
        }
        self._group = ReentrantCallbackGroup()
        self._clients: dict[str, ActionClient] = {}
        self._handles: dict[str, object] = {}
        self._world: WorldState | None = None
        self._running = bool(self.get_parameter("autostart").value)
        self._last_phase = self.planner.phase

        self.create_subscription(
            WorldState, "/td/world_state", self._on_world, 5, callback_group=self._group
        )
        self._state_pub = self.create_publisher(WorldState, "/td/mission_state", 5)
        self.create_service(Trigger, "~/start", self._on_start, callback_group=self._group)
        self.create_service(Trigger, "~/abort", self._on_abort, callback_group=self._group)

        period = float(self.get_parameter("tick_period_s").value)
        self.create_timer(period, self._tick, callback_group=self._group)

        self.get_logger().info(
            f"Диспетчер готов. Кандидаты: {', '.join(candidates) or 'всё поле'}. "
            f"Автостарт: {'да' if self._running else 'нет'}"
        )

    # --- вход --------------------------------------------------------------

    def _on_world(self, msg: WorldState) -> None:
        self._world = msg

    def _snapshot(self) -> Snapshot:
        world = self._world
        if world is None:
            return Snapshot(now=self._now())
        name = lambda cell: (c.name if (c := msg_to_cell(cell, self.field.grid)) else None)  # noqa: E731
        return Snapshot(
            now=self._now(),
            target=name(world.target_cell),
            finish=name(world.finish_cell),
            enemy=name(world.enemy_cell),
            enemy_age=float(world.enemy_age_s),
            rover=name(world.rover_cell),
            drone=name(world.drone_cell),
        )

    # --- сервисы -----------------------------------------------------------

    def _on_start(self, _request, response):
        self._running = True
        self.log.write("start", message="миссия запущена оператором")
        response.success = True
        response.message = "миссия запущена"
        self.get_logger().info(response.message)
        return response

    def _on_abort(self, _request, response):
        self._running = False
        self.planner.abort("оператор прервал миссию")
        self._cancel_all()
        self.log.write("abort", message="миссия прервана оператором")
        response.success = True
        response.message = "миссия прервана, агенты остановлены"
        self.get_logger().warn(response.message)
        return response

    # --- основной цикл -----------------------------------------------------

    def _tick(self) -> None:
        if self._running:
            for command in self.planner.step(self._snapshot()):
                self._dispatch(command)
        self._publish_state()

        if self.planner.phase is not self._last_phase:
            self.log.write(
                "phase", phase=self.planner.phase.value,
                reason=self.planner.rationale[-1] if self.planner.rationale else "",
            )
            if self.planner.phase in (Phase.DONE, Phase.FAILED):
                # Фоновое сопровождение больше не нужно — освобождаем дрон,
                # иначе он продолжит висеть над противником после финиша.
                self._cancel("drone")
                if self.planner.phase is Phase.FAILED:
                    self._running = False
                    self.get_logger().error(
                        f"Миссия провалена: {self.planner.failure_reason}"
                    )
                else:
                    self.get_logger().info("Миссия выполнена")
            self._last_phase = self.planner.phase

    def _dispatch(self, command: Command) -> None:
        agent_id = self._agents[command.agent]
        client = self._client(agent_id, command.skill)
        if not client.wait_for_server(timeout_sec=3.0):
            message = f"навык {command.skill} агента {agent_id} недоступен"
            self.get_logger().error(message)
            self.planner.on_outcome(command.agent, Outcome(command.skill, False, message))
            return

        goal = self._build_goal(command)
        self.log.write("command", agent=agent_id, skill=command.skill,
                       args=command.args, reason=command.reason)
        self.get_logger().info(f"{agent_id}: {command.skill} — {command.reason}")

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(command, fut))

    def _build_goal(self, command: Command):
        """Собрать цель действия из аргументов команды планировщика."""
        action_type = skill_action_type(command.skill)
        goal = action_type.Goal()
        args = command.args

        if command.skill == "goto_cell":
            goal.goal = cell_to_msg(self.field.cell(args["cell"]))
            goal.face_wall = bool(args.get("face_wall", False))
            goal.yaw_deg = float("nan")
        elif command.skill == "find_object":
            goal.kind = str(args.get("kind", "TARGET"))
            goal.candidates = [
                cell_to_msg(self.field.cell(name)) for name in args.get("candidates", [])
            ]
        elif command.skill == "track_target":
            goal.kind = str(args.get("kind", "ENEMY"))
        elif command.skill == "read_label":
            goal.samples = 0  # значение по умолчанию берётся из ноды чтения
        return goal

    def _on_goal_response(self, command: Command, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.planner.on_outcome(command.agent, Outcome(command.skill, False, str(exc)))
            return
        if not handle.accepted:
            self.planner.on_outcome(
                command.agent, Outcome(command.skill, False, "цель отклонена")
            )
            return
        self._handles[command.agent] = handle
        handle.get_result_async().add_done_callback(
            lambda fut: self._on_result(command, fut)
        )

    def _on_result(self, command: Command, future) -> None:
        self._handles.pop(command.agent, None)
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(getattr(result, "success", False))
            message = str(getattr(result, "message", ""))
        except Exception as exc:  # noqa: BLE001
            success, message = False, str(exc)

        self.log.write("outcome", agent=self._agents[command.agent],
                       skill=command.skill, success=success, message=message)
        self.get_logger().info(
            f"{self._agents[command.agent]}: {command.skill} — "
            f"{'успех' if success else 'неудача'} ({message})"
        )
        self.planner.on_outcome(command.agent, Outcome(command.skill, success, message))

    # --- вспомогательное ---------------------------------------------------

    def _client(self, agent_id: str, skill: str) -> ActionClient:
        name = f"{self.get_parameter('action_prefix').value}/{agent_id}/{skill}"
        if name not in self._clients:
            self._clients[name] = ActionClient(
                self, skill_action_type(skill), name, callback_group=self._group
            )
        return self._clients[name]

    def _cancel(self, agent: str) -> None:
        handle = self._handles.pop(agent, None)
        if handle is not None:
            handle.cancel_goal_async()
            self.get_logger().info(f"Отменено текущее поручение агента {agent}")

    def _cancel_all(self) -> None:
        for agent in list(self._handles):
            self._cancel(agent)

    def _publish_state(self) -> None:
        world = self._world
        msg = WorldState() if world is None else world
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.phase = self.planner.phase.value
        msg.rationale = self.planner.rationale[-10:]
        self._state_pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatcherNode()
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
