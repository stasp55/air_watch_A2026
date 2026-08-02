"""Нода `dispatcher_link` — сторона диспетчера (ноутбук).

Поднимает по серверу действия на каждую пару «агент × навык»:

    /td/rover-01/goto_cell   td_interfaces/action/GoToCell
    /td/drone-01/find_object td_interfaces/action/FindObject
    ...

и проксирует их через шину на борт. Для диспетчера это выглядит как обычные
локальные действия — он ничего не знает ни про MQTT, ни про то, что дрон и
ровер работают на разных дистрибутивах ROS.

Обратно в локальный ROS ретранслируются детекции и статусы всех агентов,
поэтому `world_state` собирает картину мира штатным способом.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from td_interfaces.msg import AgentStatus, Detection

from .bus import make_bus
from .protocol import (
    CMD_CANCEL,
    EVT_ACCEPTED,
    EVT_DETECTION,
    EVT_FEEDBACK,
    EVT_REJECTED,
    EVT_RESULT,
    EVT_STATUS,
    DuplicateFilter,
    Envelope,
    ProtocolError,
    cmd_topic,
    evt_topic,
    make_command,
)
from .serdes import dict_to_message, message_to_dict
from .skills import skill_action_type, skill_names


class _PendingGoal:
    """Состояние одной проксируемой цели."""

    def __init__(self, action_type: type) -> None:
        self.action_type = action_type
        self.accepted = threading.Event()
        self.done = threading.Event()
        self.rejected_reason = ""
        self.result_data: dict | None = None
        self.feedback_data: dict | None = None
        self.feedback_event = threading.Event()


class DispatcherLinkNode(Node):
    def __init__(self) -> None:
        super().__init__("dispatcher_link")

        self.declare_parameter("dispatcher_id", "dispatcher")
        self.declare_parameter("agents", ["rover-01", "drone-01"])
        self.declare_parameter("transport", "mqtt")
        self.declare_parameter("mqtt_host", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")
        self.declare_parameter("topic_prefix", "td/v1/agents")
        self.declare_parameter("skills", skill_names())
        self.declare_parameter("goal_timeout_s", 180.0)
        self.declare_parameter("accept_timeout_s", 5.0)

        self.dispatcher_id = str(self.get_parameter("dispatcher_id").value)
        self.prefix = str(self.get_parameter("topic_prefix").value)
        self.agents = [str(a) for a in self.get_parameter("agents").value]
        self._group = ReentrantCallbackGroup()
        self._pending: dict[str, _PendingGoal] = {}
        self._lock = threading.Lock()
        self._dupes = DuplicateFilter()

        self._detections = self.create_publisher(Detection, "/td/detections", 20)
        self._statuses = self.create_publisher(AgentStatus, "/td/agent_status", 20)

        self.bus = make_bus(
            str(self.get_parameter("transport").value),
            host=str(self.get_parameter("mqtt_host").value),
            port=int(self.get_parameter("mqtt_port").value),
            client_id=f"td-{self.dispatcher_id}",
            username=str(self.get_parameter("mqtt_username").value),
            password=str(self.get_parameter("mqtt_password").value),
            logger=self.get_logger(),
        )

        self._servers = []
        for agent in self.agents:
            self.bus.subscribe(evt_topic(self.prefix, agent), self._on_wire)
            for skill in self.get_parameter("skills").value:
                self._servers.append(self._make_server(agent, str(skill)))
        self.bus.connect()

        self.get_logger().info(
            f"dispatcher_link обслуживает агентов: {', '.join(self.agents)}"
        )

    # --- серверы действий --------------------------------------------------

    def _make_server(self, agent: str, skill: str) -> ActionServer:
        action_type = skill_action_type(skill)
        name = f"/td/{agent}/{skill}"
        self.get_logger().info(f"Прокси действия {name}")
        return ActionServer(
            self,
            action_type,
            name,
            execute_callback=lambda handle, a=agent, s=skill: self._execute(handle, a, s),
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    def _execute(self, goal_handle, agent: str, skill: str):
        action_type = skill_action_type(skill)
        envelope = make_command(
            self.dispatcher_id, agent, skill, message_to_dict(goal_handle.request)
        )
        pending = _PendingGoal(action_type)
        with self._lock:
            self._pending[envelope.msg_id] = pending

        self.bus.publish(cmd_topic(self.prefix, agent), envelope.encode())
        self.get_logger().info(f"{agent}: {skill} -> {envelope.msg_id[:8]}")

        accept_timeout = float(self.get_parameter("accept_timeout_s").value)
        if not pending.accepted.wait(timeout=accept_timeout):
            return self._fail(goal_handle, action_type,
                              f"агент {agent} не подтвердил приём за {accept_timeout:.0f} с")
        if pending.rejected_reason:
            return self._fail(goal_handle, action_type, pending.rejected_reason)

        timeout = float(self.get_parameter("goal_timeout_s").value)
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while not pending.done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                self.bus.publish(
                    cmd_topic(self.prefix, agent),
                    make_command(self.dispatcher_id, agent, CMD_CANCEL,
                                 corr_id=envelope.msg_id).encode(),
                )
                goal_handle.canceled()
                return action_type.Result()
            if pending.feedback_event.is_set():
                pending.feedback_event.clear()
                data = pending.feedback_data
                if data:
                    goal_handle.publish_feedback(
                        dict_to_message(action_type.Feedback, data)
                    )
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                return self._fail(goal_handle, action_type,
                                  f"таймаут {timeout:.0f} с без результата от {agent}")

        with self._lock:
            self._pending.pop(envelope.msg_id, None)
        goal_handle.succeed()
        return dict_to_message(action_type.Result, pending.result_data or {})

    def _fail(self, goal_handle, action_type: type, reason: str):
        self.get_logger().error(reason)
        goal_handle.abort()
        result = action_type.Result()
        if hasattr(result, "success"):
            result.success = False
        if hasattr(result, "message"):
            result.message = reason
        return result

    # --- шина -> ROS -------------------------------------------------------

    def _on_wire(self, _topic: str, payload: bytes) -> None:
        try:
            envelope = Envelope.decode(payload)
        except ProtocolError as exc:
            self.get_logger().warn(f"Отброшено сообщение: {exc}")
            return
        if self._dupes.is_duplicate(envelope.msg_id):
            return
        if envelope.type != "evt":
            return

        if envelope.name == EVT_DETECTION:
            self._detections.publish(dict_to_message(Detection, envelope.data))
            return
        if envelope.name == EVT_STATUS:
            self._statuses.publish(dict_to_message(AgentStatus, envelope.data))
            return

        with self._lock:
            pending = self._pending.get(envelope.corr_id)
        if pending is None:
            return

        if envelope.name == EVT_ACCEPTED:
            pending.accepted.set()
        elif envelope.name == EVT_REJECTED:
            pending.rejected_reason = str(envelope.data.get("reason", "отклонено"))
            pending.accepted.set()
            pending.done.set()
        elif envelope.name == EVT_FEEDBACK:
            pending.feedback_data = envelope.data
            pending.feedback_event.set()
        elif envelope.name == EVT_RESULT:
            pending.result_data = envelope.data
            pending.accepted.set()
            pending.done.set()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DispatcherLinkNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.bus.disconnect()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
