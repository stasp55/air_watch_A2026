"""Нода `agent_link` — сторона робота.

Ставится и на ровер, и на дрон. Превращает команды из шины в вызовы локальных
ROS-действий исполнителя и гонит обратно фидбек, результаты, детекции и
статус. Сам ничего не решает: вся логика миссии живёт в диспетчере.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.action import ActionClient
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
    make_event,
)
from .serdes import dict_to_message, message_to_dict
from .skills import skill_action_type, skill_names


class AgentLinkNode(Node):
    def __init__(self) -> None:
        super().__init__("agent_link")

        self.declare_parameter("agent_id", "rover-01")
        self.declare_parameter("dispatcher_id", "dispatcher")
        self.declare_parameter("transport", "mqtt")  # mqtt | loopback
        self.declare_parameter("mqtt_host", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")
        self.declare_parameter("topic_prefix", "td/v1/agents")
        self.declare_parameter("skills", skill_names())
        self.declare_parameter("action_prefix", "")
        self.declare_parameter("forward_detections", True)
        self.declare_parameter("forward_status", True)

        self.agent_id = str(self.get_parameter("agent_id").value)
        self.dispatcher_id = str(self.get_parameter("dispatcher_id").value)
        self.prefix = str(self.get_parameter("topic_prefix").value)
        self._group = ReentrantCallbackGroup()
        self._dupes = DuplicateFilter()
        self._goals: dict[str, object] = {}  # corr_id -> goal handle
        self._lock = threading.Lock()

        action_prefix = str(self.get_parameter("action_prefix").value)
        # Node already owns ``_clients`` internally. Do not shadow it with
        # action clients: Humble executor expects that field to be a list.
        self._action_clients: dict[str, ActionClient] = {}
        for skill in self.get_parameter("skills").value:
            action_type = skill_action_type(skill)
            name = f"{action_prefix}/{skill}" if action_prefix else f"/{skill}"
            self._action_clients[skill] = ActionClient(
                self, action_type, name, callback_group=self._group
            )
            self.get_logger().info(f"Навык {skill} -> действие {name}")

        self.bus = make_bus(
            str(self.get_parameter("transport").value),
            host=str(self.get_parameter("mqtt_host").value),
            port=int(self.get_parameter("mqtt_port").value),
            client_id=f"td-{self.agent_id}",
            username=str(self.get_parameter("mqtt_username").value),
            password=str(self.get_parameter("mqtt_password").value),
            logger=self.get_logger(),
        )
        self.bus.subscribe(cmd_topic(self.prefix, self.agent_id), self._on_wire)
        self.bus.connect()

        if bool(self.get_parameter("forward_detections").value):
            self.create_subscription(
                Detection, "/td/detections", self._on_detection, 20,
                callback_group=self._group,
            )
        if bool(self.get_parameter("forward_status").value):
            self.create_subscription(
                AgentStatus, "/td/agent_status", self._on_status, 10,
                callback_group=self._group,
            )

        self.get_logger().info(
            f"agent_link запущен как {self.agent_id}, тема "
            f"{cmd_topic(self.prefix, self.agent_id)}"
        )

    # --- шина -> ROS -------------------------------------------------------

    def _on_wire(self, _topic: str, payload: bytes) -> None:
        try:
            envelope = Envelope.decode(payload)
        except ProtocolError as exc:
            self.get_logger().warn(f"Отброшено сообщение: {exc}")
            return
        if self._dupes.is_duplicate(envelope.msg_id):
            return
        if envelope.type != "cmd":
            return

        if envelope.name == CMD_CANCEL:
            self._cancel(envelope.corr_id)
            return

        try:
            action_type = skill_action_type(envelope.name)
        except KeyError as exc:
            self._emit(EVT_REJECTED, {"reason": str(exc)}, envelope.msg_id)
            return

        client = self._action_clients.get(envelope.name)
        if client is None or not client.wait_for_server(timeout_sec=2.0):
            self._emit(
                EVT_REJECTED,
                {"reason": f"исполнитель навыка {envelope.name} недоступен"},
                envelope.msg_id,
            )
            return

        try:
            goal = dict_to_message(action_type.Goal, envelope.data)
        except Exception as exc:  # noqa: BLE001 - кривые аргументы не должны ронять мост
            self._emit(EVT_REJECTED, {"reason": f"плохие аргументы: {exc}"}, envelope.msg_id)
            return

        corr = envelope.msg_id
        future = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(corr, fb)
        )
        future.add_done_callback(lambda fut: self._on_goal_response(corr, fut))

    def _on_goal_response(self, corr: str, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._emit(EVT_REJECTED, {"reason": str(exc)}, corr)
            return
        if not handle.accepted:
            self._emit(EVT_REJECTED, {"reason": "цель отклонена исполнителем"}, corr)
            return
        with self._lock:
            self._goals[corr] = handle
        self._emit(EVT_ACCEPTED, {}, corr)
        handle.get_result_async().add_done_callback(
            lambda fut: self._on_result(corr, fut)
        )

    def _on_feedback(self, corr: str, feedback_msg) -> None:
        self._emit(EVT_FEEDBACK, message_to_dict(feedback_msg.feedback), corr)

    def _on_result(self, corr: str, future) -> None:
        with self._lock:
            self._goals.pop(corr, None)
        try:
            wrapped = future.result()
            data = message_to_dict(wrapped.result)
            data["status_code"] = int(wrapped.status)
        except Exception as exc:  # noqa: BLE001
            data = {"success": False, "message": f"ошибка исполнения: {exc}"}
        self._emit(EVT_RESULT, data, corr)

    def _cancel(self, corr: str) -> None:
        with self._lock:
            handle = self._goals.get(corr)
        if handle is None:
            self.get_logger().warn(f"Нечего отменять для {corr}")
            return
        handle.cancel_goal_async()
        self.get_logger().info(f"Отмена цели {corr}")

    # --- ROS -> шина -------------------------------------------------------

    def _on_detection(self, msg: Detection) -> None:
        self._emit(EVT_DETECTION, message_to_dict(msg))

    def _on_status(self, msg: AgentStatus) -> None:
        self._emit(EVT_STATUS, message_to_dict(msg))

    def _emit(self, name: str, data: dict, corr: str = "") -> None:
        envelope = make_event(self.agent_id, self.dispatcher_id, name, data, corr_id=corr)
        self.bus.publish(evt_topic(self.prefix, self.agent_id), envelope.encode())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AgentLinkNode()
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
