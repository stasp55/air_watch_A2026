"""Протокол обмена между агентами (поверх MQTT).

Дрон работает на ROS 2 Humble, ровер — на ROS 2 Jazzy; межверсионное
взаимодействие по DDS официально не поддерживается, поэтому единственный
общий транспорт — брокер сообщений. Формат конверта совместим с тем, что уже
использует `fleet_text_bridge_ros2` на ровере (`message_id` + `robot_id`),
но расширен полями маршрутизации и корреляции.

Темы MQTT::

    <prefix>/<agent_id>/cmd   команды агенту   (диспетчер -> агент)
    <prefix>/<agent_id>/evt   события агента   (агент -> диспетчер)

Модуль не зависит ни от ROS, ни от MQTT-библиотеки — только словари и JSON.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import time
import uuid
from typing import Any

__all__ = [
    "PROTOCOL_VERSION",
    "Envelope",
    "DuplicateFilter",
    "ProtocolError",
    "cmd_topic",
    "evt_topic",
    "make_command",
    "make_event",
]

PROTOCOL_VERSION = 1

TYPE_CMD = "cmd"
TYPE_EVT = "evt"

# Имена событий, на которые опирается прокси действий.
EVT_ACCEPTED = "accepted"
EVT_REJECTED = "rejected"
EVT_FEEDBACK = "feedback"
EVT_RESULT = "result"
EVT_DETECTION = "detection"
EVT_STATUS = "status"

CMD_CANCEL = "cancel"


class ProtocolError(ValueError):
    """Пришедшее сообщение не соответствует протоколу."""


@dataclass
class Envelope:
    """Конверт одного сообщения."""

    type: str
    name: str
    src: str
    dst: str
    data: dict[str, Any] = field(default_factory=dict)
    msg_id: str = ""
    corr_id: str = ""
    ts: float = 0.0
    v: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        if not self.msg_id:
            self.msg_id = str(uuid.uuid4())
        if not self.ts:
            self.ts = time.time()

    # --- сериализация ------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "v": self.v,
            "msg_id": self.msg_id,
            "corr_id": self.corr_id,
            "ts": round(self.ts, 3),
            "src": self.src,
            "dst": self.dst,
            "type": self.type,
            "name": self.name,
            "data": self.data,
        }

    def encode(self) -> bytes:
        return json.dumps(self.to_dict(), ensure_ascii=False).encode("utf-8")

    @classmethod
    def decode(cls, payload: bytes | str) -> "Envelope":
        """Разобрать конверт, отбраковав всё подозрительное.

        Сеть на площадке общая, а чужие и битые сообщения не должны валить
        ноду — поэтому любая проблема превращается в ProtocolError, который
        вызывающая сторона просто логирует.
        """
        if isinstance(payload, bytes):
            try:
                payload = payload.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise ProtocolError(f"Не UTF-8: {exc}") from exc
        try:
            raw = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ProtocolError(f"Не JSON: {exc}") from exc
        if not isinstance(raw, dict):
            raise ProtocolError("Ожидался JSON-объект")

        version = raw.get("v", PROTOCOL_VERSION)
        if version != PROTOCOL_VERSION:
            raise ProtocolError(f"Версия протокола {version} не поддерживается")

        msg_type = raw.get("type")
        if msg_type not in (TYPE_CMD, TYPE_EVT):
            raise ProtocolError(f"Неизвестный тип сообщения: {msg_type!r}")

        name = raw.get("name")
        if not isinstance(name, str) or not name:
            raise ProtocolError("Пустое поле name")

        data = raw.get("data", {})
        if not isinstance(data, dict):
            raise ProtocolError("Поле data должно быть объектом")

        return cls(
            type=msg_type,
            name=name,
            src=str(raw.get("src", "")),
            dst=str(raw.get("dst", "")),
            data=data,
            msg_id=str(raw.get("msg_id") or uuid.uuid4()),
            corr_id=str(raw.get("corr_id", "")),
            ts=float(raw.get("ts") or time.time()),
            v=version,
        )

    def age(self, now: float | None = None) -> float:
        return max(0.0, (time.time() if now is None else now) - self.ts)


def make_command(src: str, dst: str, name: str, data: dict[str, Any] | None = None,
                 corr_id: str = "") -> Envelope:
    return Envelope(type=TYPE_CMD, name=name, src=src, dst=dst,
                    data=data or {}, corr_id=corr_id)


def make_event(src: str, dst: str, name: str, data: dict[str, Any] | None = None,
               corr_id: str = "") -> Envelope:
    return Envelope(type=TYPE_EVT, name=name, src=src, dst=dst,
                    data=data or {}, corr_id=corr_id)


def cmd_topic(prefix: str, agent_id: str) -> str:
    return f"{prefix.rstrip('/')}/{agent_id}/cmd"


def evt_topic(prefix: str, agent_id: str) -> str:
    return f"{prefix.rstrip('/')}/{agent_id}/evt"


class DuplicateFilter:
    """Отсев повторов при QoS-1 и переподключениях брокера."""

    def __init__(self, capacity: int = 256) -> None:
        self._capacity = max(1, capacity)
        self._seen: dict[str, None] = {}

    def is_duplicate(self, msg_id: str) -> bool:
        if msg_id in self._seen:
            return True
        self._seen[msg_id] = None
        while len(self._seen) > self._capacity:
            self._seen.pop(next(iter(self._seen)))
        return False
