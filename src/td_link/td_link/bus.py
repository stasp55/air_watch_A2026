"""Транспорт сообщений: MQTT и локальная заглушка.

Ноды системы работают с интерфейсом `Bus`, а не с конкретной библиотекой.
Благодаря этому связку «диспетчер — агент» можно прогонять в тестах и в
симуляторе вообще без брокера (`LoopbackBus`), а на площадке переключать
транспорт одним параметром.
"""

from __future__ import annotations

import threading
from typing import Callable, Protocol

from .mqtt_min import MqttClient

__all__ = ["Bus", "MqttBus", "LoopbackBus", "make_bus"]

Handler = Callable[[str, bytes], None]


class Bus(Protocol):
    """Минимальный контракт транспорта."""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def publish(self, topic: str, payload: bytes) -> None: ...
    def subscribe(self, topic: str, handler: Handler) -> None: ...
    @property
    def connected(self) -> bool: ...


class LoopbackBus:
    """Транспорт в пределах процесса: publish сразу вызывает подписчиков.

    Поддерживает MQTT-подстановки `+` и `#`, чтобы правила подписки в тестах
    совпадали с боевыми.
    """

    def __init__(self) -> None:
        self._subs: list[tuple[str, Handler]] = []
        self._lock = threading.Lock()

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    @property
    def connected(self) -> bool:
        return True

    def subscribe(self, topic: str, handler: Handler) -> None:
        with self._lock:
            self._subs.append((topic, handler))

    def publish(self, topic: str, payload: bytes) -> None:
        with self._lock:
            targets = [h for pattern, h in self._subs if topic_matches(pattern, topic)]
        for handler in targets:
            handler(topic, payload)


def topic_matches(pattern: str, topic: str) -> bool:
    """Сопоставление темы с MQTT-шаблоном (`+` — сегмент, `#` — хвост)."""
    p_parts = pattern.split("/")
    t_parts = topic.split("/")
    for index, part in enumerate(p_parts):
        if part == "#":
            return True
        if index >= len(t_parts):
            return False
        if part != "+" and part != t_parts[index]:
            return False
    return len(p_parts) == len(t_parts)


class MqttBus:
    """MQTT-шина без внешних Python-зависимостей.

    Клиент перенесён из проверенного ``chat_agents``: QoS 1,
    автопереподключение и восстановление подписок работают на stdlib.
    """

    def __init__(
        self,
        host: str,
        port: int = 1883,
        client_id: str = "",
        username: str = "",
        password: str = "",
        keepalive: int = 30,
        qos: int = 1,
        logger=None,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._qos = int(qos)
        self._logger = logger
        self._subs: list[tuple[str, Handler]] = []
        self._connected = threading.Event()

        self._client = MqttClient(
            host=self._host,
            port=self._port,
            client_id=client_id or None,
            keepalive=int(keepalive),
            username=username or None,
            password=password or None,
            on_message=self._on_message,
            on_connect=self._on_connect,
            on_disconnect=self._on_disconnect,
            # MQTT I/O happens in its own thread. rclpy loggers are not safe
            # for mixed severities from that thread; node-level events below
            # still log successful connection/disconnection.
            logger=lambda _text: None,
        )

    # --- жизненный цикл ----------------------------------------------------

    def connect(self) -> None:
        self._client.start()

    def disconnect(self) -> None:
        self._client.stop()

    @property
    def connected(self) -> bool:
        return self._client.connected

    # --- обмен -------------------------------------------------------------

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._subs.append((topic, handler))
        if self.connected:
            self._client.subscribe(topic, qos=self._qos)

    def publish(self, topic: str, payload: bytes) -> None:
        self._client.publish(topic, payload, qos=self._qos)

    # --- callbacks ---------------------------------------------------------

    def _on_connect(self) -> None:
        self._connected.set()
        for topic, _ in self._subs:
            self._client.subscribe(topic, qos=self._qos)
        self._log("info", f"MQTT подключён к {self._host}:{self._port}")

    def _on_disconnect(self) -> None:
        self._connected.clear()
        self._log("warn", "MQTT отключён, идёт переподключение")

    def _on_message(self, topic: str, payload: bytes) -> None:
        for pattern, handler in list(self._subs):
            if topic_matches(pattern, topic):
                try:
                    handler(topic, payload)
                except Exception as exc:  # noqa: BLE001 - одна битая команда не должна ронять ноду
                    self._log("error", f"Ошибка обработки {message.topic}: {exc}")

    def _log(self, level: str, text: str) -> None:
        if self._logger is not None:
            getattr(self._logger, level, self._logger.info)(text)


def make_bus(kind: str, **kwargs) -> Bus:
    """Фабрика транспорта: `mqtt` или `loopback`."""
    kind = (kind or "mqtt").lower()
    if kind == "loopback":
        return LoopbackBus()
    if kind == "mqtt":
        return MqttBus(**kwargs)
    raise ValueError(f"Неизвестный транспорт: {kind}")
