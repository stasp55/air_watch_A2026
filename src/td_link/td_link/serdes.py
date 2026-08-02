"""Обобщённая конвертация ROS-сообщений в словари и обратно.

Используется штатный `rosidl_runtime_py`, поэтому мост не знает про
конкретные поля навыков: добавление поля в .action не требует правок здесь.
"""

from __future__ import annotations

from typing import Any

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

__all__ = ["message_to_dict", "dict_to_message"]


def message_to_dict(message: Any) -> dict:
    """ROS-сообщение -> обычный словарь, готовый к JSON."""
    return dict(message_to_ordereddict(message))


def dict_to_message(message_type: type, data: dict | None) -> Any:
    """Словарь -> ROS-сообщение указанного типа.

    Неизвестные поля игнорируются осознанно: агент с более старой версией
    контрактов должен пережить команду от более новой, а не упасть.
    """
    message = message_type()
    if not data:
        return message
    known = set(getattr(message, "get_fields_and_field_types", dict)().keys())
    filtered = {k: v for k, v in data.items() if k in known}
    set_message_fields(message, filtered)
    return message
