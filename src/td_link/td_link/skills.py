"""Реестр навыков: имя в протоколе <-> тип ROS-действия.

Добавление нового навыка агенту — это одна строка здесь плюс сервер действия
в соответствующем исполнителе. Ни диспетчер, ни мост менять не нужно.
"""

from __future__ import annotations

from td_interfaces.action import (
    FindObject,
    GoToCell,
    Land,
    ReadLabel,
    Takeoff,
    TrackTarget,
)

__all__ = ["SKILLS", "skill_action_type", "skill_names"]

SKILLS: dict[str, type] = {
    "goto_cell": GoToCell,
    "find_object": FindObject,
    "read_label": ReadLabel,
    "track_target": TrackTarget,
    "takeoff": Takeoff,
    "land": Land,
}


def skill_action_type(name: str) -> type:
    try:
        return SKILLS[name]
    except KeyError as exc:
        raise KeyError(
            f"Неизвестный навык {name!r}; доступны: {', '.join(sorted(SKILLS))}"
        ) from exc


def skill_names() -> list[str]:
    return sorted(SKILLS)
