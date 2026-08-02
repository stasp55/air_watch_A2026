"""Конвертация между чистыми типами td_world и сообщениями td_interfaces.

Вынесено отдельно, чтобы вся арифметика (grid.py, fusion.py) оставалась
независимой от ROS и тестировалась без установленного дистрибутива.
"""

from __future__ import annotations

from td_interfaces.msg import Cell as CellMsg
from td_interfaces.msg import Detection as DetectionMsg

from .fusion import Fact
from .grid import CellRef, Grid

__all__ = ["cell_to_msg", "msg_to_cell", "detection_to_fact", "stamp_to_sec"]

UNKNOWN_INDEX = 255


def cell_to_msg(cell: CellRef | None) -> CellMsg:
    msg = CellMsg()
    if cell is None:
        msg.name = ""
        msg.row = UNKNOWN_INDEX
        msg.col = UNKNOWN_INDEX
        return msg
    msg.name = cell.name
    msg.row = int(cell.row)
    msg.col = int(cell.col)
    return msg


def msg_to_cell(msg: CellMsg | None, grid: Grid | None = None) -> CellRef | None:
    """Разобрать сообщение клетки.

    Приоритет у индексов: имя используется как запасной вариант, если индексы
    не заполнены. Это позволяет внешним инструментам (LLM-агент, отладка)
    присылать только строку "A3".
    """
    if msg is None:
        return None
    if msg.row != UNKNOWN_INDEX and msg.col != UNKNOWN_INDEX:
        cell = CellRef(int(msg.row), int(msg.col))
    elif msg.name:
        cell = CellRef.parse(msg.name)
    else:
        return None
    if grid is not None and not grid.contains(cell):
        return None
    return cell


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def detection_to_fact(msg: DetectionMsg, grid: Grid | None = None) -> Fact:
    return Fact(
        kind=msg.kind,
        cell=msg_to_cell(msg.cell, grid),
        confidence=float(msg.confidence),
        source=msg.source,
        stamp=stamp_to_sec(msg.header.stamp),
        payload=msg.payload,
    )
