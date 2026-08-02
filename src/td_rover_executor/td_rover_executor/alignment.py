"""Выбор курса ровера в клетке.

Отдельный модуль, потому что «повернуться к стене, чтобы прочитать бумажку» —
это геометрия поля, а не управление приводами, и её надо проверять тестами.
"""

from __future__ import annotations

import math

from td_world.grid import CellRef, Grid

__all__ = ["wall_yaw", "yaw_towards"]


def wall_yaw(grid: Grid, cell: CellRef) -> float:
    """Курс (рад, field-фрейм) на ближайшую внешнюю стену поля.

    Бумажка с финишной клеткой висит на стене рядом с игрушкой, поэтому ровер
    доворачивается к ближайшей границе поля. При равных расстояниях (углы и
    центр поля) стены перебираются в фиксированном порядке запад -> юг ->
    восток -> север, чтобы поведение было воспроизводимым от попытки к попытке.
    """
    x, y = grid.center(cell)
    walls = [
        (math.pi, x),                    # западная стена (x = 0)
        (-math.pi / 2, y),               # южная стена (y = 0)
        (0.0, grid.width - x),           # восточная стена
        (math.pi / 2, grid.height - y),  # северная стена
    ]
    best_yaw, best_distance = walls[0]
    for yaw, distance in walls[1:]:
        if distance < best_distance - 1e-9:
            best_yaw, best_distance = yaw, distance
    return best_yaw


def yaw_towards(grid: Grid, source: CellRef, target: CellRef) -> float:
    """Курс из одной клетки на другую (например, «смотреть на цель»)."""
    sx, sy = grid.center(source)
    tx, ty = grid.center(target)
    if abs(tx - sx) < 1e-9 and abs(ty - sy) < 1e-9:
        return 0.0
    return math.atan2(ty - sy, tx - sx)
