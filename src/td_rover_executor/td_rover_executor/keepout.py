"""Построение маски запретных зон для Nav2 Keepout Filter.

Так решается требование «нельзя находиться в одной клетке с противником»:
вместо специальной логики объезда мы просто закрашиваем клетку в маске, а
штатный планировщик Nav2 сам строит обход. Логика миссии при этом не знает
ничего о костмапах, а поведение остаётся предсказуемым.

Функция чистая (numpy), поэтому маска проверяется тестами без Nav2.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np

from td_world.grid import CellRef, Grid

__all__ = ["build_mask", "MaskGeometry"]

FREE = 0
BLOCKED = 100


class MaskGeometry:
    """Размеры и разрешение маски в field-фрейме."""

    def __init__(self, width: int, height: int, resolution: float) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution

    def __repr__(self) -> str:  # pragma: no cover - для логов
        return f"MaskGeometry({self.width}x{self.height} @ {self.resolution} м)"


def build_mask(
    grid: Grid,
    blocked: Iterable[CellRef],
    resolution: float = 0.05,
    margin_m: float = 0.0,
) -> tuple[np.ndarray, MaskGeometry]:
    """Собрать маску: 0 — свободно, 100 — запрещено.

    Возвращается массив в порядке строк OccupancyGrid (снизу вверх по y),
    готовый к отправке как `data` после `flatten()`.

    `margin_m` расширяет каждую запрещённую клетку — запас на задержку данных
    о положении противника и на габарит ровера.
    """
    if resolution <= 0.0:
        raise ValueError("Разрешение маски должно быть положительным")

    width = int(round(grid.width / resolution))
    height = int(round(grid.height / resolution))
    mask = np.full((height, width), FREE, dtype=np.int8)

    for cell in blocked:
        if not grid.contains(cell):
            continue
        x_min, y_min, x_max, y_max = grid.corners(cell)
        x_min, y_min = x_min - margin_m, y_min - margin_m
        x_max, y_max = x_max + margin_m, y_max + margin_m

        col_start = max(0, int(np.floor(x_min / resolution)))
        col_end = min(width, int(np.ceil(x_max / resolution)))
        row_start = max(0, int(np.floor(y_min / resolution)))
        row_end = min(height, int(np.ceil(y_max / resolution)))
        if col_end <= col_start or row_end <= row_start:
            continue
        mask[row_start:row_end, col_start:col_end] = BLOCKED

    return mask, MaskGeometry(width, height, resolution)
