"""Загрузка описания поля (`field.yaml`) — единственного источника правды.

Всё, что зависит от конкретной площадки (размер поля, стартовая клетка,
клетки-кандидаты, привязка к карте ровера, ID меток ArUco), живёт в одном
файле конфигурации. На соревнованиях правится только он.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
from pathlib import Path
from typing import Any

import yaml

from .grid import CellRef, Grid, Transform2D

__all__ = ["FieldModel", "load_field"]


@dataclass
class FieldModel:
    """Разобранное описание поля."""

    grid: Grid
    start_cell: CellRef
    target_candidates: list[CellRef] = dc_field(default_factory=list)
    field_to_map: Transform2D = dc_field(default_factory=Transform2D)
    field_to_drone: Transform2D = dc_field(default_factory=Transform2D)
    enemy_marker_ids: list[int] = dc_field(default_factory=list)
    own_marker_ids: list[int] = dc_field(default_factory=list)
    marker_size_m: float = 0.1
    label_cells: list[CellRef] = dc_field(default_factory=list)
    raw: dict[str, Any] = dc_field(default_factory=dict)

    # --- удобные обёртки над сеткой ----------------------------------------

    def cell(self, text: str) -> CellRef:
        return self.grid.cell(text)

    def center_in_map(self, cell: CellRef) -> tuple[float, float]:
        """Центр клетки в системе координат карты ровера (`map`)."""
        x, y = self.grid.center(cell)
        return self.field_to_map.apply(x, y)

    def cell_from_map(self, x: float, y: float) -> CellRef | None:
        """Клетка по точке в системе координат карты ровера."""
        fx, fy = self.field_to_map.inverse().apply(x, y)
        return self.grid.cell_at(fx, fy)

    def center_in_drone(self, cell: CellRef) -> tuple[float, float]:
        x, y = self.grid.center(cell)
        return self.field_to_drone.apply(x, y)

    def cell_from_drone(self, x: float, y: float) -> CellRef | None:
        fx, fy = self.field_to_drone.inverse().apply(x, y)
        return self.grid.cell_at(fx, fy)


def _parse_transform(data: Any) -> Transform2D:
    if not data:
        return Transform2D()
    if isinstance(data, (list, tuple)):
        values = list(data) + [0.0] * (3 - len(data))
        return Transform2D(float(values[0]), float(values[1]), float(values[2]))
    if isinstance(data, dict):
        return Transform2D(
            tx=float(data.get("x", 0.0)),
            ty=float(data.get("y", 0.0)),
            yaw=float(data.get("yaw", 0.0)),
        )
    raise ValueError(f"Не понимаю описание трансформа: {data!r}")


def _parse_cells(data: Any, grid: Grid, what: str) -> list[CellRef]:
    if not data:
        return []
    if isinstance(data, str):
        data = [data]
    cells = []
    for item in data:
        try:
            cells.append(grid.cell(str(item)))
        except ValueError as exc:
            raise ValueError(f"{what}: {exc}") from exc
    return cells


def load_field(path: str | Path) -> FieldModel:
    """Прочитать `field.yaml` и построить модель поля.

    Ошибки конфигурации выбрасываются немедленно и с внятным текстом: лучше не
    подняться в воздух, чем лететь по неверной сетке.
    """
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    field_cfg = raw.get("field", raw)

    grid = Grid(
        rows=int(field_cfg.get("rows", 6)),
        cols=int(field_cfg.get("cols", 6)),
        cell_size=float(field_cfg.get("cell_size_m", 0.8)),
    )

    start_raw = field_cfg.get("start_cell")
    if not start_raw:
        raise ValueError(f"{path}: не задана обязательная опция field.start_cell")
    start_cell = grid.cell(str(start_raw))

    markers = field_cfg.get("markers", {}) or {}

    return FieldModel(
        grid=grid,
        start_cell=start_cell,
        target_candidates=_parse_cells(
            field_cfg.get("target_candidates"), grid, "target_candidates"
        ),
        field_to_map=_parse_transform(field_cfg.get("field_to_map")),
        field_to_drone=_parse_transform(field_cfg.get("field_to_drone")),
        enemy_marker_ids=[int(v) for v in (markers.get("enemy_ids") or [])],
        own_marker_ids=[int(v) for v in (markers.get("own_ids") or [])],
        marker_size_m=float(markers.get("size_m", 0.1)),
        label_cells=_parse_cells(field_cfg.get("label_cells"), grid, "label_cells"),
        raw=raw,
    )
