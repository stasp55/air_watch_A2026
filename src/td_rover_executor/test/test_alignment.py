"""Тесты выбора курса в клетке."""

import math

import pytest

from td_rover_executor.alignment import wall_yaw, yaw_towards
from td_world.grid import CellRef, Grid

GRID = Grid()


@pytest.mark.parametrize(
    "cell,expected",
    [
        ("A1", math.pi),          # верхний левый угол -> западная стена
        ("F1", math.pi),          # нижний левый -> тоже западная (правило приоритета)
        ("A3", math.pi / 2),      # верхний ряд -> северная стена
        ("F3", -math.pi / 2),     # нижний ряд -> южная стена
        ("C6", 0.0),              # правый столбец -> восточная стена
    ],
)
def test_wall_yaw(cell, expected):
    assert wall_yaw(GRID, CellRef.parse(cell)) == pytest.approx(expected)


def test_wall_yaw_is_deterministic_in_the_middle():
    """Для симметричных клеток результат не должен «плавать» между попытками."""
    first = wall_yaw(GRID, CellRef.parse("C3"))
    assert all(wall_yaw(GRID, CellRef.parse("C3")) == first for _ in range(5))


def test_yaw_towards_cardinal_directions():
    origin = CellRef.parse("C3")
    assert yaw_towards(GRID, origin, CellRef.parse("C4")) == pytest.approx(0.0)
    assert yaw_towards(GRID, origin, CellRef.parse("B3")) == pytest.approx(math.pi / 2)
    assert yaw_towards(GRID, origin, CellRef.parse("D3")) == pytest.approx(-math.pi / 2)
    assert abs(yaw_towards(GRID, origin, CellRef.parse("C2"))) == pytest.approx(math.pi)


def test_yaw_towards_same_cell_is_zero():
    cell = CellRef.parse("B2")
    assert yaw_towards(GRID, cell, cell) == 0.0
