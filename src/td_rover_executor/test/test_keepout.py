"""Тесты маски запретных зон."""

import numpy as np
import pytest

from td_rover_executor.keepout import BLOCKED, FREE, build_mask
from td_world.grid import CellRef, Grid


def test_empty_mask_is_all_free():
    mask, geometry = build_mask(Grid(), [], resolution=0.1)
    assert geometry.width == 48 and geometry.height == 48
    assert np.all(mask == FREE)


def test_blocked_cell_is_filled():
    grid = Grid()
    mask, _ = build_mask(grid, [CellRef.parse("F1")], resolution=0.1)
    # F1 — нижний левый угол поля: строки 0..7, столбцы 0..7 при клетке 0.8 м.
    assert np.all(mask[0:8, 0:8] == BLOCKED)
    assert mask[8, 8] == FREE


def test_blocked_area_matches_cell_size():
    grid = Grid(cell_size=0.8)
    mask, geometry = build_mask(grid, [CellRef.parse("C3")], resolution=0.05)
    blocked_area = int(np.count_nonzero(mask == BLOCKED)) * geometry.resolution ** 2
    assert blocked_area == pytest.approx(0.8 * 0.8, rel=1e-6)


def test_margin_expands_the_blocked_area():
    grid = Grid()
    plain, _ = build_mask(grid, [CellRef.parse("C3")], resolution=0.05)
    padded, _ = build_mask(grid, [CellRef.parse("C3")], resolution=0.05, margin_m=0.15)
    assert np.count_nonzero(padded == BLOCKED) > np.count_nonzero(plain == BLOCKED)


def test_several_cells_are_blocked():
    grid = Grid()
    mask, _ = build_mask(
        grid, [CellRef.parse("A1"), CellRef.parse("F6")], resolution=0.1
    )
    assert mask[0, -1] == BLOCKED    # F6 — правый нижний
    assert mask[-1, 0] == BLOCKED    # A1 — левый верхний
    assert mask[24, 24] == FREE


def test_cells_outside_the_field_are_ignored():
    grid = Grid()
    mask, _ = build_mask(grid, [CellRef(9, 9)], resolution=0.1)
    assert np.all(mask == FREE)


def test_invalid_resolution_is_rejected():
    with pytest.raises(ValueError):
        build_mask(Grid(), [], resolution=0.0)
