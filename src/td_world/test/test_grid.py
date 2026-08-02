"""Тесты сетки поля. Гоняются без ROS: `pytest src/td_world/test`."""

import math

import pytest

from td_world.grid import CellRef, Grid, InvalidCellError, Transform2D


def test_cell_parsing_variants():
    assert CellRef.parse("A1") == CellRef(0, 0)
    assert CellRef.parse("a1") == CellRef(0, 0)
    assert CellRef.parse(" b 5 ") == CellRef(1, 4)
    assert CellRef.parse("F-6") == CellRef(5, 5)


def test_cell_name_roundtrip():
    grid = Grid()
    for cell in grid.cells():
        assert CellRef.parse(cell.name) == cell


@pytest.mark.parametrize("text", ["", "3A", "AA1", "A0", "привет", "A"])
def test_cell_parsing_rejects_garbage(text):
    with pytest.raises(InvalidCellError):
        CellRef.parse(text)


def test_field_bounds_are_checked():
    grid = Grid(rows=6, cols=6)
    with pytest.raises(InvalidCellError):
        grid.cell("G1")
    with pytest.raises(InvalidCellError):
        grid.cell("A7")


def test_geometry_matches_convention():
    """A1 — верхний левый угол, F1 — нижний левый, начало у угла F1."""
    grid = Grid(rows=6, cols=6, cell_size=0.8)
    assert grid.center(CellRef.parse("F1")) == pytest.approx((0.4, 0.4))
    assert grid.center(CellRef.parse("A1")) == pytest.approx((0.4, 4.4))
    assert grid.center(CellRef.parse("A6")) == pytest.approx((4.4, 4.4))
    assert grid.width == pytest.approx(4.8)
    assert grid.height == pytest.approx(4.8)


def test_point_to_cell_is_inverse_of_center():
    grid = Grid()
    for cell in grid.cells():
        x, y = grid.center(cell)
        assert grid.cell_at(x, y) == cell


def test_point_outside_field_has_no_cell():
    grid = Grid()
    assert grid.cell_at(-0.1, 1.0) is None
    assert grid.cell_at(1.0, 99.0) is None


def test_neighbours_are_clipped_at_borders():
    grid = Grid()
    corner = grid.neighbours(CellRef.parse("A1"))
    assert sorted(c.name for c in corner) == ["A2", "B1"]
    middle = grid.neighbours(CellRef.parse("C3"))
    assert sorted(c.name for c in middle) == ["B3", "C2", "C4", "D3"]


def test_expand_includes_source_cells_without_duplicates():
    grid = Grid()
    cells = grid.expand([CellRef.parse("C3")])
    names = [c.name for c in cells]
    assert names == sorted(set(names))
    assert "C3" in names and "B3" in names


def test_manhattan_distance():
    assert CellRef.parse("A1").manhattan(CellRef.parse("A1")) == 0
    assert CellRef.parse("A1").manhattan(CellRef.parse("F6")) == 10


def test_transform_apply_and_inverse():
    tf = Transform2D(tx=1.0, ty=-2.0, yaw=math.pi / 3)
    x, y = tf.apply(0.7, 0.3)
    back = tf.inverse().apply(x, y)
    assert back == pytest.approx((0.7, 0.3))


def test_transform_fit_recovers_known_transform():
    truth = Transform2D(tx=2.5, ty=-1.25, yaw=math.radians(37.0))
    src = [(0.4, 0.4), (4.4, 0.4), (0.4, 4.4)]
    dst = [truth.apply(*p) for p in src]
    fitted = Transform2D.fit(src, dst)
    assert fitted.tx == pytest.approx(truth.tx, abs=1e-6)
    assert fitted.ty == pytest.approx(truth.ty, abs=1e-6)
    assert fitted.yaw == pytest.approx(truth.yaw, abs=1e-6)


def test_transform_fit_needs_two_points():
    with pytest.raises(ValueError):
        Transform2D.fit([(0.0, 0.0)], [(1.0, 1.0)])


def test_calibration_end_to_end():
    """Клетка -> field -> map -> обратно в клетку даёт ту же клетку."""
    grid = Grid()
    tf = Transform2D(tx=-0.35, ty=1.1, yaw=math.radians(-12.0))
    for cell in grid.cells():
        fx, fy = grid.center(cell)
        mx, my = tf.apply(fx, fy)
        bx, by = tf.inverse().apply(mx, my)
        assert grid.cell_at(bx, by) == cell
