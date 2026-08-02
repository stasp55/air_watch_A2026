"""Тесты инструмента калибровки поля."""

import math

import pytest

from td_bringup.calibrate_field import main, parse_args
from td_world.grid import Grid, Transform2D


def test_arguments_are_parsed():
    args = parse_args(["--pair", "F1", "0.1", "0.2", "--pair", "A6", "3.9", "4.2"])
    assert len(args.pair) == 2
    assert args.target == "map"


def test_recovers_a_known_transform(capsys):
    """Синтетические замеры -> инструмент должен вернуть исходное преобразование."""
    grid = Grid()
    truth = Transform2D(tx=1.25, ty=-0.4, yaw=math.radians(12.0))
    pairs = []
    for name in ("F1", "A6", "A1"):
        x, y = grid.center(grid.cell(name))
        mx, my = truth.apply(x, y)
        pairs += ["--pair", name, f"{mx:.6f}", f"{my:.6f}"]

    assert main(pairs) == 0
    output = capsys.readouterr().out
    assert "field_to_map" in output
    assert "x: 1.2500" in output
    assert "y: -0.4000" in output
    assert "12.00 град" in output


def test_reports_large_residual(capsys):
    """Грубо сбитый замер должен явно помечаться как подозрительный."""
    grid = Grid()
    pairs = []
    for name, offset in (("F1", 0.0), ("A6", 0.0), ("A1", 0.5)):
        x, y = grid.center(grid.cell(name))
        pairs += ["--pair", name, f"{x + offset:.3f}", f"{y:.3f}"]

    assert main(pairs) == 0
    captured = capsys.readouterr()
    assert "невязка" in captured.out
    assert "ВНИМАНИЕ" in captured.err


def test_requires_two_pairs(capsys):
    """Одной точки мало: инструмент обязан отказать, а не выдать случайный ответ."""
    assert main(["--pair", "F1", "0.0", "0.0"]) == 2
    assert "минимум две пары" in capsys.readouterr().err


def test_drone_target_changes_the_key(capsys):
    grid = Grid()
    pairs = []
    for name in ("F1", "A6"):
        x, y = grid.center(grid.cell(name))
        pairs += ["--pair", name, f"{x:.3f}", f"{y:.3f}"]
    main(pairs + ["--target", "drone"])
    assert "field_to_drone" in capsys.readouterr().out
