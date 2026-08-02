#!/usr/bin/env python3
"""Калибровка привязки поля к карте робота.

Процедура на площадке (5 минут, делается один раз после построения карты):

1. Поставить ровер точно в центр известной клетки, например F1.
2. Считать его координаты на карте::

       ros2 topic echo /amcl_pose --once

3. Повторить для второй, максимально удалённой клетки (например A6).
4. Скормить пары этому инструменту::

       ros2 run td_bringup calibrate_field --pair F1 0.12 0.34 --pair A6 3.90 4.21

5. Вставить полученный блок в `field.yaml` (`field_to_map` либо
   `field_to_drone` при ключе --target drone).

Точек можно давать больше двух — тогда решение усредняется по методу
наименьших квадратов, что заметно снижает влияние ошибки установки робота.
"""

from __future__ import annotations

import argparse
import math
import sys

from td_world.field import load_field
from td_world.grid import Grid, Transform2D


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Подобрать преобразование field -> карта робота по замерам",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--pair", nargs=3, action="append", metavar=("CELL", "X", "Y"), required=True,
        help="клетка и её измеренные координаты в системе робота (метры)",
    )
    parser.add_argument("--field-config", default="",
                        help="field.yaml, чтобы взять из него размеры поля")
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--cols", type=int, default=6)
    parser.add_argument("--cell-size", type=float, default=0.8)
    parser.add_argument("--target", choices=["map", "drone"], default="map",
                        help="какую строку field.yaml вы собираетесь заполнить")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if args.field_config:
        grid = load_field(args.field_config).grid
    else:
        grid = Grid(rows=args.rows, cols=args.cols, cell_size=args.cell_size)

    source, target = [], []
    for cell_name, x, y in args.pair:
        cell = grid.cell(cell_name)
        source.append(grid.center(cell))
        target.append((float(x), float(y)))

    if len(source) < 2:
        print("Нужно минимум две пары точек (--pair)", file=sys.stderr)
        return 2

    transform = Transform2D.fit(source, target)

    # Невязка показывает качество замеров: больше 3-5 см — стоит перемерить.
    residuals = []
    for (sx, sy), (tx, ty) in zip(source, target):
        px, py = transform.apply(sx, sy)
        residuals.append(math.hypot(px - tx, py - ty))
    worst = max(residuals)

    key = "field_to_map" if args.target == "map" else "field_to_drone"
    print("# вставьте в field.yaml, секция field:")
    print(f"  {key}: {{x: {transform.tx:.4f}, y: {transform.ty:.4f}, "
          f"yaw: {transform.yaw:.5f}}}   # yaw = {math.degrees(transform.yaw):.2f} град")
    print()
    print(f"# невязка: средняя {sum(residuals) / len(residuals) * 100:.1f} см, "
          f"худшая {worst * 100:.1f} см")
    if worst > 0.05:
        print("# ВНИМАНИЕ: невязка больше 5 см — проверьте установку робота "
              "и повторите замеры", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
