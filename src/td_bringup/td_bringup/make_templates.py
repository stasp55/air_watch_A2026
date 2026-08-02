#!/usr/bin/env python3
"""Генерация набора шаблонов букв и цифр для чтения метки финиша.

Нужен как заглушка, пока организаторы не выдали свои картинки: позволяет
отладить весь путь «камера -> клетка финиша» заранее. Когда картинки придут,
просто положите их в тот же каталог под именами `A.png` ... `F.png`,
`1.png` ... `6.png` — код чтения не меняется.

    ros2 run td_bringup make_templates --output ~/td_templates
    ros2 run td_bringup make_templates --output ~/td_templates --rows 6 --cols 6
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX


def render(symbol: str, size: int = 96, thickness: int = 5) -> np.ndarray:
    """Отрисовать символ по центру белого квадрата."""
    canvas = np.full((size, size), 255, np.uint8)
    scale = 2.4
    (width, height), _ = cv2.getTextSize(symbol, FONT, scale, thickness)
    origin = ((size - width) // 2, (size + height) // 2)
    cv2.putText(canvas, symbol, origin, FONT, scale, 0, thickness, cv2.LINE_AA)
    return canvas


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, help="каталог для шаблонов")
    parser.add_argument("--rows", type=int, default=6, help="сколько букв (A..)")
    parser.add_argument("--cols", type=int, default=6, help="сколько цифр (1..)")
    parser.add_argument("--size", type=int, default=96)
    args = parser.parse_args(argv)

    directory = Path(args.output).expanduser()
    directory.mkdir(parents=True, exist_ok=True)

    symbols = [chr(ord("A") + i) for i in range(args.rows)]
    symbols += [str(i + 1) for i in range(args.cols)]
    for symbol in symbols:
        path = directory / f"{symbol}.png"
        cv2.imwrite(str(path), render(symbol, size=args.size))

    print(f"Записано шаблонов: {len(symbols)} в {directory}")
    print("Замените их картинками организаторов, сохранив имена файлов.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
