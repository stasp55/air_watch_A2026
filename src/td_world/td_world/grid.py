"""Сетка игрового поля и преобразования координат.

Модуль намеренно не зависит от ROS: это чистая арифметика, которая покрывается
юнит-тестами на любой машине. Вся система обязана пользоваться только им —
иначе неизбежно расползутся два несовместимых понимания того, где находится
клетка "A3".

Соглашение о координатах (field-фрейм)
--------------------------------------
Строки обозначаются буквами A..F сверху вниз (как на фотографии поля),
столбцы — цифрами 1..6 слева направо. Начало field-фрейма — левый нижний угол
сетки, то есть внешний угол клетки F1::

    y
    ^   A1 A2 A3 A4 A5 A6
    |   B1 ...
    |   ...
    |   F1 F2 F3 F4 F5 F6
    +-------------------------> x

Ось x растёт с номером столбца, ось y — с буквой в обратную сторону (F -> A),
z смотрит вверх. Фрейм правый, как того требует REP-103.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Iterable, Iterator, Sequence

__all__ = [
    "CellRef",
    "Grid",
    "Transform2D",
    "InvalidCellError",
    "ROW_LETTERS",
]

ROW_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"

_CELL_RE = re.compile(r"^\s*([A-Za-z])\s*[-_ ]?\s*(\d{1,2})\s*$")


class InvalidCellError(ValueError):
    """Имя клетки не разбирается или выходит за пределы поля."""


@dataclass(frozen=True, order=True)
class CellRef:
    """Ссылка на клетку: одновременно имя ("A3") и индексы (row=0, col=2)."""

    row: int
    col: int

    @property
    def name(self) -> str:
        return f"{ROW_LETTERS[self.row]}{self.col + 1}"

    @classmethod
    def parse(cls, text: str) -> "CellRef":
        """Разобрать "A3", "a3", "A-3", " b 5 " в CellRef.

        Проверка попадания в границы поля — задача Grid; здесь только формат,
        поэтому функцию можно использовать и до загрузки конфигурации.
        """
        if not isinstance(text, str):
            raise InvalidCellError(f"Ожидалась строка, получено {type(text).__name__}")
        match = _CELL_RE.match(text)
        if match is None:
            raise InvalidCellError(f"Не похоже на имя клетки: {text!r}")
        letter, digits = match.groups()
        row = ROW_LETTERS.index(letter.upper())
        col = int(digits) - 1
        if col < 0:
            raise InvalidCellError(f"Номер столбца начинается с 1, получено {text!r}")
        return cls(row=row, col=col)

    def manhattan(self, other: "CellRef") -> int:
        return abs(self.row - other.row) + abs(self.col - other.col)

    def __str__(self) -> str:  # pragma: no cover - тривиально
        return self.name


@dataclass(frozen=True)
class Transform2D:
    """Плоское жёсткое преобразование: поворот вокруг z, затем сдвиг.

    Используется для привязки field-фрейма к карте ровера (`map`) и к локальному
    фрейму дрона. Точка преобразуется как ``dst = R(yaw) @ src + t``.
    """

    tx: float = 0.0
    ty: float = 0.0
    yaw: float = 0.0

    def apply(self, x: float, y: float) -> tuple[float, float]:
        cos_a, sin_a = math.cos(self.yaw), math.sin(self.yaw)
        return (
            cos_a * x - sin_a * y + self.tx,
            sin_a * x + cos_a * y + self.ty,
        )

    def apply_yaw(self, yaw: float) -> float:
        return _wrap_pi(yaw + self.yaw)

    def inverse(self) -> "Transform2D":
        cos_a, sin_a = math.cos(-self.yaw), math.sin(-self.yaw)
        return Transform2D(
            tx=-(cos_a * self.tx - sin_a * self.ty),
            ty=-(sin_a * self.tx + cos_a * self.ty),
            yaw=-self.yaw,
        )

    @classmethod
    def fit(
        cls,
        src: Sequence[tuple[float, float]],
        dst: Sequence[tuple[float, float]],
    ) -> "Transform2D":
        """Подобрать преобразование по парам соответствий (калибровка поля).

        Нужно минимум две пары точек. Реализован классический метод Кабша для
        плоского случая: он даёт жёсткое преобразование без масштаба, что
        физически корректно — поле не растягивается.
        """
        if len(src) != len(dst):
            raise ValueError("Списки соответствий разной длины")
        if len(src) < 2:
            raise ValueError("Нужно минимум две пары точек для калибровки")

        n = len(src)
        sx = sum(p[0] for p in src) / n
        sy = sum(p[1] for p in src) / n
        dx = sum(p[0] for p in dst) / n
        dy = sum(p[1] for p in dst) / n

        num = 0.0  # sum(cross)
        den = 0.0  # sum(dot)
        for (px, py), (qx, qy) in zip(src, dst):
            ax, ay = px - sx, py - sy
            bx, by = qx - dx, qy - dy
            num += ax * by - ay * bx
            den += ax * bx + ay * by
        if abs(num) < 1e-12 and abs(den) < 1e-12:
            raise ValueError("Вырожденные точки: все соответствия совпадают")
        yaw = math.atan2(num, den)

        cos_a, sin_a = math.cos(yaw), math.sin(yaw)
        return cls(
            tx=dx - (cos_a * sx - sin_a * sy),
            ty=dy - (sin_a * sx + cos_a * sy),
            yaw=yaw,
        )


class Grid:
    """Сетка поля: перевод клетка <-> метры, соседи, валидация."""

    def __init__(self, rows: int = 6, cols: int = 6, cell_size: float = 0.8) -> None:
        if rows <= 0 or cols <= 0:
            raise ValueError("Размер поля должен быть положительным")
        if rows > len(ROW_LETTERS):
            raise ValueError(f"Поддерживается максимум {len(ROW_LETTERS)} строк")
        if cell_size <= 0.0:
            raise ValueError("Размер клетки должен быть положительным")
        self.rows = int(rows)
        self.cols = int(cols)
        self.cell_size = float(cell_size)

    # --- размеры -----------------------------------------------------------

    @property
    def width(self) -> float:
        """Размер поля вдоль x, м."""
        return self.cols * self.cell_size

    @property
    def height(self) -> float:
        """Размер поля вдоль y, м."""
        return self.rows * self.cell_size

    # --- валидация ---------------------------------------------------------

    def contains(self, cell: CellRef) -> bool:
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def cell(self, text: str) -> CellRef:
        """Разобрать имя клетки и проверить, что она лежит на поле."""
        ref = CellRef.parse(text)
        if not self.contains(ref):
            raise InvalidCellError(
                f"Клетка {ref.name} вне поля {self.rows}x{self.cols}"
            )
        return ref

    def cells(self) -> Iterator[CellRef]:
        for row in range(self.rows):
            for col in range(self.cols):
                yield CellRef(row, col)

    # --- геометрия ---------------------------------------------------------

    def center(self, cell: CellRef) -> tuple[float, float]:
        """Центр клетки в field-фрейме, метры."""
        if not self.contains(cell):
            raise InvalidCellError(f"Клетка {cell.name} вне поля")
        x = (cell.col + 0.5) * self.cell_size
        y = (self.rows - 0.5 - cell.row) * self.cell_size
        return x, y

    def corners(self, cell: CellRef) -> tuple[float, float, float, float]:
        """Границы клетки (x_min, y_min, x_max, y_max) в field-фрейме."""
        cx, cy = self.center(cell)
        half = self.cell_size / 2.0
        return cx - half, cy - half, cx + half, cy + half

    def cell_at(self, x: float, y: float) -> CellRef | None:
        """Клетка, в которую попадает точка field-фрейма. None — вне поля."""
        col = math.floor(x / self.cell_size)
        row = self.rows - 1 - math.floor(y / self.cell_size)
        ref = CellRef(int(row), int(col))
        return ref if self.contains(ref) else None

    def clamp_to_field(self, x: float, y: float) -> tuple[float, float]:
        """Прижать точку к полю — защита от вылета оценки за границы."""
        eps = 1e-6
        return (
            min(max(x, 0.0), self.width - eps),
            min(max(y, 0.0), self.height - eps),
        )

    # --- топология ---------------------------------------------------------

    def neighbours(self, cell: CellRef, diagonal: bool = False) -> list[CellRef]:
        """Соседние клетки поля (без учёта стен — это забота планировщика)."""
        deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if diagonal:
            deltas += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for d_row, d_col in deltas:
            ref = CellRef(cell.row + d_row, cell.col + d_col)
            if self.contains(ref):
                result.append(ref)
        return result

    def expand(self, cells: Iterable[CellRef], diagonal: bool = False) -> list[CellRef]:
        """Клетки плюс их соседи, без дубликатов и в стабильном порядке."""
        out: dict[CellRef, None] = {}
        for cell in cells:
            if self.contains(cell):
                out[cell] = None
            for neighbour in self.neighbours(cell, diagonal=diagonal):
                out[neighbour] = None
        return sorted(out)


def _wrap_pi(angle: float) -> float:
    """Привести угол к диапазону (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
