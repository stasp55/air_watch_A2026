"""Основной способ чтения метки: сопоставление с эталонными картинками.

Организаторы выдают изображения букв и цифр — из них собирается набор
шаблонов, и задача сводится к сравнению двух вырезанных глифов с эталонами.
Метод не требует ни обучения, ни интернета, работает на CPU ровера за
единицы миллисекунд и полностью детерминирован, что важно на зачётной попытке.

Каталог шаблонов::

    templates/
        A.png B.png C.png D.png E.png F.png
        1.png 2.png 3.png 4.png 5.png 6.png

Имя файла (без расширения) — это символ, который он изображает.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .base import LabelReader, LabelResult, register_reader

__all__ = ["TemplateMatcher", "load_templates", "extract_glyphs", "read_label"]

GLYPH_SIZE = 40


def load_templates(directory: str | Path) -> dict[str, np.ndarray]:
    """Загрузить эталоны из каталога. Ключ — символ из имени файла."""
    directory = Path(directory)
    if not directory.is_dir():
        raise FileNotFoundError(f"Каталог шаблонов не найден: {directory}")

    templates: dict[str, np.ndarray] = {}
    for path in sorted(directory.iterdir()):
        if path.suffix.lower() not in {".png", ".jpg", ".jpeg", ".bmp"}:
            continue
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        symbol = path.stem.strip().upper()[:1]
        if symbol:
            templates[symbol] = _normalize_glyph(image)
    if not templates:
        raise FileNotFoundError(f"В каталоге {directory} нет пригодных шаблонов")
    return templates


def _normalize_glyph(gray: np.ndarray) -> np.ndarray:
    """Привести глиф к единому виду: тёмное на светлом, обрезка, размер."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    coords = cv2.findNonZero(binary)
    if coords is not None:
        x, y, w, h = cv2.boundingRect(coords)
        if w > 1 and h > 1:
            binary = binary[y:y + h, x:x + w]
    return cv2.resize(binary, (GLYPH_SIZE, GLYPH_SIZE), interpolation=cv2.INTER_AREA)


def _binarize(gray: np.ndarray, adaptive: bool) -> np.ndarray:
    """Выделить тёмные символы. Otsu — основной путь, adaptive — для теней."""
    if adaptive:
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 10
        )
    else:
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def _glyph_boxes(binary: np.ndarray, min_area_px: int) -> list[tuple[int, int, int, int]]:
    """Рамки-кандидаты символов с отбраковкой фона и краёв кадра."""
    height, width = binary.shape[:2]
    frame_area = float(height * width)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        # Контур края бумаги или всего кадра — не символ.
        if w * h > 0.25 * frame_area:
            continue
        if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
            continue
        if w * h < min_area_px or h < height * 0.05:
            continue
        aspect = w / float(h)
        if aspect > 2.0 or aspect < 0.15:
            continue
        boxes.append((x, y, w, h))
    return boxes


def extract_glyphs(
    image: np.ndarray,
    max_glyphs: int = 2,
    min_area_px: int = 80,
) -> list[tuple[np.ndarray, tuple[int, int, int, int]]]:
    """Вырезать отдельные символы надписи, слева направо.

    Сначала пробуется глобальный порог Otsu — на белой бумаге он даёт чистые
    символы. Если символов нашлось меньше двух (например, лист лежит в тени),
    повторяем с адаптивным порогом.
    """
    if image is None or image.size == 0:
        return []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    binary = _binarize(gray, adaptive=False)
    boxes = _glyph_boxes(binary, min_area_px)
    if len(boxes) < 2:
        adaptive_binary = _binarize(gray, adaptive=True)
        adaptive_boxes = _glyph_boxes(adaptive_binary, min_area_px)
        if len(adaptive_boxes) > len(boxes):
            binary, boxes = adaptive_binary, adaptive_boxes

    boxes.sort(key=lambda b: b[2] * b[3], reverse=True)
    boxes = boxes[:max_glyphs]
    boxes.sort(key=lambda b: b[0])  # слева направо — так же, как читает человек

    glyphs = []
    for x, y, w, h in boxes:
        crop = binary[y:y + h, x:x + w]
        glyphs.append((cv2.resize(crop, (GLYPH_SIZE, GLYPH_SIZE),
                                  interpolation=cv2.INTER_AREA), (x, y, w, h)))
    return glyphs


def _match_glyph(glyph: np.ndarray, candidates: dict[str, np.ndarray]) -> tuple[str, float]:
    best_symbol, best_score = "", -1.0
    glyph_f = glyph.astype(np.float32)
    for symbol, template in candidates.items():
        score = float(
            cv2.matchTemplate(glyph_f, template.astype(np.float32), cv2.TM_CCOEFF_NORMED)[0][0]
        )
        if score > best_score:
            best_symbol, best_score = symbol, score
    return best_symbol, max(0.0, best_score)


def read_label(
    image: np.ndarray,
    templates: dict[str, np.ndarray],
) -> LabelResult | None:
    """Прочитать метку вида "A3": первый глиф — буква, второй — цифра."""
    letters = {k: v for k, v in templates.items() if k.isalpha()}
    digits = {k: v for k, v in templates.items() if k.isdigit()}
    if not letters or not digits:
        raise ValueError("В наборе шаблонов должны быть и буквы, и цифры")

    glyphs = extract_glyphs(image)
    if len(glyphs) < 2:
        return None

    (letter_glyph, letter_box), (digit_glyph, digit_box) = glyphs[0], glyphs[1]
    letter, letter_score = _match_glyph(letter_glyph, letters)
    digit, digit_score = _match_glyph(digit_glyph, digits)

    x = min(letter_box[0], digit_box[0])
    y = min(letter_box[1], digit_box[1])
    x2 = max(letter_box[0] + letter_box[2], digit_box[0] + digit_box[2])
    y2 = max(letter_box[1] + letter_box[3], digit_box[1] + digit_box[3])

    return LabelResult(
        text=f"{letter}{digit}",
        confidence=float(min(letter_score, digit_score)),
        bbox=(x, y, x2 - x, y2 - y),
        debug={"letter_score": round(letter_score, 3), "digit_score": round(digit_score, 3)},
    )


@register_reader
class TemplateMatcher(LabelReader):
    name = "template_match"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        directory = self.params.get("templates_dir", "")
        if not directory:
            raise ValueError("template_match: обязателен параметр templates_dir")
        self.templates = load_templates(directory)

    def read(self, image) -> LabelResult | None:
        return read_label(image, self.templates)
