"""Нормализация и голосование по прочитанному имени клетки.

Любой распознаватель — шаблонный, OCR или VLM — возвращает грязный текст:
кириллица вместо латиницы, лишние слова, ноль вместо буквы O. Приводим всё к
канону "A3" здесь, в одном месте, и покрываем тестами: на площадке отлаживать
эту логику будет некогда.
"""

from __future__ import annotations

from collections import Counter
import re
from typing import Iterable

__all__ = ["normalize_label", "vote_labels", "LabelVote"]

# Кириллические буквы, визуально неотличимые от латинских.
_CYRILLIC_TO_LATIN = {
    "А": "A", "В": "B", "С": "C", "Е": "E", "Н": "H", "К": "K",
    "М": "M", "О": "O", "Р": "P", "Т": "T", "Х": "X", "У": "Y",
    "а": "A", "в": "B", "с": "C", "е": "E", "о": "O", "р": "P", "х": "X",
}

# Частые ошибки распознавания цифр.
_DIGIT_FIXES = {"O": "0", "О": "0", "Q": "0", "D": "0",
                "I": "1", "L": "1", "|": "1",
                "Z": "2", "S": "5", "B": "8", "G": "6"}

_LABEL_RE = re.compile(r"([A-Z])\s*[-_.]?\s*([0-9])")


def _transliterate(text: str) -> str:
    return "".join(_CYRILLIC_TO_LATIN.get(char, char) for char in text)


def normalize_label(text: str, rows: int = 6, cols: int = 6) -> str | None:
    """Привести сырой ответ распознавателя к виду "A3".

    Возвращает None, если ничего похожего на имя клетки нет либо клетка вне
    поля — лучше признать неудачу и переснять кадр, чем поехать не туда.
    """
    if not text:
        return None

    cleaned = _transliterate(str(text)).upper()
    cleaned = re.sub(r"[^A-Z0-9]+", " ", cleaned).strip()
    if not cleaned:
        return None

    for match in _LABEL_RE.finditer(cleaned):
        letter, digit = match.groups()
        if _in_field(letter, digit, rows, cols):
            return f"{letter}{digit}"

    # Второй проход: две «склеенные» позиции, где цифру приняли за букву.
    compact = cleaned.replace(" ", "")
    if len(compact) >= 2:
        letter = compact[0]
        digit = _DIGIT_FIXES.get(compact[1], compact[1])
        if letter.isalpha() and digit.isdigit() and _in_field(letter, digit, rows, cols):
            return f"{letter}{digit}"
    return None


def _in_field(letter: str, digit: str, rows: int, cols: int) -> bool:
    row = ord(letter) - ord("A")
    col = int(digit) - 1
    return 0 <= row < rows and 0 <= col < cols


class LabelVote:
    """Итог голосования по нескольким кадрам."""

    def __init__(self, label: str | None, votes: int, total: int, confidence: float) -> None:
        self.label = label
        self.votes = votes
        self.total = total
        self.confidence = confidence

    def __repr__(self) -> str:  # pragma: no cover - для логов
        return f"LabelVote({self.label!r}, {self.votes}/{self.total}, {self.confidence:.2f})"


def vote_labels(
    readings: Iterable[tuple[str, float]],
    rows: int = 6,
    cols: int = 6,
) -> LabelVote:
    """Свести несколько прочтений к одному ответу.

    Каждое прочтение — пара (текст, уверенность распознавателя). Побеждает
    мода; итоговая уверенность учитывает и согласованность кадров, и качество
    самих распознаваний, поэтому одиночное «повезло» не проходит порог.
    """
    normalized: list[tuple[str, float]] = []
    total = 0
    for text, confidence in readings:
        total += 1
        label = normalize_label(text, rows=rows, cols=cols)
        if label is not None:
            normalized.append((label, float(confidence)))

    if not normalized:
        return LabelVote(None, 0, total, 0.0)

    counter = Counter(label for label, _ in normalized)
    label, votes = counter.most_common(1)[0]
    scores = [conf for name, conf in normalized if name == label]
    agreement = votes / float(max(1, total))
    quality = sum(scores) / len(scores)
    return LabelVote(label, votes, total, float(min(1.0, 0.5 * agreement + 0.5 * quality)))
