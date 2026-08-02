"""Точка расширения чтения метки финиша.

Наружу любой плагин отдаёт одинаковый `LabelResult` — сырой текст и
уверенность. Нормализацией и голосованием занимается `label_parse`, поэтому
плагину достаточно «как-то» прочитать надпись.

Плагины по возрастанию сложности:
    template_match  — сопоставление с картинками букв от организаторов;
    ocr_tesseract   — классический OCR, если он установлен;
    vlm_label       — вопрос к VLM (`/vlm/query`), самый устойчивый к шрифту.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any

__all__ = [
    "LabelResult",
    "LabelReader",
    "register_reader",
    "create_reader",
    "available_readers",
]


@dataclass
class LabelResult:
    """Что плагин прочитал на кадре."""

    text: str
    confidence: float
    bbox: tuple[int, int, int, int] = (0, 0, 0, 0)
    debug: dict[str, Any] = field(default_factory=dict)


class LabelReader(abc.ABC):
    name: str = "base"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        self.params = dict(params or {})

    @abc.abstractmethod
    def read(self, image) -> LabelResult | None:
        """Прочитать надпись на BGR-кадре. None — не прочитано."""

    def describe(self) -> str:
        return f"rover/{self.name}"


_REGISTRY: dict[str, type[LabelReader]] = {}


def register_reader(cls: type[LabelReader]) -> type[LabelReader]:
    _REGISTRY[cls.name] = cls
    return cls


def available_readers() -> list[str]:
    return sorted(_REGISTRY)


def create_reader(name: str, params: dict[str, Any] | None = None) -> LabelReader:
    from . import ocr_tesseract, template_match, vlm_label  # noqa: F401

    try:
        cls = _REGISTRY[name]
    except KeyError as exc:
        raise KeyError(
            f"Неизвестный плагин чтения {name!r}. "
            f"Доступны: {', '.join(available_readers())}"
        ) from exc
    return cls(params)
