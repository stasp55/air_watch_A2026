"""Точка расширения технического зрения дрона.

Диспетчер и модель мира знают только о `DetectionResult`. Как именно объект
найден — цветом, нейросетью или VLM — определяется параметром ноды
`detector.plugin`, поэтому реализацию можно заменить на площадке за минуту,
не трогая логику миссии.

Свой детектор добавляется так::

    class MyDetector(ObjectDetector):
        name = "my_detector"
        def detect(self, image): ...

    register_detector(MyDetector)
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any

__all__ = [
    "DetectionResult",
    "ObjectDetector",
    "register_detector",
    "create_detector",
    "available_detectors",
]


@dataclass
class DetectionResult:
    """Что увидел плагин на одном кадре."""

    pixel: tuple[float, float]          # центр объекта, пиксели
    confidence: float                   # 0..1
    bbox: tuple[int, int, int, int] = (0, 0, 0, 0)  # x, y, w, h
    label: str = ""                     # текстовая расшифровка, если есть
    debug: dict[str, Any] = field(default_factory=dict)


class ObjectDetector(abc.ABC):
    """Базовый класс детектора объекта на кадре."""

    #: имя, по которому плагин выбирается в конфигурации
    name: str = "base"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        self.params = dict(params or {})

    @abc.abstractmethod
    def detect(self, image) -> DetectionResult | None:
        """Найти объект на BGR-кадре. None — не найден."""

    def describe(self) -> str:
        """Строка для логов и для поля `source` в детекции."""
        return f"drone/{self.name}"


_REGISTRY: dict[str, type[ObjectDetector]] = {}


def register_detector(cls: type[ObjectDetector]) -> type[ObjectDetector]:
    _REGISTRY[cls.name] = cls
    return cls


def available_detectors() -> list[str]:
    return sorted(_REGISTRY)


def create_detector(name: str, params: dict[str, Any] | None = None) -> ObjectDetector:
    """Создать детектор по имени плагина."""
    # Импорт здесь, а не наверху: тяжёлые зависимости (onnxruntime, rclpy)
    # подтягиваются только если соответствующий плагин реально выбран.
    from . import cheburashka_hsv, hsv_color, vlm_query, yolo_onnx  # noqa: F401

    try:
        cls = _REGISTRY[name]
    except KeyError as exc:
        raise KeyError(
            f"Неизвестный плагин детектора {name!r}. "
            f"Доступны: {', '.join(available_detectors())}"
        ) from exc
    return cls(params)
