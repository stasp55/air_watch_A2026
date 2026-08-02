"""Базовый детектор: поиск объекта по цвету в пространстве HSV.

Рабочая лошадка для поиска игрушки: быстрый, предсказуемый, настраивается
двумя порогами прямо на площадке под конкретный свет. Ограничение известно —
цвет чувствителен к освещению, поэтому в конфигурации предусмотрены запасные
плагины (`yolo_onnx`, `vlm_query`).

Уверенность считается не «на глаз», а из измеримых величин: доли заполнения
контура и близости площади к ожидаемой. Это делает порог `min_confidence`
осмысленным и переносимым между площадками.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .base import DetectionResult, ObjectDetector, register_detector

__all__ = ["HsvColorDetector", "detect_color_blob"]


def detect_color_blob(
    image: np.ndarray,
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
    min_area_px: int = 300,
    max_area_px: int = 0,
    open_kernel: int = 3,
    lower2: tuple[int, int, int] | None = None,
    upper2: tuple[int, int, int] | None = None,
) -> DetectionResult | None:
    """Найти самое крупное цветное пятно в заданном диапазоне HSV.

    Второй диапазон (`lower2`/`upper2`) нужен для красного цвета, который в
    HSV разрезан на две части около нуля.
    """
    if image is None or image.size == 0:
        return None

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    if lower2 is not None and upper2 is not None:
        mask = cv2.bitwise_or(
            mask, cv2.inRange(hsv, np.array(lower2, np.uint8), np.array(upper2, np.uint8))
        )

    if open_kernel > 0:
        kernel = np.ones((open_kernel, open_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area_px:
        return None
    if max_area_px and area > max_area_px:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    moments = cv2.moments(contour)
    if moments["m00"] <= 0.0:
        return None
    cx = moments["m10"] / moments["m00"]
    cy = moments["m01"] / moments["m00"]

    fill = area / float(max(1, w * h))            # насколько пятно плотное
    size_score = min(1.0, area / float(max(1, min_area_px * 4)))
    confidence = float(np.clip(0.35 + 0.4 * fill + 0.25 * size_score, 0.0, 1.0))

    return DetectionResult(
        pixel=(float(cx), float(cy)),
        confidence=confidence,
        bbox=(int(x), int(y), int(w), int(h)),
        label="color_blob",
        debug={"area_px": area, "fill": round(fill, 3)},
    )


@register_detector
class HsvColorDetector(ObjectDetector):
    """Плагин-обёртка над `detect_color_blob`."""

    name = "hsv_color"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        self.lower = tuple(self.params.get("hsv_lower", (10, 120, 90)))
        self.upper = tuple(self.params.get("hsv_upper", (28, 255, 255)))
        second_lower = self.params.get("hsv_lower2")
        second_upper = self.params.get("hsv_upper2")
        self.lower2 = tuple(second_lower) if second_lower else None
        self.upper2 = tuple(second_upper) if second_upper else None
        self.min_area_px = int(self.params.get("min_area_px", 300))
        self.max_area_px = int(self.params.get("max_area_px", 0))
        self.open_kernel = int(self.params.get("open_kernel", 3))

    def detect(self, image) -> DetectionResult | None:
        return detect_color_blob(
            image,
            lower=self.lower,
            upper=self.upper,
            min_area_px=self.min_area_px,
            max_area_px=self.max_area_px,
            open_kernel=self.open_kernel,
            lower2=self.lower2,
            upper2=self.upper2,
        )
