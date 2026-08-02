"""Запасной детектор: YOLO через cv2.dnn (ONNX).

Нужен, если цвет подводит — например, при резкой смене освещения на площадке.
Модель `yolov5n.onnx` уже лежит в `sverk_rover/src/system/rover_vision/models`,
путь задаётся параметром, поэтому вес не дублируется в этом репозитории.

Плагин намеренно ограничен: одна модель, один интересующий класс. Задача —
подтвердить наличие объекта и дать его центр, а не построить детектор общего
назначения.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .base import DetectionResult, ObjectDetector, register_detector

__all__ = ["YoloOnnxDetector", "decode_yolo_output"]


def decode_yolo_output(
    output: np.ndarray,
    conf_threshold: float,
    class_id: int | None,
) -> tuple[np.ndarray, float, int] | None:
    """Выбрать лучшую детекцию из «сырого» выхода YOLOv5/v8.

    Ожидается массив (N, 5+C) для v5 (x, y, w, h, obj, classes...) либо
    (N, 4+C) для v8 (x, y, w, h, classes...). Формат определяется по наличию
    столбца objectness: у v8 его нет, поэтому число столбцов на единицу меньше
    при том же числе классов — различаем по эвристике ниже.
    """
    if output.ndim != 2 or output.shape[0] == 0:
        return None

    columns = output.shape[1]
    if columns < 6:
        return None

    # v5: objectness в столбце 4, классы дальше. v8: классы сразу с 4-го.
    has_objectness = bool(np.all(output[:, 4] <= 1.0001)) and columns > 5
    class_scores = output[:, 5:] if has_objectness else output[:, 4:]
    objectness = output[:, 4] if has_objectness else np.ones(output.shape[0])

    if class_id is not None:
        if class_id >= class_scores.shape[1]:
            return None
        scores = class_scores[:, class_id] * objectness
        classes = np.full(output.shape[0], class_id)
    else:
        classes = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(output.shape[0]), classes] * objectness

    best = int(np.argmax(scores))
    if float(scores[best]) < conf_threshold:
        return None
    return output[best, :4], float(scores[best]), int(classes[best])


@register_detector
class YoloOnnxDetector(ObjectDetector):
    name = "yolo_onnx"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        self.model_path = str(self.params.get("model_path", ""))
        if not self.model_path:
            raise ValueError("yolo_onnx: обязателен параметр model_path")
        self.input_size = int(self.params.get("input_size", 640))
        self.conf_threshold = float(self.params.get("conf_threshold", 0.35))
        class_id = self.params.get("class_id", None)
        self.class_id = None if class_id in (None, -1, "") else int(class_id)
        self._net = cv2.dnn.readNetFromONNX(self.model_path)

    def detect(self, image) -> DetectionResult | None:
        if image is None or image.size == 0:
            return None
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(
            image, 1 / 255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False,
        )
        self._net.setInput(blob)
        raw = self._net.forward()
        output = np.squeeze(raw)
        if output.ndim == 2 and output.shape[0] < output.shape[1]:
            output = output.T  # YOLOv8 отдаёт (channels, anchors)

        decoded = decode_yolo_output(output, self.conf_threshold, self.class_id)
        if decoded is None:
            return None
        box, score, class_index = decoded

        scale_x = width / float(self.input_size)
        scale_y = height / float(self.input_size)
        cx, cy, w, h = box * np.array([scale_x, scale_y, scale_x, scale_y])
        return DetectionResult(
            pixel=(float(cx), float(cy)),
            confidence=float(min(1.0, score)),
            bbox=(int(cx - w / 2), int(cy - h / 2), int(w), int(h)),
            label=f"class_{class_index}",
            debug={"score": round(float(score), 3)},
        )
