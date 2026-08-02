"""Чтение метки классическим OCR (Tesseract), если он установлен.

Держится как третий независимый источник: шаблоны опираются на форму, VLM —
на семантику, OCR — на обученные шрифтовые модели. Если совпадают хотя бы
два, ответу можно верить.

Устанавливается отдельно::

    sudo apt install tesseract-ocr python3-pil
    pip3 install pytesseract
"""

from __future__ import annotations

from typing import Any

import cv2

from .base import LabelReader, LabelResult, register_reader

__all__ = ["TesseractReader"]

# Ограничиваем алфавит — на бумаге может быть только клетка поля.
WHITELIST = "ABCDEFGHIJ0123456789"


@register_reader
class TesseractReader(LabelReader):
    name = "ocr_tesseract"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        try:
            import pytesseract
        except ImportError as exc:  # pragma: no cover - зависит от окружения
            raise RuntimeError(
                "Плагин ocr_tesseract требует pytesseract: pip3 install pytesseract"
            ) from exc
        self._pytesseract = pytesseract
        self.psm = int(self.params.get("psm", 7))  # одна строка текста
        self.min_confidence = float(self.params.get("min_confidence", 0.3))

    def read(self, image) -> LabelResult | None:
        if image is None or image.size == 0:
            return None
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        config = f"--psm {self.psm} -c tessedit_char_whitelist={WHITELIST}"
        data = self._pytesseract.image_to_data(
            binary, config=config, output_type=self._pytesseract.Output.DICT
        )

        best_text, best_conf = "", -1.0
        for text, conf in zip(data.get("text", []), data.get("conf", [])):
            text = (text or "").strip()
            try:
                conf_value = float(conf)
            except (TypeError, ValueError):
                continue
            if text and conf_value > best_conf:
                best_text, best_conf = text, conf_value

        if not best_text or best_conf < 0.0:
            return None
        confidence = max(0.0, min(1.0, best_conf / 100.0))
        if confidence < self.min_confidence:
            return None
        return LabelResult(text=best_text, confidence=confidence,
                           debug={"tesseract_conf": best_conf})
