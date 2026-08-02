"""Плагины технического зрения дрона."""

from .base import (
    DetectionResult,
    ObjectDetector,
    available_detectors,
    create_detector,
    register_detector,
)

__all__ = [
    "DetectionResult",
    "ObjectDetector",
    "available_detectors",
    "create_detector",
    "register_detector",
]
