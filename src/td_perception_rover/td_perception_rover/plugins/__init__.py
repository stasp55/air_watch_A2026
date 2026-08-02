"""Плагины чтения метки финиша."""

from .base import (
    LabelReader,
    LabelResult,
    available_readers,
    create_reader,
    register_reader,
)

__all__ = [
    "LabelReader",
    "LabelResult",
    "available_readers",
    "create_reader",
    "register_reader",
]
