"""Общая модель мира системы «Дозор»: сетка поля, конфигурация, фьюжн фактов."""

from .field import FieldModel, load_field
from .fusion import BeliefStore, Fact, KindPolicy
from .grid import CellRef, Grid, InvalidCellError, Transform2D

__all__ = [
    "BeliefStore",
    "CellRef",
    "Fact",
    "FieldModel",
    "Grid",
    "InvalidCellError",
    "KindPolicy",
    "Transform2D",
    "load_field",
]
