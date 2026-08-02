"""Проецирование пикселя в точку на полу поля.

Дрон видит цель как пиксель, а миссии нужна клетка. Пересчёт делается через
камерную модель и позу камеры: луч из оптического центра пересекается с
плоскостью пола. Модуль чистый (numpy), поэтому проверяется тестами без
дрона и без ROS.

Система координат камеры — стандартная оптическая (REP-103): x вправо,
y вниз, z вперёд по оптической оси.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["CameraModel", "quaternion_to_matrix", "pixel_to_ground", "ground_to_pixel"]


@dataclass(frozen=True)
class CameraModel:
    """Внутренние параметры камеры (из sensor_msgs/CameraInfo)."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0

    @classmethod
    def from_camera_info(cls, info) -> "CameraModel":
        k = list(info.k) if hasattr(info, "k") else list(info.K)
        return cls(fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                   width=int(info.width), height=int(info.height))

    def validate(self) -> None:
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Камера не откалибрована: fx/fy должны быть положительными")


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Кватернион (x, y, z, w) -> матрица поворота 3x3."""
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm < 1e-9:
        raise ValueError("Нулевой кватернион")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def pixel_to_ground(
    pixel: tuple[float, float],
    camera: CameraModel,
    camera_position: tuple[float, float, float],
    camera_rotation: np.ndarray,
    plane_z: float = 0.0,
) -> tuple[float, float] | None:
    """Точка на плоскости `z = plane_z`, в которую смотрит пиксель.

    Возвращает None, если луч уходит вверх или параллелен полу — в этом случае
    оценка бессмысленна, и лучше честно сказать «не знаю», чем выдать
    произвольную клетку.
    """
    camera.validate()
    u, v = float(pixel[0]), float(pixel[1])
    ray_camera = np.array(
        [(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0], dtype=float
    )
    ray_world = np.asarray(camera_rotation, dtype=float) @ ray_camera
    origin = np.asarray(camera_position, dtype=float)

    if abs(ray_world[2]) < 1e-6:
        return None
    t = (plane_z - origin[2]) / ray_world[2]
    if t <= 0.0:
        return None
    point = origin + t * ray_world
    return float(point[0]), float(point[1])


def ground_to_pixel(
    point: tuple[float, float],
    camera: CameraModel,
    camera_position: tuple[float, float, float],
    camera_rotation: np.ndarray,
    plane_z: float = 0.0,
) -> tuple[float, float] | None:
    """Обратная задача: где на кадре окажется точка пола. Нужна для отладки."""
    camera.validate()
    world = np.array([point[0], point[1], plane_z], dtype=float)
    rel = np.asarray(camera_rotation, dtype=float).T @ (world - np.asarray(camera_position))
    if rel[2] <= 1e-6:
        return None
    return (
        float(camera.fx * rel[0] / rel[2] + camera.cx),
        float(camera.fy * rel[1] / rel[2] + camera.cy),
    )
