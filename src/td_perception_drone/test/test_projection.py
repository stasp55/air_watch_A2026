"""Тесты проецирования пикселя на пол поля."""

import math

import numpy as np
import pytest

from td_perception_drone.projection import (
    CameraModel,
    ground_to_pixel,
    pixel_to_ground,
    quaternion_to_matrix,
)

CAMERA = CameraModel(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)

# Камера смотрит строго вниз: оптическая ось z совпадает с -z поля.
# Это поворот на 180 градусов вокруг оси x.
DOWNWARD = quaternion_to_matrix(1.0, 0.0, 0.0, 0.0)


def test_quaternion_identity():
    assert np.allclose(quaternion_to_matrix(0.0, 0.0, 0.0, 1.0), np.eye(3))


def test_quaternion_rejects_zero():
    with pytest.raises(ValueError):
        quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)


def test_center_pixel_maps_to_point_under_camera():
    ground = pixel_to_ground((320.0, 240.0), CAMERA, (2.0, 2.0, 2.5), DOWNWARD)
    assert ground == pytest.approx((2.0, 2.0))


def test_offset_pixel_scales_with_altitude():
    """Смещение на 60 пикселей при f=600 и высоте 2 м даёт ровно 0.2 м."""
    ground = pixel_to_ground((380.0, 240.0), CAMERA, (2.0, 2.0, 2.0), DOWNWARD)
    assert ground == pytest.approx((2.2, 2.0))


def test_y_axis_is_flipped_for_downward_camera():
    """Пиксель ниже центра кадра соответствует меньшему y в поле."""
    ground = pixel_to_ground((320.0, 300.0), CAMERA, (2.0, 2.0, 2.0), DOWNWARD)
    assert ground[1] < 2.0


def test_ray_pointing_up_has_no_ground_point():
    upward = quaternion_to_matrix(0.0, 0.0, 0.0, 1.0)  # оптическая ось вверх по +z
    assert pixel_to_ground((320.0, 240.0), CAMERA, (2.0, 2.0, 2.0), upward) is None


def test_projection_roundtrip():
    for point in [(0.4, 0.4), (2.0, 3.6), (4.4, 4.4)]:
        pixel = ground_to_pixel(point, CAMERA, (2.4, 2.4, 3.0), DOWNWARD)
        assert pixel is not None
        back = pixel_to_ground(pixel, CAMERA, (2.4, 2.4, 3.0), DOWNWARD)
        assert back == pytest.approx(point, abs=1e-6)


def test_tilted_camera_still_hits_the_floor():
    """Наклон на 20 градусов смещает точку прицеливания вперёд, а не ломает расчёт."""
    tilt = math.radians(20.0)
    rotation = DOWNWARD @ np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(tilt), -math.sin(tilt)],
            [0.0, math.sin(tilt), math.cos(tilt)],
        ]
    )
    ground = pixel_to_ground((320.0, 240.0), CAMERA, (2.0, 2.0, 2.0), rotation)
    assert ground is not None
    assert ground[0] == pytest.approx(2.0)
    assert ground[1] != pytest.approx(2.0)


def test_uncalibrated_camera_is_rejected():
    bad = CameraModel(fx=0.0, fy=0.0, cx=320.0, cy=240.0)
    with pytest.raises(ValueError):
        pixel_to_ground((0.0, 0.0), bad, (0.0, 0.0, 1.0), DOWNWARD)
