"""Тесты плагинов зрения. Работают на синтетических кадрах, без камеры."""

import cv2
import numpy as np
import pytest

from td_perception_drone.plugins.base import available_detectors, create_detector
from td_perception_drone.plugins.hsv_color import detect_color_blob
from td_perception_drone.plugins.vlm_query import interpret_answer
from td_perception_drone.plugins.yolo_onnx import decode_yolo_output

ORANGE_LOWER = (10, 120, 90)
ORANGE_UPPER = (28, 255, 255)


def frame_with_blob(colour_bgr, centre=(400, 300), radius=40, size=(480, 640)):
    image = np.zeros((size[0], size[1], 3), np.uint8)
    image[:] = (40, 40, 40)
    cv2.circle(image, centre, radius, colour_bgr, -1)
    return image


def test_finds_orange_blob_and_reports_its_centre():
    image = frame_with_blob((20, 140, 235))  # оранжевый в BGR
    found = detect_color_blob(image, ORANGE_LOWER, ORANGE_UPPER, min_area_px=300)
    assert found is not None
    assert found.pixel[0] == pytest.approx(400, abs=3)
    assert found.pixel[1] == pytest.approx(300, abs=3)
    assert 0.0 < found.confidence <= 1.0


def test_ignores_blob_of_another_colour():
    image = frame_with_blob((235, 60, 20))  # синий
    assert detect_color_blob(image, ORANGE_LOWER, ORANGE_UPPER, min_area_px=300) is None


def test_ignores_too_small_blob():
    image = frame_with_blob((20, 140, 235), radius=4)
    assert detect_color_blob(image, ORANGE_LOWER, ORANGE_UPPER, min_area_px=300) is None


def test_picks_the_largest_blob_when_several_present():
    image = frame_with_blob((20, 140, 235), centre=(100, 100), radius=20)
    cv2.circle(image, (500, 350), 60, (20, 140, 235), -1)
    found = detect_color_blob(image, ORANGE_LOWER, ORANGE_UPPER, min_area_px=300)
    assert found is not None
    assert found.pixel[0] == pytest.approx(500, abs=5)


def test_empty_frame_is_handled():
    assert detect_color_blob(np.zeros((0, 0, 3), np.uint8), ORANGE_LOWER, ORANGE_UPPER) is None
    assert detect_color_blob(None, ORANGE_LOWER, ORANGE_UPPER) is None


def test_plugin_is_created_by_name():
    detector = create_detector("hsv_color", {"hsv_lower": ORANGE_LOWER,
                                             "hsv_upper": ORANGE_UPPER,
                                             "min_area_px": 300})
    result = detector.detect(frame_with_blob((20, 140, 235)))
    assert result is not None
    assert detector.describe() == "drone/hsv_color"


def test_unknown_plugin_reports_available_ones():
    with pytest.raises(KeyError, match="hsv_color"):
        create_detector("no_such_plugin")
    assert "hsv_color" in available_detectors()


def test_yolo_decoding_picks_best_row():
    output = np.array(
        [
            [10.0, 10.0, 20.0, 20.0, 0.9, 0.1, 0.8],
            [50.0, 50.0, 10.0, 10.0, 0.5, 0.9, 0.05],
        ]
    )
    box, score, class_id = decode_yolo_output(output, conf_threshold=0.3, class_id=None)
    assert class_id == 1
    assert score == pytest.approx(0.72, abs=1e-6)
    assert box.tolist() == [10.0, 10.0, 20.0, 20.0]


def test_yolo_decoding_respects_threshold():
    output = np.array([[10.0, 10.0, 20.0, 20.0, 0.2, 0.1, 0.1]])
    assert decode_yolo_output(output, conf_threshold=0.5, class_id=None) is None


@pytest.mark.parametrize(
    "answer,expected",
    [
        ("Да, на изображении игрушка", True),
        ("yes, I see a toy", True),
        ("Нет, ничего похожего", False),
        ("", False),
        ("да, но возможно нет", False),
    ],
)
def test_vlm_answer_interpretation(answer, expected):
    found, confidence = interpret_answer(answer)
    assert found is expected
    assert 0.0 <= confidence <= 1.0
