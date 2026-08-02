"""Тесты шаблонного чтения метки на синтетических изображениях."""

import cv2
import numpy as np
import pytest

from td_perception_rover.label_parse import normalize_label
from td_perception_rover.plugins.template_match import (
    extract_glyphs,
    load_templates,
    read_label,
)

FONT = cv2.FONT_HERSHEY_SIMPLEX


def render_symbol(symbol: str, size: int = 80, thickness: int = 4) -> np.ndarray:
    """Отрисовать один символ так, как выглядел бы эталон организаторов."""
    canvas = np.full((size, size), 255, np.uint8)
    (w, h), _ = cv2.getTextSize(symbol, FONT, 2.0, thickness)
    cv2.putText(canvas, symbol, ((size - w) // 2, (size + h) // 2),
                FONT, 2.0, 0, thickness, cv2.LINE_AA)
    return canvas


def render_label(text: str, width: int = 320, height: int = 200) -> np.ndarray:
    """Кадр с «бумажкой»: тёмная надпись на белом листе."""
    image = np.full((height, width, 3), 200, np.uint8)
    cv2.rectangle(image, (40, 40), (width - 40, height - 40), (255, 255, 255), -1)
    (w, h), _ = cv2.getTextSize(text, FONT, 2.4, 6)
    cv2.putText(image, text, ((width - w) // 2, (height + h) // 2),
                FONT, 2.4, (0, 0, 0), 6, cv2.LINE_AA)
    return image


@pytest.fixture(scope="module")
def templates(tmp_path_factory):
    directory = tmp_path_factory.mktemp("templates")
    for symbol in "ABCDEF123456":
        cv2.imwrite(str(directory / f"{symbol}.png"), render_symbol(symbol))
    return load_templates(directory)


def test_templates_are_loaded(templates):
    assert set(templates) == set("ABCDEF123456")
    assert all(image.shape == (40, 40) for image in templates.values())


def test_missing_templates_directory_is_reported():
    with pytest.raises(FileNotFoundError):
        load_templates("/nonexistent/templates/dir")


def test_two_glyphs_are_extracted_left_to_right():
    glyphs = extract_glyphs(render_label("B5"))
    assert len(glyphs) == 2
    assert glyphs[0][1][0] < glyphs[1][1][0]


@pytest.mark.parametrize("label", ["A3", "B5", "C2", "E4", "F6"])
def test_labels_are_read_correctly(templates, label):
    result = read_label(render_label(label), templates)
    assert result is not None
    assert normalize_label(result.text) == label
    assert result.confidence > 0.5


def test_blank_paper_yields_nothing(templates):
    blank = np.full((200, 320, 3), 255, np.uint8)
    assert read_label(blank, templates) is None


def test_templates_must_contain_letters_and_digits():
    letters_only = {"A": np.zeros((40, 40), np.uint8)}
    with pytest.raises(ValueError, match="буквы"):
        read_label(render_label("A3"), letters_only)
