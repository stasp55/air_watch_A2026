"""Тесты нормализации и голосования по метке финиша."""

import pytest

from td_perception_rover.label_parse import normalize_label, vote_labels


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("A3", "A3"),
        ("a3", "A3"),
        ("A 3", "A3"),
        ("A-3", "A3"),
        ("Финишная клетка B5", "B5"),
        ("The finish cell is B5.", "B5"),
        ("  e2  ", "E2"),
    ],
)
def test_plain_labels(raw, expected):
    assert normalize_label(raw) == expected


def test_cyrillic_homoglyphs_are_transliterated():
    assert normalize_label("А3") == "A3"      # кириллическая А
    assert normalize_label("В5") == "B5"      # кириллическая В
    assert normalize_label("Е2") == "E2"      # кириллическая Е
    assert normalize_label("С4") == "C4"      # кириллическая С


def test_digit_lookalikes_are_repaired():
    assert normalize_label("AI") == "A1"      # I вместо 1
    assert normalize_label("AS") == "A5"      # S вместо 5


@pytest.mark.parametrize("raw", ["", "   ", "hello", "33", "!!!", None])
def test_garbage_is_rejected(raw):
    assert normalize_label(raw) is None


def test_cells_outside_the_field_are_rejected():
    assert normalize_label("Z9") is None
    assert normalize_label("G1") is None, "поле 6x6 — строки только A..F"
    assert normalize_label("A9") is None
    assert normalize_label("A0") is None


def test_larger_field_accepts_more_cells():
    assert normalize_label("H8", rows=8, cols=8) == "H8"


def test_vote_picks_the_majority():
    vote = vote_labels([("A3", 0.8), ("A3", 0.85), ("B5", 0.9)])
    assert vote.label == "A3"
    assert vote.votes == 2 and vote.total == 3
    assert 0.0 < vote.confidence <= 1.0


def test_vote_ignores_unreadable_frames():
    vote = vote_labels([("", 0.0), ("мусор", 0.1), ("B5", 0.9), ("B5", 0.8)])
    assert vote.label == "B5"
    assert vote.votes == 2 and vote.total == 4


def test_vote_confidence_drops_when_frames_disagree():
    agreed = vote_labels([("A3", 0.9), ("A3", 0.9), ("A3", 0.9)])
    split = vote_labels([("A3", 0.9), ("B5", 0.9), ("C2", 0.9)])
    assert agreed.confidence > split.confidence


def test_vote_without_readings():
    vote = vote_labels([])
    assert vote.label is None
    assert vote.confidence == 0.0
