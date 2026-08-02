"""Техническое зрение ровера: чтение метки финишной клетки."""

from .label_parse import LabelVote, normalize_label, vote_labels

__all__ = ["LabelVote", "normalize_label", "vote_labels"]
