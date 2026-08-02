"""Тесты загрузки описания поля."""

import textwrap

import pytest

from td_world.field import load_field


def write(tmp_path, text):
    path = tmp_path / "field.yaml"
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return path


def test_minimal_config(tmp_path):
    path = write(tmp_path, """
        field:
          rows: 6
          cols: 6
          cell_size_m: 0.8
          start_cell: F1
    """)
    model = load_field(path)
    assert model.grid.rows == 6
    assert model.start_cell.name == "F1"
    assert model.target_candidates == []


def test_full_config_is_parsed(tmp_path):
    path = write(tmp_path, """
        field:
          rows: 6
          cols: 6
          cell_size_m: 0.8
          start_cell: F1
          target_candidates: [A1, E2]
          label_cells: [A1, E2]
          field_to_map: {x: 0.25, y: -1.5, yaw: 0.1}
          markers:
            own_ids: [7]
            enemy_ids: [11, 12]
            size_m: 0.12
    """)
    model = load_field(path)
    assert [c.name for c in model.target_candidates] == ["A1", "E2"]
    assert model.enemy_marker_ids == [11, 12]
    assert model.own_marker_ids == [7]
    assert model.marker_size_m == pytest.approx(0.12)
    assert model.field_to_map.tx == pytest.approx(0.25)


def test_start_cell_is_required(tmp_path):
    path = write(tmp_path, """
        field:
          rows: 6
          cols: 6
    """)
    with pytest.raises(ValueError, match="start_cell"):
        load_field(path)


def test_bad_cell_name_is_reported_with_context(tmp_path):
    path = write(tmp_path, """
        field:
          start_cell: F1
          target_candidates: [A1, Z9]
    """)
    with pytest.raises(ValueError, match="target_candidates"):
        load_field(path)


def test_map_conversion_uses_calibration(tmp_path):
    path = write(tmp_path, """
        field:
          start_cell: F1
          field_to_map: [1.0, 2.0, 0.0]
    """)
    model = load_field(path)
    assert model.center_in_map(model.cell("F1")) == pytest.approx((1.4, 2.4))
    assert model.cell_from_map(1.4, 2.4).name == "F1"
