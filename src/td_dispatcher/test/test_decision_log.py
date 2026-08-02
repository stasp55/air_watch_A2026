"""Тесты журнала решений."""

import json

from td_dispatcher.decision_log import DecisionLog


def test_records_are_written_to_file(tmp_path):
    path = tmp_path / "logs" / "mission.jsonl"
    log = DecisionLog(path)
    log.write("command", agent="rover-01", skill="goto_cell", reason="едем к цели")
    log.write("outcome", agent="rover-01", skill="goto_cell", success=True)

    lines = path.read_text(encoding="utf-8").strip().split("\n")
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["event"] == "command"
    assert first["reason"] == "едем к цели"
    assert "ts" in first


def test_works_without_a_file():
    log = DecisionLog(None)
    log.write("phase", phase="SCAN", reason="дрон в воздухе")
    assert len(log.records) == 1


def test_memory_tail_is_bounded():
    log = DecisionLog(None, keep_last=5)
    for index in range(20):
        log.write("tick", index=index)
    assert len(log.records) == 5
    assert log.records[-1]["index"] == 19


def test_summary_is_human_readable():
    log = DecisionLog(None)
    log.write("command", agent="drone-01", skill="find_object", reason="осматриваем A1, E2")
    log.write("outcome", agent="drone-01", skill="find_object", success=False)
    log.write("phase", phase="FAILED", reason="игрушка не найдена")

    summary = log.summary()
    assert "drone-01: find_object — осматриваем A1, E2" in summary
    assert "drone-01: find_object — неудача" in summary
    assert "фаза FAILED: игрушка не найдена" in summary


def test_write_failure_does_not_raise(tmp_path):
    """Недоступный журнал не должен срывать миссию."""
    path = tmp_path / "mission.jsonl"
    log = DecisionLog(path)
    path.parent.chmod(0o500)
    try:
        log.write("command", agent="rover-01", skill="goto_cell")
    finally:
        path.parent.chmod(0o700)
    assert len(log.records) == 1
