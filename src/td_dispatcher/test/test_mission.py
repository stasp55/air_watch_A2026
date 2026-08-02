"""Тесты сценария миссии, включая ветки отказов."""

import pytest

from td_dispatcher.mission import (
    Command,
    MissionConfig,
    MissionPlanner,
    Outcome,
    Phase,
    Snapshot,
)


def planner(**kwargs) -> MissionPlanner:
    config = MissionConfig(candidates=["A1", "E2"], **kwargs)
    return MissionPlanner(config)


def only(commands: list[Command]) -> Command:
    assert len(commands) == 1, f"ожидалась одна команда, получено {commands}"
    return commands[0]


def run_to_target_found(mp: MissionPlanner) -> None:
    only(mp.step(Snapshot()))
    mp.on_outcome("drone", Outcome("takeoff", True))
    only(mp.step(Snapshot()))
    mp.on_outcome("drone", Outcome("find_object", True))


# --- happy path ------------------------------------------------------------


def test_mission_starts_with_takeoff():
    mp = planner()
    command = only(mp.step(Snapshot()))
    assert command.agent == "drone" and command.skill == "takeoff"
    assert mp.phase is Phase.TAKEOFF


def test_after_takeoff_drone_searches_candidates():
    mp = planner()
    mp.step(Snapshot())
    mp.on_outcome("drone", Outcome("takeoff", True))
    command = only(mp.step(Snapshot()))
    assert command.skill == "find_object"
    assert command.args["candidates"] == ["A1", "E2"]
    assert mp.phase is Phase.SCAN


def test_target_found_sends_rover_and_starts_escort():
    mp = planner()
    run_to_target_found(mp)
    commands = mp.step(Snapshot(target="E2", rover="F1"))
    skills = {(c.agent, c.skill) for c in commands}
    assert ("rover", "goto_cell") in skills
    assert ("drone", "track_target") in skills, "дрон должен переключиться на противника"
    assert mp.phase is Phase.TARGET_FOUND


def test_full_happy_path():
    mp = planner()
    run_to_target_found(mp)
    mp.step(Snapshot(target="E2", rover="F1"))
    mp.on_outcome("rover", Outcome("goto_cell", True))
    assert mp.phase is Phase.READ_LABEL

    command = only([c for c in mp.step(Snapshot(target="E2", rover="E2")) if c.agent == "rover"])
    assert command.skill == "read_label"
    mp.on_outcome("rover", Outcome("read_label", True))
    assert mp.phase is Phase.TO_FINISH

    command = only([c for c in mp.step(Snapshot(target="E2", finish="B5", rover="E2"))
                    if c.agent == "rover"])
    assert command.args["cell"] == "B5"
    mp.on_outcome("rover", Outcome("goto_cell", True))
    assert mp.phase is Phase.DONE


def test_drone_lands_after_the_mission_and_only_once():
    mp = planner()
    mp.abort("тест")  # быстрый способ убедиться, что после провала посадка не навязывается
    assert mp.step(Snapshot()) == []

    mp = planner()
    run_to_target_found(mp)
    mp.step(Snapshot(target="E2", rover="F1"))
    mp.on_outcome("rover", Outcome("goto_cell", True))
    mp.on_outcome("drone", Outcome("track_target", True))
    mp.step(Snapshot(target="E2", rover="E2"))
    mp.on_outcome("rover", Outcome("read_label", True))
    mp.step(Snapshot(target="E2", finish="B5", rover="E2"))
    mp.on_outcome("rover", Outcome("goto_cell", True))
    assert mp.phase is Phase.DONE

    command = only(mp.step(Snapshot()))
    assert command.agent == "drone" and command.skill == "land"
    assert mp.step(Snapshot()) == [], "повторная посадка не выдаётся"


def test_landing_can_be_disabled():
    mp = planner(land_when_done=False)
    mp.abort("не важно")
    mp.phase = Phase.DONE
    assert mp.step(Snapshot()) == []


def test_rover_already_on_target_skips_the_drive():
    mp = planner()
    run_to_target_found(mp)
    commands = mp.step(Snapshot(target="E2", rover="E2"))
    rover_commands = [c for c in commands if c.agent == "rover"]
    assert only(rover_commands).skill == "read_label"
    assert mp.phase is Phase.READ_LABEL


# --- отказы ----------------------------------------------------------------


def test_failed_takeoff_fails_the_mission():
    mp = planner()
    mp.step(Snapshot())
    mp.on_outcome("drone", Outcome("takeoff", False, "нет арма"))
    assert mp.phase is Phase.FAILED
    assert "нет арма" in mp.failure_reason


def test_search_is_retried_before_giving_up():
    mp = planner(scan_retries=1)
    mp.step(Snapshot())
    mp.on_outcome("drone", Outcome("takeoff", True))
    mp.step(Snapshot())

    mp.on_outcome("drone", Outcome("find_object", False, "пусто"))
    assert mp.phase is Phase.SCAN, "после первой неудачи миссия продолжается"
    assert only(mp.step(Snapshot())).skill == "find_object"

    mp.on_outcome("drone", Outcome("find_object", False, "снова пусто"))
    assert mp.phase is Phase.FAILED


def test_unreadable_label_is_retried_then_fails():
    mp = planner(label_retries=2)
    run_to_target_found(mp)
    mp.step(Snapshot(target="E2", rover="F1"))
    mp.on_outcome("rover", Outcome("goto_cell", True))

    for attempt in range(2):
        rover_commands = [c for c in mp.step(Snapshot(target="E2", rover="E2"))
                          if c.agent == "rover"]
        assert only(rover_commands).args["attempt"] == attempt
        mp.on_outcome("rover", Outcome("read_label", False, "размыто"))
        assert mp.phase is Phase.READ_LABEL

    rover_commands = [c for c in mp.step(Snapshot(target="E2", rover="E2"))
                      if c.agent == "rover"]
    assert only(rover_commands).args["attempt"] == 2
    mp.on_outcome("rover", Outcome("read_label", False, "размыто"))
    assert mp.phase is Phase.FAILED


def test_blocked_drive_is_retried():
    mp = planner(move_retries=1)
    run_to_target_found(mp)
    mp.step(Snapshot(target="E2", rover="F1"))
    mp.on_outcome("rover", Outcome("goto_cell", False, "клетка занята противником"))
    assert mp.phase is Phase.TARGET_FOUND
    rover_commands = [c for c in mp.step(Snapshot(target="E2", rover="F1")) if c.agent == "rover"]
    assert only(rover_commands).skill == "goto_cell"

    mp.on_outcome("rover", Outcome("goto_cell", False, "снова занята"))
    assert mp.phase is Phase.FAILED


def test_escort_failure_does_not_break_the_mission():
    mp = planner()
    run_to_target_found(mp)
    mp.step(Snapshot(target="E2", rover="F1"))
    mp.on_outcome("drone", Outcome("track_target", False, "метка потеряна"))
    assert mp.phase is Phase.TARGET_FOUND
    assert any("сопровождение" in line for line in mp.rationale)


# --- дисциплина выдачи команд ---------------------------------------------


def test_agent_gets_no_second_command_while_busy():
    mp = planner()
    run_to_target_found(mp)
    first = mp.step(Snapshot(target="E2", rover="F1"))
    assert first, "первый шаг должен выдать команды"
    assert mp.step(Snapshot(target="E2", rover="F1")) == []
    assert mp.busy("rover") and mp.busy("drone")


def test_escort_is_started_only_once():
    mp = planner()
    run_to_target_found(mp)
    mp.step(Snapshot(target="E2", rover="F1"))
    mp.on_outcome("rover", Outcome("goto_cell", True))
    later = mp.step(Snapshot(target="E2", rover="E2"))
    assert [c for c in later if c.skill == "track_target"] == []


def test_escort_can_be_disabled():
    mp = planner(escort_enemy=False)
    run_to_target_found(mp)
    commands = mp.step(Snapshot(target="E2", rover="F1"))
    assert all(c.skill != "track_target" for c in commands)


def test_takeoff_can_be_skipped():
    mp = planner(takeoff_first=False)
    command = only(mp.step(Snapshot()))
    assert command.skill == "find_object"
    assert mp.phase is Phase.SCAN


def test_abort_stops_everything():
    mp = planner()
    run_to_target_found(mp)
    mp.abort("оператор нажал стоп")
    assert mp.phase is Phase.FAILED
    assert mp.step(Snapshot(target="E2")) == []


def test_unknown_skill_result_is_ignored_safely():
    mp = planner()
    mp.on_outcome("rover", Outcome("fly_to_the_moon", True))
    assert mp.phase is Phase.INIT
    assert any("неизвестный навык" in line for line in mp.rationale)


def test_rationale_explains_transitions():
    mp = planner()
    run_to_target_found(mp)
    joined = " | ".join(mp.rationale)
    assert "TAKEOFF" in joined and "SCAN" in joined
    assert "TARGET_FOUND" in joined
