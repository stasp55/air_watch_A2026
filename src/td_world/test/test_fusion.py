"""Тесты политики доверия к детекциям."""

from td_world.fusion import BeliefStore, Fact, KindPolicy
from td_world.grid import CellRef


def fact(kind="TARGET", cell="A1", conf=0.9, stamp=1.0, source="test", payload=""):
    return Fact(
        kind=kind,
        cell=CellRef.parse(cell) if cell else None,
        confidence=conf,
        source=source,
        stamp=stamp,
        payload=payload,
    )


def test_low_confidence_is_rejected():
    store = BeliefStore()
    assert store.update(fact(kind="ENEMY", conf=0.1)) is False
    assert store.get("ENEMY", now=1.0) is None
    assert "ниже порога" in store.drain_rejections()[0]


def test_target_needs_two_confirmations():
    store = BeliefStore()
    assert store.update(fact(stamp=1.0)) is False
    assert store.update(fact(stamp=2.0)) is True
    assert store.get("TARGET", now=2.0).cell.name == "A1"


def test_target_confirmations_must_be_consistent():
    store = BeliefStore()
    store.update(fact(cell="A1", stamp=1.0))
    store.update(fact(cell="E2", stamp=2.0))  # другая клетка — счётчик сбрасывается
    assert store.get("TARGET", now=2.0) is None
    store.update(fact(cell="E2", stamp=3.0))
    assert store.get("TARGET", now=3.0).cell.name == "E2"


def test_latched_target_is_not_flipped_by_weaker_detection():
    store = BeliefStore()
    store.update(fact(cell="A1", conf=0.9, stamp=1.0))
    store.update(fact(cell="A1", conf=0.9, stamp=2.0))
    store.update(fact(cell="E2", conf=0.95, stamp=3.0))
    store.update(fact(cell="E2", conf=0.95, stamp=4.0))
    assert store.get("TARGET", now=4.0).cell.name == "A1"


def test_latched_target_is_overridden_by_much_stronger_detection():
    store = BeliefStore()
    store.update(fact(cell="A1", conf=0.65, stamp=1.0))
    store.update(fact(cell="A1", conf=0.65, stamp=2.0))
    store.update(fact(cell="E2", conf=0.99, stamp=3.0))
    store.update(fact(cell="E2", conf=0.99, stamp=4.0))
    assert store.get("TARGET", now=4.0).cell.name == "E2"


def test_enemy_position_expires():
    store = BeliefStore()
    assert store.update(fact(kind="ENEMY", cell="C3", conf=0.8, stamp=10.0)) is True
    assert store.get("ENEMY", now=11.0).cell.name == "C3"
    assert store.get("ENEMY", now=20.0) is None, "старая позиция противника должна протухнуть"
    assert store.get_stale("ENEMY").cell.name == "C3"


def test_enemy_position_always_takes_latest():
    store = BeliefStore()
    store.update(fact(kind="ENEMY", cell="C3", conf=0.9, stamp=10.0))
    store.update(fact(kind="ENEMY", cell="C4", conf=0.5, stamp=10.5))
    assert store.get("ENEMY", now=10.6).cell.name == "C4"


def test_out_of_order_detection_is_ignored():
    store = BeliefStore()
    store.update(fact(kind="ENEMY", cell="C4", conf=0.9, stamp=10.0))
    store.update(fact(kind="ENEMY", cell="C3", conf=0.9, stamp=9.0))
    assert store.get("ENEMY", now=10.0).cell.name == "C4"


def test_detection_outside_candidate_list_is_rejected():
    store = BeliefStore(allowed_cells={"TARGET": [CellRef.parse("A1"), CellRef.parse("E2")]})
    assert store.update(fact(cell="C3", stamp=1.0)) is False
    assert "не входит в список допустимых" in store.drain_rejections()[0]


def test_forget_clears_latched_fact():
    store = BeliefStore(policies={"FINISH_LABEL": KindPolicy(min_confidence=0.5, latch=True)})
    store.update(fact(kind="FINISH_LABEL", cell="B5", conf=0.8, stamp=1.0, payload="B5"))
    assert store.get("FINISH_LABEL", now=1.0) is not None
    store.forget("FINISH_LABEL")
    assert store.get("FINISH_LABEL", now=1.0) is None
