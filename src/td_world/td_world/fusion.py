"""Слияние детекций в непротиворечивую картину мира.

Здесь нет ROS и нет зрения — только политика доверия: что принять, что
отбросить, что считать протухшим. Благодаря этому поведение системы при
ложных срабатываниях воспроизводится в тестах, а не «ловится» на площадке.

Две принципиально разные политики:

* **latched-факты** (цель, финишная клетка) — устанавливаются один раз и не
  скачут. Перебить такой факт может только заметно более уверенная детекция.
* **live-факты** (противник, свои позиции) — всегда берётся самая свежая
  оценка, а старая быстро протухает: ехать по устаревшей позиции противника
  опаснее, чем признать, что мы её не знаем.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

from .grid import CellRef

__all__ = ["Fact", "KindPolicy", "BeliefStore", "DEFAULT_POLICIES"]


@dataclass(frozen=True)
class Fact:
    """Единичное утверждение о мире, привязанное ко времени."""

    kind: str
    cell: CellRef | None
    confidence: float
    source: str
    stamp: float
    payload: str = ""

    def age(self, now: float) -> float:
        return max(0.0, now - self.stamp)


@dataclass(frozen=True)
class KindPolicy:
    """Правила приёма фактов одного типа."""

    min_confidence: float = 0.5
    ttl_s: float = 0.0  # 0 = не протухает
    latch: bool = False
    # На сколько новая детекция должна быть увереннее закреплённой, чтобы её
    # перебить. Защищает от переобнаружения цели в соседней клетке.
    override_margin: float = 0.15
    # Сколько подряд идущих согласованных детекций нужно, чтобы факт закрепился.
    confirmations: int = 1


DEFAULT_POLICIES: dict[str, KindPolicy] = {
    "TARGET": KindPolicy(min_confidence=0.6, ttl_s=0.0, latch=True, confirmations=2),
    "FINISH_LABEL": KindPolicy(min_confidence=0.6, ttl_s=0.0, latch=True, confirmations=2),
    "ENEMY": KindPolicy(min_confidence=0.4, ttl_s=2.5, latch=False),
    "SELF": KindPolicy(min_confidence=0.0, ttl_s=1.5, latch=False),
}


class BeliefStore:
    """Хранилище фактов с TTL, порогами доверия и подтверждениями."""

    def __init__(
        self,
        policies: dict[str, KindPolicy] | None = None,
        allowed_cells: dict[str, Iterable[CellRef]] | None = None,
    ) -> None:
        self._policies = dict(policies or DEFAULT_POLICIES)
        self._facts: dict[str, Fact] = {}
        self._pending: dict[str, tuple[CellRef | None, str, int]] = {}
        self._rejections: list[str] = []
        self._allowed: dict[str, set[CellRef]] = {
            kind: set(cells) for kind, cells in (allowed_cells or {}).items()
        }

    # --- запись ------------------------------------------------------------

    def update(self, fact: Fact) -> bool:
        """Принять или отклонить детекцию. Возвращает True, если принята."""
        policy = self._policies.get(fact.kind, KindPolicy())

        if fact.confidence < policy.min_confidence:
            self._reject(fact, f"уверенность {fact.confidence:.2f} ниже порога "
                               f"{policy.min_confidence:.2f}")
            return False

        allowed = self._allowed.get(fact.kind)
        if allowed and fact.cell is not None and fact.cell not in allowed:
            self._reject(fact, f"клетка {fact.cell.name} не входит в список допустимых")
            return False

        current = self._facts.get(fact.kind)
        if policy.latch and current is not None:
            same_cell = current.cell == fact.cell and current.payload == fact.payload
            if not same_cell and fact.confidence < current.confidence + policy.override_margin:
                self._reject(
                    fact,
                    f"закреплено {_describe(current)}, новая детекция недостаточно "
                    f"увереннее ({fact.confidence:.2f} против {current.confidence:.2f})",
                )
                return False

        if policy.confirmations > 1 and not self._confirm(fact, policy):
            return False

        if current is not None and fact.stamp < current.stamp:
            self._reject(fact, "детекция старше уже принятой")
            return False

        self._facts[fact.kind] = fact
        return True

    def _confirm(self, fact: Fact, policy: KindPolicy) -> bool:
        """Накопить подряд идущие согласованные детекции."""
        key = (fact.cell, fact.payload)
        prev = self._pending.get(fact.kind)
        count = prev[2] + 1 if prev is not None and (prev[0], prev[1]) == key else 1
        self._pending[fact.kind] = (fact.cell, fact.payload, count)
        if count < policy.confirmations:
            self._reject(
                fact,
                f"нужно подтверждений {policy.confirmations}, получено {count}",
            )
            return False
        return True

    def _reject(self, fact: Fact, reason: str) -> None:
        self._rejections.append(f"{fact.kind} от {fact.source}: отклонено — {reason}")
        del self._rejections[:-20]  # держим только хвост, лог не должен расти вечно

    # --- чтение ------------------------------------------------------------

    def get(self, kind: str, now: float) -> Fact | None:
        """Актуальный факт или None, если его нет либо он протух."""
        fact = self._facts.get(kind)
        if fact is None:
            return None
        policy = self._policies.get(kind, KindPolicy())
        if policy.ttl_s > 0.0 and fact.age(now) > policy.ttl_s:
            return None
        return fact

    def get_stale(self, kind: str) -> Fact | None:
        """Последний известный факт, даже протухший (для диагностики)."""
        return self._facts.get(kind)

    def age(self, kind: str, now: float) -> float:
        fact = self._facts.get(kind)
        return float("inf") if fact is None else fact.age(now)

    def drain_rejections(self) -> list[str]:
        """Забрать и очистить накопленные причины отказов (идут в лог решений)."""
        out = list(self._rejections)
        self._rejections.clear()
        return out

    def forget(self, kind: str) -> None:
        """Сбросить факт — например, по команде оператора при явной ошибке."""
        self._facts.pop(kind, None)
        self._pending.pop(kind, None)

    def set_allowed_cells(self, kind: str, cells: Iterable[CellRef]) -> None:
        self._allowed[kind] = set(cells)

    def snapshot(self, now: float) -> dict[str, Fact]:
        """Все живые факты на текущий момент."""
        return {
            kind: fact
            for kind in list(self._facts)
            if (fact := self.get(kind, now)) is not None
        }

    def with_confidence(self, kind: str, value: float) -> Fact | None:
        """Копия факта с изменённой уверенностью (используется в тестах)."""
        fact = self._facts.get(kind)
        return None if fact is None else replace(fact, confidence=value)


def _describe(fact: Fact) -> str:
    cell = fact.cell.name if fact.cell else "?"
    return f"{cell}{'/' + fact.payload if fact.payload else ''}"
