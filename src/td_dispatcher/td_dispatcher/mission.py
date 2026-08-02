"""Логика миссии — конечный автомат без ROS.

Это «мозг» системы: он решает, что делать дальше, опираясь только на картину
мира и на результаты уже выданных команд. Никаких подписок, таймеров и
сетевых вызовов здесь нет, поэтому весь сценарий, включая ветки отказов,
проверяется юнит-тестами за доли секунды.

Сценарий (TASK.txt)::

    INIT -> взлёт дрона
         -> SCAN: дрон осматривает клетки-кандидаты, ищет игрушку
         -> TARGET_FOUND: параллельно
              * ровер едет к найденной клетке,
              * дрон переключается на сопровождение ровера противника
         -> READ_LABEL: ровер читает бумажку с финишной клеткой
         -> TO_FINISH: ровер едет на финиш (обходя противника)
         -> DONE

Каждый переход сопровождается человекочитаемым объяснением: оно идёт и в лог
решений, и в поле `rationale` состояния мира.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

__all__ = [
    "Phase",
    "Snapshot",
    "Command",
    "Outcome",
    "MissionConfig",
    "MissionPlanner",
]


class Phase(str, Enum):
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    SCAN = "SCAN"
    TARGET_FOUND = "TARGET_FOUND"
    READ_LABEL = "READ_LABEL"
    TO_FINISH = "TO_FINISH"
    DONE = "DONE"
    FAILED = "FAILED"


ROVER = "rover"
DRONE = "drone"


@dataclass(frozen=True)
class Snapshot:
    """Срез картины мира на момент принятия решения."""

    now: float = 0.0
    target: str | None = None       # клетка с игрушкой
    finish: str | None = None       # прочитанная финишная клетка
    enemy: str | None = None        # клетка противника
    enemy_age: float = float("inf")
    rover: str | None = None        # где ровер
    drone: str | None = None        # где дрон


@dataclass(frozen=True)
class Command:
    """Одно поручение конкретному агенту."""

    agent: str
    skill: str
    args: dict
    reason: str

    def key(self) -> str:
        return f"{self.agent}:{self.skill}"


@dataclass(frozen=True)
class Outcome:
    """Чем закончилось поручение."""

    skill: str
    success: bool
    message: str = ""


@dataclass
class MissionConfig:
    """Настройки поведения миссии."""

    candidates: list[str] = field(default_factory=list)
    scan_retries: int = 1
    label_retries: int = 2
    move_retries: int = 2
    escort_enemy: bool = True
    land_when_done: bool = True
    takeoff_first: bool = True
    enemy_data_max_age_s: float = 3.0


class MissionPlanner:
    """Конечный автомат миссии.

    Использование::

        planner = MissionPlanner(MissionConfig(candidates=["A1", "E2"]))
        commands = planner.step(snapshot)      # что выдать агентам сейчас
        planner.on_outcome("rover", outcome)   # чем закончилось поручение
    """

    def __init__(self, config: MissionConfig | None = None) -> None:
        self.config = config or MissionConfig()
        self.phase = Phase.INIT
        self._pending: dict[str, Command] = {}
        self._retries: dict[str, int] = {}
        self._rationale: list[str] = []
        self._escorting = False
        self._landing = False
        self._failure = ""

    # --- публичный интерфейс -----------------------------------------------

    @property
    def rationale(self) -> list[str]:
        """Накопленные объяснения решений (последние 20)."""
        return list(self._rationale)

    @property
    def failure_reason(self) -> str:
        return self._failure

    def busy(self, agent: str) -> bool:
        return agent in self._pending

    def pending_command(self, agent: str) -> Command | None:
        return self._pending.get(agent)

    def step(self, snap: Snapshot) -> list[Command]:
        """Выдать команды, уместные в текущем состоянии.

        Возвращает только новые поручения; агент, у которого поручение уже
        выполняется, новых команд не получает.
        """
        if self.phase is Phase.FAILED:
            return []
        if self.phase is Phase.DONE:
            return self._plan_done()

        commands: list[Command] = []
        if self.phase is Phase.INIT:
            commands += self._plan_init()
        elif self.phase is Phase.SCAN:
            commands += self._plan_scan()
        elif self.phase is Phase.TARGET_FOUND:
            commands += self._plan_to_target(snap)
        elif self.phase is Phase.READ_LABEL:
            commands += self._plan_read_label(snap)
        elif self.phase is Phase.TO_FINISH:
            commands += self._plan_to_finish(snap)

        # Сопровождение противника ведётся параллельно основному сценарию.
        if self._should_escort():
            commands.append(self._escort_command())

        for command in commands:
            self._pending[command.agent] = command
        return commands

    def on_outcome(self, agent: str, outcome: Outcome) -> None:
        """Учесть результат поручения и при необходимости сменить фазу."""
        self._pending.pop(agent, None)
        handler = {
            "takeoff": self._after_takeoff,
            "find_object": self._after_find,
            "goto_cell": self._after_goto,
            "read_label": self._after_read,
            "track_target": self._after_track,
            "land": lambda o: None,
        }.get(outcome.skill)
        if handler is None:
            self._note(f"неизвестный навык {outcome.skill} — результат проигнорирован")
            return
        handler(outcome)

    def abort(self, reason: str) -> None:
        """Принудительно завершить миссию (оператор, критическая ошибка)."""
        self._fail(reason)

    # --- планирование по фазам ---------------------------------------------

    def _plan_init(self) -> list[Command]:
        if not self.config.takeoff_first:
            self._advance(Phase.SCAN, "взлёт отключён конфигурацией")
            return self._plan_scan()
        if self.busy(DRONE):
            return []
        self._advance(Phase.TAKEOFF, "поднимаем дрон перед поиском")
        return [Command(DRONE, "takeoff", {}, "начало миссии")]

    def _plan_scan(self) -> list[Command]:
        if self.busy(DRONE):
            return []
        candidates = list(self.config.candidates)
        return [
            Command(
                DRONE, "find_object",
                {"kind": "TARGET", "candidates": candidates},
                f"осматриваем кандидатов: {', '.join(candidates) or 'всё поле'}",
            )
        ]

    def _plan_to_target(self, snap: Snapshot) -> list[Command]:
        if self.busy(ROVER) or snap.target is None:
            return []
        if snap.rover == snap.target:
            self._advance(Phase.READ_LABEL, f"ровер уже в клетке {snap.target}")
            return self._plan_read_label(snap)
        return [
            Command(
                ROVER, "goto_cell",
                {"cell": snap.target, "face_wall": True},
                f"едем к игрушке в {snap.target}",
            )
        ]

    def _plan_read_label(self, snap: Snapshot) -> list[Command]:
        if self.busy(ROVER):
            return []
        attempt = self._retries.get("read_label", 0)
        return [
            Command(
                ROVER, "read_label", {"attempt": attempt},
                "читаем бумажку с номером финишной клетки"
                if attempt == 0
                else f"повторное чтение, попытка {attempt + 1}",
            )
        ]

    def _plan_to_finish(self, snap: Snapshot) -> list[Command]:
        if self.busy(ROVER) or snap.finish is None:
            return []
        if snap.rover == snap.finish:
            self._finish("ровер на финише")
            return []
        return [
            Command(
                ROVER, "goto_cell",
                {"cell": snap.finish, "face_wall": False},
                f"едем на финиш {snap.finish}",
            )
        ]

    def _plan_done(self) -> list[Command]:
        """После выполнения миссии дрон садится — сам он этого не сделает."""
        if not self.config.land_when_done or self._landing or self.busy(DRONE):
            return []
        self._landing = True
        command = Command(DRONE, "land", {}, "миссия выполнена, сажаем дрон")
        self._pending[DRONE] = command
        return [command]

    # --- сопровождение противника ------------------------------------------

    def _should_escort(self) -> bool:
        return (
            self.config.escort_enemy
            and not self._escorting
            and not self.busy(DRONE)
            and self.phase in (Phase.TARGET_FOUND, Phase.READ_LABEL, Phase.TO_FINISH)
        )

    def _escort_command(self) -> Command:
        self._escorting = True
        return Command(
            DRONE, "track_target", {"kind": "ENEMY"},
            "дрон переключается на слежение за ровером противника",
        )

    # --- обработка результатов ---------------------------------------------

    def _after_takeoff(self, outcome: Outcome) -> None:
        if outcome.success:
            self._advance(Phase.SCAN, "дрон в воздухе, начинаем поиск")
        else:
            self._fail(f"взлёт не удался: {outcome.message}")

    def _after_find(self, outcome: Outcome) -> None:
        if outcome.success:
            self._advance(Phase.TARGET_FOUND, "игрушка найдена, направляем ровер")
            return
        if self._retry("find_object", self.config.scan_retries):
            self._note(f"поиск не дал результата ({outcome.message}), повторяем облёт")
            return
        self._fail(f"игрушка не найдена: {outcome.message}")

    def _after_goto(self, outcome: Outcome) -> None:
        if outcome.success:
            if self.phase is Phase.TARGET_FOUND:
                self._advance(Phase.READ_LABEL, "ровер у игрушки, читаем метку")
            elif self.phase is Phase.TO_FINISH:
                self._finish("ровер прибыл на финиш")
            return
        if self._retry("goto_cell", self.config.move_retries):
            self._note(f"не доехали ({outcome.message}), пробуем ещё раз")
            return
        self._fail(f"ровер не смог доехать: {outcome.message}")

    def _after_read(self, outcome: Outcome) -> None:
        if outcome.success:
            self._advance(Phase.TO_FINISH, "финишная клетка прочитана")
            return
        if self._retry("read_label", self.config.label_retries):
            self._note(
                f"метка не прочитана ({outcome.message}); "
                "подъезжаем заново и снимаем под другим ракурсом"
            )
            return
        self._fail(f"не удалось прочитать финишную клетку: {outcome.message}")

    def _after_track(self, outcome: Outcome) -> None:
        # Сопровождение — фоновая задача: её падение не срывает миссию,
        # но и молча терять её нельзя.
        self._escorting = False
        if not outcome.success:
            self._note(f"сопровождение противника прервано: {outcome.message}")

    # --- служебное ---------------------------------------------------------

    def _retry(self, skill: str, limit: int) -> bool:
        used = self._retries.get(skill, 0)
        if used >= limit:
            return False
        self._retries[skill] = used + 1
        return True

    def _advance(self, phase: Phase, reason: str) -> None:
        if phase is not self.phase:
            self._note(f"{self.phase.value} -> {phase.value}: {reason}")
            self.phase = phase

    def _finish(self, reason: str) -> None:
        self._advance(Phase.DONE, reason)
        self._pending.clear()

    def _fail(self, reason: str) -> None:
        self._failure = reason
        self._advance(Phase.FAILED, reason)
        self._pending.clear()

    def _note(self, text: str) -> None:
        self._rationale.append(text)
        del self._rationale[:-20]
