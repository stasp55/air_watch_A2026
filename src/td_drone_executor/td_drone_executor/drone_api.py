"""Адаптер к полётному стеку дрона.

Вся система общается с дроном через узкий интерфейс `DroneApi`: взлететь,
лететь в точку, сесть, отдать телеметрию. За ним стоит либо штатная
библиотека `sverk_interfaces`, либо заглушка для отладки логики без железа.

Такая изоляция избавляет от единственной по-настоящему опасной ошибки —
случайного взлёта при отладке наземной логики: заглушка выбирается одним
параметром `backend: mock`.
"""

from __future__ import annotations

import abc
import threading

__all__ = ["DroneApi", "MockDrone", "SverkDrone", "create_drone_api"]


class DroneApi(abc.ABC):
    """Минимальный контракт управления дроном."""

    @abc.abstractmethod
    def takeoff(self, altitude: float, speed: float) -> tuple[bool, str]: ...

    @abc.abstractmethod
    def go_to(self, x: float, y: float, z: float, yaw: float | None,
              speed: float, wait: bool) -> tuple[bool, str]:
        """Лететь в точку локального фрейма дрона."""

    @abc.abstractmethod
    def land(self, timeout: float) -> tuple[bool, str]: ...

    @abc.abstractmethod
    def position(self) -> tuple[float, float, float] | None:
        """Текущая позиция в локальном фрейме дрона, None если неизвестна."""

    def emergency_stop(self) -> None:
        """Мгновенно прекратить манёвр и зависнуть."""

    def close(self) -> None:
        """Освободить ресурсы."""


class MockDrone(DroneApi):
    """Заглушка: телепортируется в цель, ничего не поднимая в воздух."""

    def __init__(self, start: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self._position = start
        self._airborne = False
        self._lock = threading.Lock()
        self.log: list[str] = []

    def takeoff(self, altitude: float, speed: float) -> tuple[bool, str]:
        with self._lock:
            x, y, _ = self._position
            self._position = (x, y, altitude)
            self._airborne = True
        self.log.append(f"takeoff {altitude:.2f}")
        return True, "mock: взлёт выполнен"

    def go_to(self, x, y, z, yaw, speed, wait) -> tuple[bool, str]:
        if not self._airborne:
            return False, "mock: дрон не в воздухе"
        with self._lock:
            self._position = (x, y, z)
        self.log.append(f"go_to {x:.2f} {y:.2f} {z:.2f}")
        return True, "mock: точка достигнута"

    def land(self, timeout: float) -> tuple[bool, str]:
        with self._lock:
            x, y, _ = self._position
            self._position = (x, y, 0.0)
            self._airborne = False
        self.log.append("land")
        return True, "mock: посадка выполнена"

    def position(self):
        with self._lock:
            return self._position

    def emergency_stop(self) -> None:
        self.log.append("emergency_stop")


class SverkDrone(DroneApi):
    """Обёртка над `sverk_interfaces` — штатным API стека sverk-ros2."""

    def __init__(self, node_name: str = "td_drone_executor", frame_id: str = "map") -> None:
        try:
            import sverk_interfaces
        except ImportError as exc:  # pragma: no cover - зависит от борта
            raise RuntimeError(
                "Не найден sverk_interfaces. Запускайте эту ноду на дроне со "
                "стеком sverk-ros2 либо выберите backend: mock"
            ) from exc
        self._drone = sverk_interfaces.init(Nodename=node_name)
        self._frame_id = frame_id

    def takeoff(self, altitude: float, speed: float) -> tuple[bool, str]:
        response = self._drone.control.navigate(
            x=0.0, y=0.0, z=float(altitude), frame_id="body",
            speed=float(speed), auto_arm=True,
        )
        return bool(response.success), str(response.message)

    def go_to(self, x, y, z, yaw, speed, wait) -> tuple[bool, str]:
        kwargs = dict(x=float(x), y=float(y), z=float(z),
                      frame_id=self._frame_id, speed=float(speed))
        if yaw is not None:
            kwargs["yaw"] = float(yaw)
        if wait:
            response = self._drone.control.navigate_to(**kwargs)
            return bool(response.success), str(response.message)
        response = self._drone.control.navigate(**kwargs)
        return bool(response.success), str(response.message)

    def land(self, timeout: float) -> tuple[bool, str]:
        response = self._drone.control.land(timeout=float(timeout))
        return bool(response.success), str(response.message)

    def position(self):
        try:
            telemetry = self._drone.control.get_telemetry()
        except Exception:  # noqa: BLE001 - телеметрия может быть недоступна
            return None
        return (float(telemetry.x), float(telemetry.y), float(telemetry.z))

    def emergency_stop(self) -> None:
        self._drone.control.emergency_stop(land=False)

    def close(self) -> None:
        self._drone.close()


def create_drone_api(backend: str, **kwargs) -> DroneApi:
    backend = (backend or "sverk").lower()
    if backend == "mock":
        return MockDrone()
    if backend == "sverk":
        return SverkDrone(**kwargs)
    raise ValueError(f"Неизвестный backend дрона: {backend}")
