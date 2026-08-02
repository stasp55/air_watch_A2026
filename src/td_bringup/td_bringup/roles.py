"""Таблица ролей: что такое «ровер», «дрон» и «ноутбук» в терминах пакетов.

Единственное место, где записано соответствие «роль -> пакеты + launch-файл».
Им пользуются двое:

* скрипт ``./td`` — чтобы собрать ровно то, что нужно этой машине;
* ``td.launch.py`` — чтобы по одному аргументу поднять нужную сторону.

Модуль намеренно не зависит ни от ROS, ни от чего бы то ни было: его можно
импортировать до сборки workspace и проверить тестами на любой машине.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

#: Пакеты, нужные всем без исключения: контракты, модель мира, транспорт.
CORE_PACKAGES = ["td_interfaces", "td_world", "td_link", "td_bringup"]


@dataclass(frozen=True)
class Role:
    """Одна сторона системы."""

    name: str
    launch_file: str
    #: Пакеты сверх CORE_PACKAGES.
    extra_packages: List[str] = field(default_factory=list)
    description: str = ""
    #: Нужен ли этой роли брокер MQTT (у dryrun транспорт loopback).
    needs_broker: bool = True
    #: Аргументы, которые объявляет профиль роли. Передать профилю то, чего он
    #: не объявляет, — ошибка запуска, поэтому список сверяется тестом.
    launch_arguments: List[str] = field(default_factory=list)

    @property
    def packages(self) -> List[str]:
        """Полный список пакетов для colcon, в порядке зависимостей."""
        return CORE_PACKAGES + list(self.extra_packages)


ROLES: Dict[str, Role] = {
    "dispatcher": Role(
        name="dispatcher",
        launch_file="dispatcher.launch.py",
        extra_packages=["td_dispatcher"],
        description="Наземная станция: модель мира, сценарий миссии, мост к агентам",
        launch_arguments=["field_config", "transport", "mqtt_host", "autostart"],
    ),
    "rover": Role(
        name="rover",
        launch_file="rover.launch.py",
        extra_packages=["td_perception_rover", "td_rover_executor"],
        description="Ровер: движение по клеткам, чтение метки, запретные зоны",
        launch_arguments=["field_config", "transport", "mqtt_host"],
    ),
    "drone": Role(
        name="drone",
        launch_file="drone.launch.py",
        extra_packages=["td_perception_drone", "td_drone_executor"],
        description="Дрон: взлёт и полёт, поиск игрушки, слежение за противником",
        launch_arguments=["field_config", "transport", "mqtt_host", "backend", "plugin"],
    ),
    "dryrun": Role(
        name="dryrun",
        launch_file="dryrun.launch.py",
        extra_packages=[
            "td_dispatcher",
            "td_drone_executor",
            "td_perception_drone",
        ],
        description="Прогон всего сценария на одной машине, без железа и брокера",
        needs_broker=False,
        # Транспорт и брокер здесь не настраиваются: профиль всегда loopback.
        launch_arguments=["field_config"],
    ),
}


def get_role(name: str) -> Role:
    """Роль по имени. Ошибка перечисляет допустимые значения, а не просто ругается."""
    try:
        return ROLES[name.strip().lower()]
    except KeyError:
        known = ", ".join(sorted(ROLES))
        raise ValueError(f"неизвестная роль '{name}', доступны: {known}") from None


def packages_for(name: str) -> List[str]:
    """Список пакетов для colcon build."""
    return get_role(name).packages


def guess_role(distro: Optional[str] = None, hostname: Optional[str] = None) -> str:
    """Догадаться о роли машины, чтобы не заставлять называть её каждый раз.

    Сначала смотрим на имя хоста (самое надёжное — его задаёт человек), затем на
    дистрибутив ROS: у дрона Humble, у ровера Jazzy. Если ничего не понятно,
    считаем машину наземной станцией: это единственная роль, которая ничем не
    управляет физически, поэтому ошибиться в её пользу безопаснее всего.
    """
    host = (hostname or "").lower()
    for token, role in (("rover", "rover"), ("drone", "drone"), ("uav", "drone")):
        if token in host:
            return role

    distro = (distro or "").lower()
    if distro == "humble":
        return "drone"
    if distro == "jazzy":
        return "rover"
    return "dispatcher"
