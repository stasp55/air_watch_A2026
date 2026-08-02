"""Тесты таблицы ролей.

Таблицу читают и скрипт запуска, и launch-файл, поэтому ошибка в ней ломает
запуск на площадке молча — до первой попытки поднять систему.
"""

import re
from pathlib import Path

import pytest

from td_bringup.roles import CORE_PACKAGES, ROLES, get_role, guess_role, packages_for

LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def declared_arguments(launch_file: str) -> set:
    """Имена аргументов, объявленных в launch-файле.

    Читаем текстом, а не импортом: тесты обязаны идти на машине без ROS.
    """
    text = (LAUNCH_DIR / launch_file).read_text(encoding="utf-8")
    return set(re.findall(r'DeclareLaunchArgument\(\s*\n?\s*"([^"]+)"', text))


def test_every_role_includes_the_core_packages():
    for name, role in ROLES.items():
        assert set(CORE_PACKAGES) <= set(role.packages), name


def test_roles_do_not_drag_in_the_other_side():
    """Ровер не должен собирать пакеты дрона и наоборот: у них разные дистрибутивы."""
    assert "td_drone_executor" not in packages_for("rover")
    assert "td_perception_drone" not in packages_for("rover")
    assert "td_rover_executor" not in packages_for("drone")
    assert "td_perception_rover" not in packages_for("drone")


def test_package_lists_have_no_duplicates():
    for name, role in ROLES.items():
        assert len(role.packages) == len(set(role.packages)), name


def test_launch_files_are_named_consistently():
    for name, role in ROLES.items():
        assert role.launch_file == f"{name}.launch.py"


def test_launch_profiles_exist():
    for role in ROLES.values():
        assert (LAUNCH_DIR / role.launch_file).is_file(), role.launch_file


def test_forwarded_arguments_are_actually_declared_by_the_profile():
    """Передать профилю необъявленный аргумент — ошибка запуска, а не предупреждение.

    Ловим расхождение здесь, а не на площадке за минуту до заезда.
    """
    for name, role in ROLES.items():
        declared = declared_arguments(role.launch_file)
        unknown = set(role.launch_arguments) - declared
        assert not unknown, f"{name}: профиль не знает про {sorted(unknown)}"


def test_single_entry_point_declares_everything_it_forwards():
    """td.launch.py должен уметь принять любой аргумент, который отдаёт профилям."""
    declared = declared_arguments("td.launch.py")
    for name, role in ROLES.items():
        missing = set(role.launch_arguments) - declared
        assert not missing, f"{name}: td.launch.py не объявляет {sorted(missing)}"


def test_dryrun_takes_no_broker_arguments():
    """У прогона без железа транспорт всегда loopback — брокер ему передавать нечем."""
    assert "mqtt_host" not in get_role("dryrun").launch_arguments
    assert "transport" not in get_role("dryrun").launch_arguments


def test_dryrun_needs_no_broker():
    """Прогон без железа обязан работать в самолёте: без сети и без mosquitto."""
    assert get_role("dryrun").needs_broker is False
    assert all(ROLES[name].needs_broker for name in ("rover", "drone", "dispatcher"))


def test_unknown_role_lists_the_known_ones():
    with pytest.raises(ValueError) as excinfo:
        get_role("rovver")
    message = str(excinfo.value)
    assert "rovver" in message
    for name in ROLES:
        assert name in message


def test_role_lookup_is_forgiving_to_case_and_spaces():
    assert get_role(" Rover ").name == "rover"


@pytest.mark.parametrize(
    "distro, hostname, expected",
    [
        ("humble", "", "drone"),
        ("jazzy", "", "rover"),
        # Имя хоста задаёт человек — оно важнее дистрибутива.
        ("humble", "rover-01", "rover"),
        ("jazzy", "drone-01", "drone"),
        # Ничего не понятно -> наземная станция: она ничем не управляет физически.
        (None, None, "dispatcher"),
        ("iron", "laptop", "dispatcher"),
    ],
)
def test_role_is_guessed_from_the_machine(distro, hostname, expected):
    assert guess_role(distro, hostname) == expected
