"""Единая точка входа: один launch-файл на все машины.

    ros2 launch td_bringup td.launch.py role:=dispatcher mqtt_host:=10.63.18.111
    ros2 launch td_bringup td.launch.py role:=rover      mqtt_host:=10.63.18.111
    ros2 launch td_bringup td.launch.py role:=drone      mqtt_host:=10.63.18.111
    ros2 launch td_bringup td.launch.py role:=dryrun

Файл ничего не поднимает сам — он выбирает и включает профиль роли из
`roles.py`. Профили (`rover.launch.py` и остальные) остались на месте и
по-прежнему запускаются напрямую, если нужно тонко покрутить аргументы.

Роль можно не указывать: тогда она берётся из переменной окружения `TD_ROLE`,
а если и её нет — угадывается по имени хоста и дистрибутиву ROS. Хост брокера
по умолчанию берётся из `TD_MQTT_HOST`. Обе переменные выставляет скрипт `./td`,
поэтому в обычной работе аргументы писать не нужно вовсе.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from td_bringup.roles import get_role, guess_role


def _launch_setup(context, *args, **kwargs):
    """Разворачивается в момент запуска, когда аргументы уже известны.

    Выбор через OpaqueFunction, а не через условия: условные подстановки в
    Humble и Jazzy называются по-разному, а один и тот же файл должен
    запускаться на обеих машинах.
    """
    role_name = LaunchConfiguration("role").perform(context).strip()
    if not role_name:
        role_name = guess_role(os.environ.get("ROS_DISTRO"), os.uname().nodename)

    role = get_role(role_name)
    share = FindPackageShare("td_bringup").perform(context)
    profile = os.path.join(share, "launch", role.launch_file)

    # Прокидываем только то, что профиль объявляет, и только если задано явно:
    # пустое значение означает «оставить умолчание профиля».
    arguments = []
    for key in role.launch_arguments:
        value = LaunchConfiguration(key).perform(context)
        # Родительский launch объявляет field_config как пустую строку, чтобы
        # пользователь мог его переопределить. Без явной подстановки это пустое
        # значение затеняет default дочернего профиля.
        if not value:
            value = {
                "field_config": os.path.join(share, "config", "field.yaml"),
                "transport": "mqtt",
                "backend": "sverk",
                "plugin": "hsv_color",
                "autostart": "false",
            }.get(key, "")
        if value:
            arguments.append((key, value))

    print(f"[td] роль: {role.name} — {role.description}")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(profile),
            launch_arguments=arguments,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    declarations = [
        DeclareLaunchArgument(
            "role",
            default_value=os.environ.get("TD_ROLE", ""),
            description="dispatcher | rover | drone | dryrun (пусто = угадать)",
        ),
        DeclareLaunchArgument(
            "mqtt_host",
            default_value=os.environ.get("TD_MQTT_HOST", "127.0.0.1"),
            description="Адрес брокера MQTT",
        ),
        DeclareLaunchArgument(
            "transport",
            default_value="",
            description="mqtt | loopback (пусто = как в профиле роли)",
        ),
        DeclareLaunchArgument(
            "field_config", default_value="",
            description="Своё описание поля вместо config/field.yaml",
        ),
        DeclareLaunchArgument(
            "autostart", default_value="",
            description="true = диспетчер начинает миссию сразу после запуска",
        ),
        DeclareLaunchArgument(
            "backend", default_value="",
            description="Только role:=drone — sverk | mock",
        ),
        DeclareLaunchArgument(
            "plugin", default_value="",
            description="Только role:=drone — hsv_color | yolo_onnx | vlm_query",
        ),
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
