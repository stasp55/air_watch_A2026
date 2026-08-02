"""Запуск наземной станции (ноутбук).

Поднимает модель мира, диспетчера, мост до агентов и статическую привязку
поля к карте ровера.

    ros2 launch td_bringup dispatcher.launch.py
    ros2 service call /dispatcher/start std_srvs/srv/Trigger   # старт миссии
    ros2 service call /dispatcher/abort std_srvs/srv/Trigger   # аварийный стоп
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("td_bringup")
    field_config = LaunchConfiguration("field_config")
    params = LaunchConfiguration("params")
    transport = LaunchConfiguration("transport")
    mqtt_host = LaunchConfiguration("mqtt_host")
    autostart = LaunchConfiguration("autostart")

    arguments = [
        DeclareLaunchArgument(
            "field_config",
            default_value=PathJoinSubstitution([share, "config", "field.yaml"]),
        ),
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution([share, "config", "dispatcher.yaml"]),
        ),
        DeclareLaunchArgument("transport", default_value="mqtt"),
        DeclareLaunchArgument("mqtt_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("autostart", default_value="false",
                              description="true = начать миссию сразу после запуска"),
    ]

    common = [params, {"field_config": field_config}]

    nodes = [
        Node(package="td_world", executable="world_state",
             name="world_state", output="screen", parameters=common),
        Node(package="td_world", executable="field_tf",
             name="field_tf", output="screen", parameters=common),
        Node(package="td_dispatcher", executable="dispatcher",
             name="dispatcher", output="screen",
             parameters=[params, {"field_config": field_config, "autostart": autostart}]),
        Node(package="td_link", executable="dispatcher_link",
             name="dispatcher_link", output="screen",
             parameters=[params, {"transport": transport, "mqtt_host": mqtt_host}]),
    ]
    return LaunchDescription(arguments + nodes)
