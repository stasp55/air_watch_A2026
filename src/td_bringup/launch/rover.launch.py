"""Запуск стороны ровера.

Поднимает навыки ровера, чтение метки, публикацию запретных зон, отчёт о
статусе и мост до диспетчера. Штатный стек ровера (Nav2, SLAM, камера)
запускается отдельно своим `rover_bringup`.

    ros2 launch td_bringup rover.launch.py
    ros2 launch td_bringup rover.launch.py transport:=loopback   # без брокера
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

    arguments = [
        DeclareLaunchArgument(
            "field_config",
            default_value=PathJoinSubstitution([share, "config", "field.yaml"]),
            description="Описание поля (единственный источник правды)",
        ),
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution([share, "config", "rover.yaml"]),
            description="Параметры нод ровера",
        ),
        DeclareLaunchArgument("transport", default_value="mqtt",
                              description="mqtt | loopback"),
        DeclareLaunchArgument("mqtt_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_label_reader", default_value="true"),
    ]

    common = [params, {"field_config": field_config}]

    nodes = [
        Node(package="td_rover_executor", executable="goto_cell",
             name="goto_cell", output="screen", parameters=common),
        Node(package="td_rover_executor", executable="keepout_publisher",
             name="keepout_publisher", output="screen", parameters=common),
        Node(package="td_rover_executor", executable="rover_status",
             name="rover_status", output="screen", parameters=common),
        Node(package="td_perception_rover", executable="label_reader",
             name="label_reader", output="screen", parameters=common),
        Node(package="td_link", executable="agent_link",
             name="agent_link", output="screen",
             parameters=[params, {"transport": transport, "mqtt_host": mqtt_host}]),
    ]
    return LaunchDescription(arguments + nodes)
