"""Прогон всей системы на одной машине, без железа и без брокера.

Поднимает диспетчера, модель мира, обе стороны моста через `loopback`,
навыки дрона на заглушке (`backend: mock`) и имитатор противника. Позволяет
за минуту проверить весь сценарий целиком после любой правки логики.

    ros2 launch td_bringup dryrun.launch.py

Ровер в этом режиме не запускается: его навык goto_cell требует Nav2.
Для полного прогона с ровером используйте SITL-симулятор и rover.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("td_bringup")
    field_config = LaunchConfiguration("field_config")
    dispatcher_params = PathJoinSubstitution([share, "config", "dispatcher.yaml"])
    drone_params = PathJoinSubstitution([share, "config", "drone.yaml"])

    arguments = [
        DeclareLaunchArgument(
            "field_config",
            default_value=PathJoinSubstitution([share, "config", "field.yaml"]),
        ),
        DeclareLaunchArgument("ghost_enemy", default_value="true"),
    ]

    local = {"transport": "loopback"}

    nodes = [
        Node(package="td_world", executable="world_state", name="world_state",
             output="screen", parameters=[dispatcher_params, {"field_config": field_config}]),
        Node(package="td_dispatcher", executable="dispatcher", name="dispatcher",
             output="screen",
             parameters=[dispatcher_params,
                         {"field_config": field_config, "autostart": True}]),
        Node(package="td_link", executable="dispatcher_link", name="dispatcher_link",
             output="screen", parameters=[dispatcher_params, local]),
        Node(package="td_link", executable="agent_link", name="agent_link_drone",
             output="screen", parameters=[drone_params, local]),
        Node(package="td_drone_executor", executable="drone_skills", name="drone_skills",
             output="screen",
             parameters=[drone_params, {"field_config": field_config, "backend": "mock"}]),
        Node(package="td_bringup", executable="ghost_enemy", name="ghost_enemy",
             output="screen"),
    ]
    return LaunchDescription(arguments + nodes)
