"""Запуск стороны дрона.

Поднимает навыки полёта, поиск игрушки, сопровождение противника и мост до
диспетчера. Полётный стек (PX4, offboard, камера, ArUco) запускается отдельно
штатным `launch_system` из sverk-ros2.

    ros2 launch td_bringup drone.launch.py
    ros2 launch td_bringup drone.launch.py backend:=mock   # отладка без полёта
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
    backend = LaunchConfiguration("backend")
    plugin = LaunchConfiguration("plugin")
    transport = LaunchConfiguration("transport")
    mqtt_host = LaunchConfiguration("mqtt_host")

    arguments = [
        DeclareLaunchArgument(
            "field_config",
            default_value=PathJoinSubstitution([share, "config", "field.yaml"]),
        ),
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution([share, "config", "drone.yaml"]),
        ),
        DeclareLaunchArgument("backend", default_value="sverk",
                              description="sverk (реальный полёт) | mock (без полёта)"),
        DeclareLaunchArgument("plugin", default_value="hsv_color",
                              description="hsv_color | yolo_onnx | vlm_query"),
        DeclareLaunchArgument("transport", default_value="mqtt"),
        DeclareLaunchArgument("mqtt_host", default_value="127.0.0.1"),
    ]

    nodes = [
        Node(package="td_drone_executor", executable="drone_skills",
             name="drone_skills", output="screen",
             parameters=[params, {"field_config": field_config, "backend": backend}]),
        Node(package="td_perception_drone", executable="object_finder",
             name="object_finder", output="screen",
             parameters=[params, {"field_config": field_config, "plugin": plugin}]),
        Node(package="td_perception_drone", executable="enemy_tracker",
             name="enemy_tracker", output="screen",
             parameters=[params, {"field_config": field_config}]),
        Node(package="td_link", executable="agent_link",
             name="agent_link", output="screen",
             parameters=[params, {"transport": transport, "mqtt_host": mqtt_host}]),
    ]
    return LaunchDescription(arguments + nodes)
