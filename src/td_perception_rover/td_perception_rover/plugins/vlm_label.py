"""Чтение метки через VLM (`a733_vlm_ros2`, action `/vlm/query`).

Самый устойчивый к шрифту и наклону вариант и одновременно тот самый
LLM/VLM-компонент, который показывается на технической защите. Медленный
(доли секунды на кадр), поэтому используется как запасной или подтверждающий,
а не как основной.
"""

from __future__ import annotations

from typing import Any

from .base import LabelReader, LabelResult, register_reader

__all__ = ["VlmLabelReader"]

DEFAULT_QUERY = (
    "На бумаге написано обозначение клетки из буквы и цифры, например A3 или B5. "
    "Ответь только этим обозначением, без пояснений."
)


@register_reader
class VlmLabelReader(LabelReader):
    name = "vlm_label"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        self.query = str(self.params.get("query", DEFAULT_QUERY))
        self.action_name = str(self.params.get("action_name", "/vlm/query"))
        self.timeout_s = float(self.params.get("timeout_s", 20.0))
        self.confidence = float(self.params.get("assumed_confidence", 0.7))
        self.use_npu = bool(self.params.get("use_npu_vision", False))
        self._node = None
        self._client = None

    def bind(self, node) -> None:
        """Подключиться к ноде ROS (вызывает label_reader_node)."""
        from rclpy.action import ActionClient
        from vlm_interfaces.action import VlmQuery

        self._node = node
        self._client = ActionClient(node, VlmQuery, self.action_name)

    def read(self, image) -> LabelResult | None:
        if self._client is None:
            raise RuntimeError("vlm_label не привязан к ноде: вызовите bind(node)")

        import rclpy
        from cv_bridge import CvBridge
        from vlm_interfaces.action import VlmQuery

        if not self._client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().warn(f"VLM-сервер {self.action_name} недоступен")
            return None

        goal = VlmQuery.Goal()
        goal.query = self.query
        if hasattr(goal, "use_npu_vision"):
            goal.use_npu_vision = self.use_npu
        if hasattr(goal, "image"):
            goal.image = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")

        send = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send, timeout_sec=self.timeout_s)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=self.timeout_s)
        wrapped = result_future.result()
        if wrapped is None:
            return None

        answer = str(getattr(wrapped.result, "answer", "")).strip()
        if not answer:
            return None
        # Нормализацией занимается label_parse — здесь отдаём как есть.
        return LabelResult(text=answer, confidence=self.confidence,
                           debug={"raw": answer[:200]})
