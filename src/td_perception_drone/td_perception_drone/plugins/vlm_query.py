"""Детектор через VLM (`a733_vlm_ros2`, action `/vlm/query`).

Самый медленный и самый гибкий вариант: модель отвечает текстом на вопрос
вида «есть ли на изображении игрушка?». Полезен как независимая проверка
цветового детектора и как заметный аргумент на технической защите — та же
модель обслуживает и чтение метки финиша на ровере.

Центр объекта VLM не даёт, поэтому возвращается центр кадра: этого достаточно,
если дрон уже висит над клеткой-кандидатом и решает вопрос «здесь или нет».
"""

from __future__ import annotations

from typing import Any

from .base import DetectionResult, ObjectDetector, register_detector

__all__ = ["VlmQueryDetector", "interpret_answer"]

POSITIVE_WORDS = ("да", "yes", "есть", "вижу", "присутствует", "true")
NEGATIVE_WORDS = ("нет", "no", "отсутствует", "не вижу", "false")


def interpret_answer(text: str) -> tuple[bool, float]:
    """Свести свободный ответ модели к «нашли / не нашли» и уверенности."""
    lowered = (text or "").strip().lower()
    if not lowered:
        return False, 0.0
    negative = any(word in lowered for word in NEGATIVE_WORDS)
    positive = any(word in lowered for word in POSITIVE_WORDS)
    if positive and not negative:
        return True, 0.75
    if positive and negative:
        return False, 0.3   # модель сомневается — не считаем находкой
    return False, 0.0


@register_detector
class VlmQueryDetector(ObjectDetector):
    name = "vlm_query"

    def __init__(self, params: dict[str, Any] | None = None) -> None:
        super().__init__(params)
        self.query = str(
            self.params.get("query", "На изображении есть игрушка? Ответь да или нет.")
        )
        self.timeout_s = float(self.params.get("timeout_s", 15.0))
        self.action_name = str(self.params.get("action_name", "/vlm/query"))
        self.use_npu = bool(self.params.get("use_npu_vision", False))
        self._client = None
        self._node = None

    def bind(self, node) -> None:
        """Подключиться к ноде ROS. Вызывается object_finder_node при создании."""
        from rclpy.action import ActionClient
        from vlm_interfaces.action import VlmQuery

        self._node = node
        self._client = ActionClient(node, VlmQuery, self.action_name)

    def detect(self, image) -> DetectionResult | None:
        if self._client is None:
            raise RuntimeError(
                "vlm_query не привязан к ноде: вызовите bind(node) перед detect()"
            )
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

        import rclpy

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

        answer = getattr(wrapped.result, "answer", "")
        found, confidence = interpret_answer(answer)
        if not found:
            return None

        height, width = image.shape[:2]
        return DetectionResult(
            pixel=(width / 2.0, height / 2.0),
            confidence=confidence,
            bbox=(0, 0, width, height),
            label="vlm",
            debug={"answer": answer[:200]},
        )
