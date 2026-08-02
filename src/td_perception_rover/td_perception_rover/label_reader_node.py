"""Нода `label_reader` — навык ровера «прочитать финишную клетку».

Снимает несколько кадров с бортовой камеры, прогоняет их через выбранный
плагин, нормализует ответы и берёт моду. Наружу уходит детекция
`FINISH_LABEL` с каноническим именем клетки в поле `payload`.

Плагин выбирается параметром `plugin` (`template_match`, `vlm_label`,
`ocr_tesseract`) и может быть заменён на лету через `ros2 param set`.
"""

from __future__ import annotations

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from td_interfaces.action import ReadLabel
from td_interfaces.msg import Detection
from td_world.field import load_field
from td_world.ros_convert import cell_to_msg

from .label_parse import vote_labels
from .plugins.base import create_reader


class LabelReaderNode(Node):
    def __init__(self) -> None:
        super().__init__("label_reader")

        self.declare_parameter("field_config", "")
        self.declare_parameter("plugin", "template_match")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("templates_dir", "")
        self.declare_parameter("samples", 7)
        self.declare_parameter("sample_period_s", 0.25)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("query", "")

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        self._group = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._image: Image | None = None

        self.reader = create_reader(
            str(self.get_parameter("plugin").value),
            {
                "templates_dir": str(self.get_parameter("templates_dir").value),
                "query": str(self.get_parameter("query").value) or None,
            },
        )
        if hasattr(self.reader, "bind"):
            self.reader.bind(self)
        self.get_logger().info(f"Плагин чтения метки: {self.reader.describe()}")

        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._on_image, 1, callback_group=self._group,
        )
        self._detections = self.create_publisher(Detection, "/td/detections", 10)
        self._debug = (
            self.create_publisher(Image, "~/debug_image", 1)
            if bool(self.get_parameter("publish_debug_image").value)
            else None
        )

        self._server = ActionServer(
            self, ReadLabel, "/read_label",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    def _on_image(self, msg: Image) -> None:
        self._image = msg

    # --- навык -------------------------------------------------------------

    def _execute(self, goal_handle):
        request = goal_handle.request
        samples = int(request.samples or self.get_parameter("samples").value)
        period = float(self.get_parameter("sample_period_s").value)
        min_conf = float(request.min_confidence or self.get_parameter("min_confidence").value)
        deadline = time.time() + (request.timeout or 20.0)

        readings: list[tuple[str, float]] = []
        result = ReadLabel.Result()

        for index in range(max(1, samples)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "чтение отменено"
                return result
            if time.time() > deadline:
                break

            image_msg = self._image
            if image_msg is None:
                self.get_logger().warn("Нет кадров с камеры", throttle_duration_sec=3.0)
                time.sleep(period)
                continue

            image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            try:
                found = self.reader.read(image)
            except Exception as exc:  # noqa: BLE001 - плагин не должен ронять навык
                self.get_logger().error(f"Плагин чтения упал: {exc}")
                found = None

            if found is not None:
                readings.append((found.text, found.confidence))
                self._publish_debug(image, found)

            feedback = ReadLabel.Feedback()
            feedback.samples_done = index + 1
            feedback.last_text = found.text if found else ""
            feedback.best_confidence = max((c for _, c in readings), default=0.0)
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        vote = vote_labels(readings, rows=self.field.grid.rows, cols=self.field.grid.cols)
        result.raw_text = "; ".join(text for text, _ in readings)

        if vote.label is None or vote.confidence < min_conf:
            goal_handle.abort()
            result.success = False
            result.confidence = vote.confidence
            result.message = (
                "метка не прочитана"
                if vote.label is None
                else f"голосование дало {vote.label} с уверенностью "
                     f"{vote.confidence:.2f} ниже порога {min_conf:.2f}"
            )
            self.get_logger().warn(result.message)
            return result

        cell = self.field.cell(vote.label)
        detection = Detection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = "field"
        detection.kind = Detection.KIND_FINISH_LABEL
        detection.cell = cell_to_msg(cell)
        detection.payload = vote.label
        detection.confidence = float(vote.confidence)
        detection.source = self.reader.describe()
        self._detections.publish(detection)

        goal_handle.succeed()
        result.success = True
        result.cell = cell_to_msg(cell)
        result.confidence = float(vote.confidence)
        result.message = (
            f"финиш {vote.label} ({vote.votes}/{vote.total} кадров, "
            f"уверенность {vote.confidence:.2f})"
        )
        self.get_logger().info(result.message)
        return result

    def _publish_debug(self, image, found) -> None:
        if self._debug is None:
            return
        import cv2

        annotated = image.copy()
        x, y, w, h = found.bbox
        if w > 0 and h > 0:
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            annotated, f"{found.text} {found.confidence:.2f}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
        )
        self._debug.publish(self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LabelReaderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
