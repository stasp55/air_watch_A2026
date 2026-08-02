"""Нода `object_finder` — навык дрона «найти объект».

Облетает клетки-кандидаты, на каждой снимает несколько кадров и прогоняет их
через выбранный плагин зрения. Найденный пиксель проецируется на пол и
превращается в клетку — наружу уходит уже готовая детекция.

Обход кандидатов выполняется здесь, а не в диспетчере: так поиск переживает
кратковременную потерю связи с наземной станцией.

Плагин выбирается параметром `plugin` (`hsv_color`, `yolo_onnx`, `vlm_query`).
"""

from __future__ import annotations

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from td_interfaces.action import FindObject, GoToCell
from td_interfaces.msg import Detection
from td_world.field import load_field
from td_world.grid import CellRef
from td_world.ros_convert import cell_to_msg, msg_to_cell

from .plugins.base import create_detector
from .projection import CameraModel, pixel_to_ground, quaternion_to_matrix


class ObjectFinderNode(Node):
    def __init__(self) -> None:
        super().__init__("object_finder")

        self.declare_parameter("field_config", "")
        self.declare_parameter("plugin", "hsv_color")
        self.declare_parameter("kind", Detection.KIND_TARGET)
        self.declare_parameter("image_topic", "/camera_1/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_1/camera_info")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("field_frame", "field")
        self.declare_parameter("samples_per_cell", 5)
        self.declare_parameter("sample_period_s", 0.3)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("scan_altitude_m", 1.6)
        self.declare_parameter("goto_action", "/goto_cell")
        self.declare_parameter("use_goto", True)
        # Параметры плагинов (лишние для выбранного плагина просто игнорируются).
        self.declare_parameter("hsv_lower", [10, 120, 90])
        self.declare_parameter("hsv_upper", [28, 255, 255])
        self.declare_parameter("min_area_px", 300)
        self.declare_parameter("model_path", "")
        self.declare_parameter("class_id", -1)
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("query", "На изображении есть игрушка? Ответь да или нет.")

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        self._group = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._image: Image | None = None
        self._camera: CameraModel | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        plugin_name = str(self.get_parameter("plugin").value)
        self.detector = create_detector(plugin_name, self._plugin_params())
        if hasattr(self.detector, "bind"):
            self.detector.bind(self)
        self.get_logger().info(f"Плагин зрения: {self.detector.describe()}")

        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._on_image, 1, callback_group=self._group,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info, 1, callback_group=self._group,
        )
        self._detections = self.create_publisher(Detection, "/td/detections", 10)

        self._goto = ActionClient(
            self, GoToCell, str(self.get_parameter("goto_action").value),
            callback_group=self._group,
        )
        self._server = ActionServer(
            self, FindObject, "/find_object",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    def _plugin_params(self) -> dict:
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        return {
            "hsv_lower": tuple(int(v) for v in get("hsv_lower")),
            "hsv_upper": tuple(int(v) for v in get("hsv_upper")),
            "min_area_px": int(get("min_area_px")),
            "model_path": str(get("model_path")),
            "class_id": int(get("class_id")),
            "conf_threshold": float(get("conf_threshold")),
            "query": str(get("query")),
        }

    # --- данные камеры -----------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        self._image = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera is None:
            self._camera = CameraModel.from_camera_info(msg)
            self.get_logger().info(
                f"Камера: fx={self._camera.fx:.1f} fy={self._camera.fy:.1f} "
                f"{self._camera.width}x{self._camera.height}"
            )

    # --- выполнение навыка -------------------------------------------------

    def _execute(self, goal_handle):
        request = goal_handle.request
        kind = request.kind or str(self.get_parameter("kind").value)
        min_conf = request.min_confidence or float(self.get_parameter("min_confidence").value)

        candidates = [
            cell for cell in (msg_to_cell(c, self.field.grid) for c in request.candidates)
            if cell is not None
        ]
        # Empty candidates is a manual inspection: OpenCV sees the current frame
        # only. Automatic mode always sends an explicit list of field cells.
        inspect_current = not candidates
        if inspect_current:
            # Keep a valid cell for action feedback; movement is suppressed below.
            candidates = [self.field.start_cell]

        if inspect_current:
            self.get_logger().info(f"Ручная проверка {kind}: текущий кадр OpenCV")
        else:
            self.get_logger().info(
                f"Поиск {kind} в клетках: {', '.join(c.name for c in candidates)}"
            )

        result = FindObject.Result()
        best: Detection | None = None
        deadline = time.time() + (request.timeout or 120.0)

        for index, cell in enumerate(candidates):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "поиск отменён"
                return result
            if time.time() > deadline:
                break

            if not inspect_current and bool(self.get_parameter("use_goto").value) and not self._fly_to(cell):
                self.get_logger().warn(f"Не удалось выйти на клетку {cell.name}, пропускаю")
                continue

            detection = self._inspect(kind)
            feedback = FindObject.Feedback()
            feedback.inspecting = cell_to_msg(cell)
            feedback.candidates_done = index + 1
            feedback.candidates_total = len(candidates)
            feedback.best_confidence = detection.confidence if detection else 0.0
            goal_handle.publish_feedback(feedback)

            if detection is None:
                continue
            self._detections.publish(detection)
            if best is None or detection.confidence > best.confidence:
                best = detection
            if detection.confidence >= min_conf:
                break

        if best is None or best.confidence < min_conf:
            goal_handle.abort()
            result.success = False
            result.message = (
                "объект не найден"
                if best is None
                else f"лучшая уверенность {best.confidence:.2f} ниже порога {min_conf:.2f}"
            )
            return result

        goal_handle.succeed()
        result.success = True
        result.detection = best
        result.message = f"{kind} в клетке {best.cell.name}"
        self.get_logger().info(result.message)
        return result

    def _fly_to(self, cell: CellRef) -> bool:
        """Вывести дрон на клетку через локальный навык перемещения."""
        if not self._goto.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Навык goto_cell недоступен, снимаю с текущей позиции")
            return True
        goal = GoToCell.Goal()
        goal.goal = cell_to_msg(cell)
        goal.yaw_deg = float("nan")
        future = self._goto.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        wrapped = result_future.result()
        return bool(wrapped and wrapped.result.success)

    def _inspect(self, kind: str) -> Detection | None:
        """Снять несколько кадров и вернуть лучшую детекцию."""
        samples = int(self.get_parameter("samples_per_cell").value)
        period = float(self.get_parameter("sample_period_s").value)
        best: Detection | None = None

        for _ in range(max(1, samples)):
            image_msg = self._image
            if image_msg is None:
                time.sleep(period)
                continue
            image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            found = self.detector.detect(image)
            if found is not None:
                detection = self._to_detection(kind, found)
                if detection is not None and (best is None or detection.confidence > best.confidence):
                    best = detection
            time.sleep(period)
        return best

    def _to_detection(self, kind: str, found) -> Detection | None:
        """Пиксель -> точка на полу -> клетка."""
        if self._camera is None:
            self.get_logger().warn("Нет CameraInfo, детекцию некуда проецировать")
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter("field_frame").value),
                str(self.get_parameter("camera_frame").value),
                rclpy.time.Time(),
            )
        except Exception as exc:  # noqa: BLE001 - TF может отсутствовать в первые секунды
            self.get_logger().warn(f"Нет трансформа камеры: {exc}")
            return None

        translation = tf.transform.translation
        rotation = tf.transform.rotation
        ground = pixel_to_ground(
            found.pixel,
            self._camera,
            (translation.x, translation.y, translation.z),
            quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
        )
        if ground is None:
            self.get_logger().warn("Луч не пересекает пол — детекция отброшена")
            return None

        frame = str(self.get_parameter("field_frame").value)
        # The real flight stack publishes camera TF in map.  Convert the
        # projected point back into the field grid before choosing a cell.
        cell = self.field.cell_from_map(*ground) if frame == "map" else self.field.grid.cell_at(*ground)
        if cell is None:
            self.get_logger().warn(
                f"Точка {ground[0]:.2f},{ground[1]:.2f} вне поля — детекция отброшена"
            )
            return None

        detection = Detection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = str(self.get_parameter("field_frame").value)
        detection.kind = kind
        detection.cell = cell_to_msg(cell)
        detection.confidence = float(found.confidence)
        detection.source = self.detector.describe()
        detection.has_pose = True
        detection.pose.header = detection.header
        detection.pose.pose.position.x = float(ground[0])
        detection.pose.pose.position.y = float(ground[1])
        detection.pose.pose.orientation.w = 1.0
        return detection


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectFinderNode()
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
