"""Нода `enemy_tracker` — навык дрона «висеть над противником».

Берёт детекции ArUco из штатного пакета `aruco_det_loc` (топик
`/aruco/det/markers`), отбирает метки противника, проецирует их на пол и
публикует клетку. Пока действие TrackTarget активно, нода дополнительно
выдаёт цель зависания для исполнителя дрона.

Тип сообщения меток резолвится по строке имени, поэтому пакет собирается и
там, где стек дрона не установлен (ноутбук, ровер).
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener

from td_interfaces.action import TrackTarget
from td_interfaces.msg import Detection
from td_world.field import load_field
from td_world.ros_convert import cell_to_msg

from .projection import CameraModel, pixel_to_ground, quaternion_to_matrix


class EnemyTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("enemy_tracker")

        self.declare_parameter("field_config", "")
        self.declare_parameter("markers_topic", "/aruco/det/markers")
        self.declare_parameter("markers_type", "aruco_det_loc/msg/MarkerArray")
        self.declare_parameter("camera_info_topic", "/camera_1/camera_info")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("field_frame", "field")
        self.declare_parameter("enemy_marker_ids", [])
        self.declare_parameter("hover_altitude_m", 2.0)
        self.declare_parameter("detection_confidence", 0.9)
        self.declare_parameter("hover_target_topic", "/td/drone/hover_target")
        self.declare_parameter("publish_always", True)

        config = self.get_parameter("field_config").value
        if not config:
            raise RuntimeError("Параметр field_config обязателен")
        self.field = load_field(config)

        try:
            ids = [int(v) for v in self.get_parameter("enemy_marker_ids").value]
        except ParameterUninitializedException:
            # Humble treats an explicitly empty YAML list as an untyped,
            # uninitialised parameter. Empty means "take IDs from field.yaml".
            ids = []
        self.enemy_ids = set(ids or self.field.enemy_marker_ids)
        if not self.enemy_ids:
            self.get_logger().warn(
                "Не заданы ID меток противника — трекер примет любую метку"
            )
        else:
            self.get_logger().info(f"Метки противника: {sorted(self.enemy_ids)}")

        self._group = ReentrantCallbackGroup()
        self._camera: CameraModel | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last: Detection | None = None
        self._tracking = False

        try:
            markers_type = get_message(str(self.get_parameter("markers_type").value))
        except (ModuleNotFoundError, ValueError) as exc:
            # The vendor ArUco package is optional in mock/SITL environments.
            # Keep the node and action server alive; TrackTarget will simply
            # report that marker input is unavailable.
            markers_type = None
            self.get_logger().warn(f"ArUco message type unavailable: {exc}")
        if markers_type is not None:
            self.create_subscription(
                markers_type, str(self.get_parameter("markers_topic").value),
                self._on_markers, 10, callback_group=self._group,
            )
        self._markers_available = markers_type is not None
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info, 1, callback_group=self._group,
        )

        self._detections = self.create_publisher(Detection, "/td/detections", 10)
        self._hover = self.create_publisher(
            PoseStamped, str(self.get_parameter("hover_target_topic").value), 10
        )
        self._server = ActionServer(
            self, TrackTarget, "/track_target",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    # --- входные данные ----------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera is None:
            self._camera = CameraModel.from_camera_info(msg)

    def _on_markers(self, msg) -> None:
        marker = self._pick_enemy(msg)
        if marker is None:
            return
        detection = self._marker_to_detection(marker)
        if detection is None:
            return
        self._last = detection
        if self._tracking or bool(self.get_parameter("publish_always").value):
            self._detections.publish(detection)
        if self._tracking:
            self._publish_hover(detection)

    def _pick_enemy(self, msg):
        """Выбрать метку противника: самую крупную из подходящих по ID."""
        best = None
        best_area = 0.0
        for marker in getattr(msg, "markers", []):
            if self.enemy_ids and int(marker.id) not in self.enemy_ids:
                continue
            area = _corner_area(marker)
            if best is None or area > best_area:
                best, best_area = marker, area
        return best

    def _marker_to_detection(self, marker) -> Detection | None:
        pixel = _corner_centroid(marker)
        if pixel is None or self._camera is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter("field_frame").value),
                str(self.get_parameter("camera_frame").value),
                rclpy.time.Time(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Нет трансформа камеры: {exc}", throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        ground = pixel_to_ground(
            pixel, self._camera, (t.x, t.y, t.z),
            quaternion_to_matrix(r.x, r.y, r.z, r.w),
        )
        if ground is None:
            return None
        frame = str(self.get_parameter("field_frame").value)
        cell = self.field.cell_from_map(*ground) if frame == "map" else self.field.grid.cell_at(*ground)
        if cell is None:
            return None

        detection = Detection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = str(self.get_parameter("field_frame").value)
        detection.kind = Detection.KIND_ENEMY
        detection.cell = cell_to_msg(cell)
        detection.confidence = float(self.get_parameter("detection_confidence").value)
        detection.source = f"drone/aruco_{int(marker.id)}"
        detection.has_pose = True
        detection.pose.header = detection.header
        detection.pose.pose.position.x = float(ground[0])
        detection.pose.pose.position.y = float(ground[1])
        detection.pose.pose.orientation.w = 1.0
        return detection

    def _publish_hover(self, detection: Detection) -> None:
        target = PoseStamped()
        target.header = detection.header
        target.pose.position.x = detection.pose.pose.position.x
        target.pose.position.y = detection.pose.pose.position.y
        target.pose.position.z = float(self.get_parameter("hover_altitude_m").value)
        target.pose.orientation.w = 1.0
        self._hover.publish(target)

    # --- навык -------------------------------------------------------------

    def _execute(self, goal_handle):
        request = goal_handle.request
        timeout = request.timeout or 0.0
        started = time.time()
        if not self._markers_available:
            goal_handle.abort()
            return self._track_result(False, "ArUco input is unavailable", started)
        self._tracking = True
        self.get_logger().info("Сопровождение противника начато")

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._track_result(True, "сопровождение отменено", started)
                if timeout and time.time() - started > timeout:
                    goal_handle.succeed()
                    return self._track_result(True, "истёк заданный срок", started)

                feedback = TrackTarget.Feedback()
                fresh = self._last is not None and _age(self, self._last) < 1.5
                feedback.locked = bool(fresh)
                if self._last is not None:
                    feedback.detection = self._last
                goal_handle.publish_feedback(feedback)
                time.sleep(0.2)
        finally:
            self._tracking = False

        goal_handle.abort()
        return self._track_result(False, "нода останавливается", started)

    def _track_result(self, success: bool, message: str, started: float):
        result = TrackTarget.Result()
        result.success = success
        result.message = message
        result.tracked_time_s = float(time.time() - started)
        self.get_logger().info(f"Сопровождение завершено: {message}")
        return result


def _corner_centroid(marker) -> tuple[float, float] | None:
    corners = getattr(marker, "corners", None)
    if not corners:
        return None
    xs = [float(c.x) for c in corners]
    ys = [float(c.y) for c in corners]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def _corner_area(marker) -> float:
    """Площадь четырёхугольника метки в пикселях (формула шнурования)."""
    corners = getattr(marker, "corners", None)
    if not corners or len(corners) < 3:
        return 0.0
    total = 0.0
    count = len(corners)
    for i in range(count):
        x1, y1 = float(corners[i].x), float(corners[i].y)
        x2, y2 = float(corners[(i + 1) % count].x), float(corners[(i + 1) % count].y)
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def _age(node: Node, detection: Detection) -> float:
    now = node.get_clock().now().nanoseconds * 1e-9
    stamp = float(detection.header.stamp.sec) + float(detection.header.stamp.nanosec) * 1e-9
    return max(0.0, now - stamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnemyTrackerNode()
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
