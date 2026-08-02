"""
VLM Action Server для A733 (Orange Pi Zero 3W).

Принимает sensor_msgs/Image + текстовый запрос → ответ VLM.
Два режима vision encoding:
  - CPU: llama-cli с --mmproj (mmproj GGUF файл)
  - NPU: llama-cli с A733_NPU_EMBEDDINGS=1 (vision через VIPLite MobileCLIP-S0)

CPU affinity: llama-cli subprocess привязывается к A76 ядрам (0-1),
чтобы не вытеснять ArUco/odometry с A55 (2-7).
"""

import os
import subprocess
import tempfile
import time
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from vlm_interfaces.action import VlmQuery


# A733 Linux CPU numbering: 0-5 are Cortex-A55, 6-7 are the two big A76.
# The dispatcher itself stays on A55; the one-shot VLM job may use all cores.
_A76_CORES = {6, 7}
_A55_CORES = {0, 1, 2, 3, 4, 5}


def _pin_to_cores(cores: set) -> None:
    try:
        os.sched_setaffinity(0, cores)
    except OSError:
        pass  # внутри Docker без CAP_SYS_NICE — молча игнорируем


class VlmActionServer(Node):

    def __init__(self):
        super().__init__('vlm_action_server')

        # ── параметры ──────────────────────────────────────────────────
        self.declare_parameter('model_path', '')
        self.declare_parameter('mmproj_path', '')
        self.declare_parameter('npu_nbg_path', '')       # путь к NBG для NPU
        self.declare_parameter('ctx_size', 2048)
        self.declare_parameter('max_tokens', 256)
        self.declare_parameter('temperature', 0.0)       # 0 = greedy, быстрее
        self.declare_parameter('llama_cli_bin', 'llama-cli')
        self.declare_parameter('inference_timeout_s', 45.0)
        # Системный промпт — настраивается без пересборки
        self.declare_parameter(
            'system_prompt',
            'You are a vision assistant on a drone. Be concise.'
        )

        self._bridge = CvBridge()
        self._lock = threading.Lock()       # только один инференс за раз
        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            VlmQuery,
            'vlm/query',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        # Пин самого ROS-нода на A55 — он только диспетчер
        _pin_to_cores(_A55_CORES)

        self.get_logger().info('VLM action server ready on /vlm/query')

    # ── callbacks ──────────────────────────────────────────────────────

    def _goal_cb(self, _goal):
        if self._lock.locked():
            self.get_logger().warn('VLM busy, rejecting new goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal):
        return CancelResponse.ACCEPT

    # ── main execute ───────────────────────────────────────────────────

    def _execute(self, goal_handle):
        req = goal_handle.request
        t0 = time.monotonic()

        result = VlmQuery.Result()

        # Получаем параметры
        model_path  = self.get_parameter('model_path').value
        mmproj_path = self.get_parameter('mmproj_path').value
        npu_nbg     = self.get_parameter('npu_nbg_path').value
        use_npu     = req.use_npu_vision and bool(npu_nbg)

        if not model_path or not os.path.exists(model_path):
            result.success = False
            result.error_msg = f'model_path not set or missing: {model_path!r}'
            goal_handle.abort()
            return result

        with self._lock:
            try:
                response = self._run_inference(goal_handle, req, model_path,
                                               mmproj_path, use_npu, npu_nbg)
                result.success = True
                result.response = response
            except subprocess.TimeoutExpired:
                result.success = False
                result.error_msg = 'inference timeout'
                goal_handle.abort()
                return result
            except Exception as e:
                result.success = False
                result.error_msg = str(e)
                goal_handle.abort()
                return result

        result.inference_time_ms = (time.monotonic() - t0) * 1000.0
        goal_handle.succeed()
        return result

    # ── inference pipeline ─────────────────────────────────────────────

    def _run_inference(self, goal_handle, req, model_path,
                       mmproj_path, use_npu, npu_nbg):
        # 1. ROS Image → JPEG во временный файл
        self._send_feedback(goal_handle, 'preprocessing')
        img_path = self._ros_image_to_jpeg(req.image)

        try:
            # 2. Строим команду llama-cli
            cmd = self._build_cmd(
                model_path, mmproj_path, img_path,
                req.query, use_npu
            )

            # 3. Env: NPU hybrid путь активируется переменной окружения
            env = os.environ.copy()
            if use_npu:
                env['A733_NPU_EMBEDDINGS'] = '1'
                env['A733_NBG_PATH'] = npu_nbg
                self._send_feedback(goal_handle, 'vision_encoding (NPU)')
            else:
                self._send_feedback(goal_handle, 'vision_encoding (CPU)')

            # 4. This is an infrequent, blocking vision query.  Use the whole
            # CPU cluster, otherwise 2 A76 cores cannot finish this VLM fast.
            taskset_cmd = ['taskset', '-c', '0-7'] + cmd

            self._send_feedback(goal_handle, 'generating')

            proc = subprocess.Popen(
                taskset_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                text=True,
            )

            # communicate() owns both pipes and limits the whole subprocess.
            # The former blocking stdout.read() made wait(timeout=120)
            # unreachable when inference stopped producing output.
            try:
                response_text, stderr = proc.communicate(
                    timeout=float(self.get_parameter('inference_timeout_s').value)
                )
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate()
                raise
            if proc.returncode != 0:
                raise RuntimeError(f'llama-cli exit {proc.returncode}: {stderr[:300]}')

            return response_text.strip()

        finally:
            try:
                os.unlink(img_path)
            except OSError:
                pass

    def _build_cmd(self, model_path, mmproj_path, img_path, query, use_npu):
        system_prompt = self.get_parameter('system_prompt').value
        ctx           = self.get_parameter('ctx_size').value
        max_tok       = self.get_parameter('max_tokens').value
        temp          = self.get_parameter('temperature').value
        llama_bin     = self.get_parameter('llama_cli_bin').value

        # SmolVLM / LLaVA-style prompt format
        prompt = (
            f'<|im_start|>system\n{system_prompt}<|im_end|>\n'
            f'<|im_start|>user\n<image>\n{query}<|im_end|>\n'
            f'<|im_start|>assistant\n'
        )

        cmd = [
            llama_bin,
            '--model', model_path,
            '--ctx-size', str(ctx),
            '--predict', str(max_tok),
            '--temp', str(temp),
            '--image', img_path,
            '--no-display-prompt',
            '--log-disable',
            '-p', prompt,
            '--threads', '8',
        ]

        if not use_npu and mmproj_path and os.path.exists(mmproj_path):
            cmd += ['--mmproj', mmproj_path]

        return cmd

    def _ros_image_to_jpeg(self, ros_image) -> str:
        # Конвертируем в BGR если нужно (cv_bridge делает это по encoding)
        try:
            cv_img = self._bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except Exception as e:
            raise RuntimeError(f'cv_bridge conversion failed: {e}')

        fd, path = tempfile.mkstemp(suffix='.jpg', prefix='vlm_frame_')
        os.close(fd)
        # Binary toy detection does not need a 1280x960 camera frame.  A small
        # image drastically shortens CPU vision encoding on A733, while 256 px
        # on the long side leaves ample detail for a large brown plush toy.
        height, width = cv_img.shape[:2]
        max_side = 256
        if max(height, width) > max_side:
            scale = max_side / float(max(height, width))
            cv_img = cv2.resize(
                cv_img,
                (round(width * scale), round(height * scale)),
                interpolation=cv2.INTER_AREA,
            )
        cv2.imwrite(path, cv_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return path

    def _stream_output(self, goal_handle, proc) -> str:
        accumulated = ''
        feedback = VlmQuery.Feedback()

        for chunk in iter(lambda: proc.stdout.read(32), ''):
            if goal_handle.is_cancel_requested:
                proc.kill()
                goal_handle.canceled()
                return accumulated

            accumulated += chunk
            feedback.stage = 'generating'
            feedback.partial_response = accumulated
            goal_handle.publish_feedback(feedback)

        return accumulated

    def _send_feedback(self, goal_handle, stage: str) -> None:
        fb = VlmQuery.Feedback()
        fb.stage = stage
        fb.partial_response = ''
        goal_handle.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = VlmActionServer()
    # MultiThreadedExecutor — action сервер требует минимум 2 потока
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
