#!/usr/bin/env python3
# encoding: utf-8
"""YOLOv11-based shape detector (ROS2)."""

import os
import queue
import threading
from typing import Optional

import cv2
import numpy as np
from ultralytics import YOLO
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

import sdk.common as common


MODE_PATH = os.path.split(os.path.realpath(__file__))[0]


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolov5_node')
        self.active = False
        self.calibration = False
        self.model: Optional[YOLO] = None
        self.image_sub = None
        self.latest_shape = 'None'

        self.image_queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=2)
        self.worker_stop = threading.Event()
        self.worker_thread = threading.Thread(target=self._inference_worker, daemon=True)
        self.worker_thread.start()

        self.declare_parameter('model_path', os.path.join(MODE_PATH, 'best.pt'))
        self.declare_parameter('classes', ['cube', 'box', 'cylinder'])
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('camera_name', 'astra_camera')
        self.declare_parameter('calibration', False)
        self.declare_parameter('debug_display', False)
        # Limit enqueue rate to reduce CPU when camera FPS is high.
        # Set <= 0 to disable throttling.
        self.declare_parameter('max_fps', 8.0)
        # Ultralytics YOLO inference image size (int or [h,w]).
        # Smaller reduces compute; model will internally letterbox.
        self.declare_parameter('imgsz', 640)

        self.declare_parameter('debug_max_width', 320)
        self.declare_parameter('debug_max_height', 240)

        self.model_path = self.get_parameter('model_path').value
        self.classes = self.get_parameter('classes').value
        self.conf_thresh = float(self.get_parameter('conf_thresh').value)
        self.camera_name = self.get_parameter('camera_name').value
        self.calibration = bool(self.get_parameter('calibration').value)
        self.debug_display = bool(self.get_parameter('debug_display').value)
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.imgsz = self.get_parameter('imgsz').value
        self.debug_max_width = int(self.get_parameter('debug_max_width').value)
        self.debug_max_height = int(self.get_parameter('debug_max_height').value)
        self.debug_window = 'yolo'
        self.debug_window_open = False
        self.last_log_time = 0.0
        self.last_enqueue_time = 0.0

        self.shape_pub = self.create_publisher(String, '/yolov5/shape', 10)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)

        self.create_service(Trigger, '/yolov5/start', self.start_srv_callback)
        self.create_service(Trigger, '/yolov5/stop', self.stop_srv_callback)
        self.create_service(Trigger, '/yolov5/calibration', self.calibration_srv_callback)

        self.get_logger().info('YOLO node ready')

    # ------------------------------------------------------------------ #
    def _publish_shape(self, value: str):
        if value != self.latest_shape:
            self.latest_shape = value
            msg = String()
            msg.data = value
            self.shape_pub.publish(msg)

    def _clear_queue(self):
        while not self.image_queue.empty():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                break

    # ------------------------------------------------------------------ #
    def calibration_srv_callback(self, _: Trigger.Request, response: Trigger.Response):
        self.calibration = True
        response.success = True
        response.message = 'calibration mode enabled'
        return response

    def start_srv_callback(self, _: Trigger.Request, response: Trigger.Response):
        if self.active:
            response.success = True
            response.message = 'already running'
            return response

        model_path = self.model_path
        if not os.path.isabs(model_path):
            model_path = os.path.join(MODE_PATH, model_path)
        if not os.path.exists(model_path):
            response.success = False
            response.message = f'model file missing: {model_path}'
            return response

        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model loaded: {model_path}')
        except Exception as exc:
            response.success = False
            response.message = f'failed to load model: {exc}'
            return response

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = f'/{self.camera_name}/rgb/image_raw'
        self.image_sub = self.create_subscription(Image, topic, self.image_callback, qos)

        self.active = True
        self._clear_queue()
        self._publish_shape('None')
        response.success = True
        response.message = 'yolo started'
        return response

    def stop_srv_callback(self, _: Trigger.Request, response: Trigger.Response):
        self.get_logger().info('stop yolo detect')
        self.active = False
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        self._clear_queue()
        self.model = None
        self._close_debug_window()
        response.success = True
        response.message = 'yolo stopped'
        return response

    # ------------------------------------------------------------------ #
    def image_callback(self, ros_image: Image):
        if not self.active or self.model is None:
            return
        if self.max_fps > 0.0:
            now = time.time()
            min_period = 1.0 / self.max_fps
            if (now - self.last_enqueue_time) < min_period:
                return
            self.last_enqueue_time = now
        rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data,
        )
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        try:
            self.image_queue.put_nowait(bgr_image)
        except queue.Full:
            try:
                _ = self.image_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.image_queue.put_nowait(bgr_image)
            except queue.Full:
                pass

    def _inference_worker(self):
        while not self.worker_stop.is_set():
            try:
                frame = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self.active or self.model is None:
                continue
            self._run_inference(frame)

    def _run_inference(self, frame: np.ndarray):
        need_visual = bool(self.calibration) or bool(self.debug_display)
        annotated = frame.copy() if need_visual else frame

        try:
            results = self.model(
                frame,
                conf=self.conf_thresh,
                imgsz=self.imgsz,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        debug_frame = annotated
        if not results or results[0].boxes is None or results[0].boxes.xyxy.shape[0] == 0:
            self._publish_shape('None')
            if self.calibration:
                ros_img = common.cv2_image2ros(frame, frame_id='yolo')
                self.result_image_pub.publish(ros_img)
            self._show_debug_frame(debug_frame)
            return

        boxes = results[0].boxes.xyxy.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)

        best_idx = int(np.argmax(confidences))
        cls_id = class_ids[best_idx]
        shape_name = self.classes[cls_id] if 0 <= cls_id < len(self.classes) else 'unknown'
        self._publish_shape(shape_name)
        now = time.time()
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f'YOLO检测: {shape_name} (conf={confidences[best_idx]:.2f})')
            self.last_log_time = now

        if need_visual:
            for box, conf, cid in zip(boxes, confidences, class_ids):
                x1, y1, x2, y2 = box.astype(int)
                label = f"{self.classes[cid] if cid < len(self.classes) else 'cls'}:{conf:.2f}"
                common.plot_one_box(
                    [x1, y1, x2, y2],
                    annotated,
                    color=common.colors(cid, True),
                    label=label,
                )

        if self.calibration:
            ros_img = common.cv2_image2ros(annotated, frame_id='yolo')
            self.result_image_pub.publish(ros_img)
        self._show_debug_frame(annotated)

    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self.worker_stop.set()
        super().destroy_node()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        self._close_debug_window()

    def _resize_for_display(self, frame: np.ndarray) -> np.ndarray:
        """根据配置将调试画面缩放到合适大小，避免显示不全。"""
        if frame is None:
            return None
        h, w = frame.shape[:2]
        max_w = max(1, int(self.debug_max_width))
        max_h = max(1, int(self.debug_max_height))
        scale = min(max_w / float(w), max_h / float(h), 1.0)
        if scale >= 0.999:
            return frame
        new_size = (max(1, int(w * scale)), max(1, int(h * scale)))
        return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)

    def _show_debug_frame(self, frame):
        if not self.debug_display:
            return
        frame = self._resize_for_display(frame)
        if frame is None:
            return
        if not self.debug_window_open:
            cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
            self.debug_window_open = True
        cv2.imshow(self.debug_window, frame)
        cv2.waitKey(1)

    def _close_debug_window(self):
        if self.debug_display and self.debug_window_open:
            try:
                cv2.destroyWindow(self.debug_window)
            except cv2.error:
                pass
            self.debug_window_open = False


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down yolo node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
