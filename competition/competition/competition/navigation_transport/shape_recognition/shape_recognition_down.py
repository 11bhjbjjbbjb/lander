#!/usr/bin/env python3
# coding=utf8

from sklearn.linear_model import LinearRegression  
import os
import cv2
from . import tone
import math
import queue
import time
import threading
import numpy as np
import sdk.common as common
import message_filters
import transforms3d as tfs
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool, Trigger
from servo_controller_msgs.msg import ServosPosition
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from sdk import pid, fps
from servo_controller.bus_servo_control import set_servo_position
from kinematics import kinematics_control


def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat


def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(
            math.radians(euler[0]),
            math.radians(euler[1]),
            math.radians(euler[2]),
        )
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat


def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler


def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


class RgbDepthImageNode(Node):
    def __init__(self):
        super().__init__('shape_recognition')

        share_dir = get_package_share_directory('competition')
        self.config_path = os.path.join(share_dir, 'config', 'config.yaml')
        cfg = common.get_yaml_data(self.config_path)
        if cfg is None:
            self.get_logger().warn(f'配置文件 {self.config_path} 加载失败，使用默认参数')
            cfg = {}
        self.config = cfg
        self.declare_parameter('debug_display', False)
        self.declare_parameter('target_shape', 'box')
        self.debug_display = bool(self.get_parameter('debug_display').value)
        self.debug_window = 'shape_recognition_debug'
        self.debug_window_open = False
        self.debug_frame = None
        self.debug_frame_lock = threading.Lock()
        self.debug_timer = None
        self.debug_active = False
        self.last_detect_log = 0.0
        self.last_idle_log = 0.0
        self.last_camera_info_warn = 0.0
        self.last_endpoint_warn = 0.0
        offset = cfg.get('offset', [0.0, 0.0, 0.0])
        self.shape_dist = cfg.get('shape_dist', 220.0)
        self.pick_location_time = cfg.get('pick_location_time', [3.0])
        self.offset_x, self.offset_y, self.offset_z = offset
        sf = cfg.get('shape_flat', [1.0, 1.0])
        self.shape_flat = list(sf) if isinstance(sf, (list, tuple)) and len(sf) >= 2 else [1.0, 1.0]

        self.fps = fps.FPS()
        self.last_shape = 'none'
        self.queue = queue.Queue(maxsize=1)
        self.moving = False
        self.count = 0
        self.close = False
        self.endpoint = None
        self.shape = None
        self.calibration_flat = False
        self.calibration_dist = False
        self.pick_state = False
        self.status = 'start'
        self.target_shape = self.get_parameter('target_shape').get_parameter_value().string_value
        self.current_shape_msg = 'None'
        self.depth_camera_info = None
        self.camera_info_lock = threading.Lock()
        self._async_tasks = {}
        self._async_lock = threading.Lock()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.status_pub = self.create_publisher(String, '/shape_recognition/status', 10)
        self.shape_pub = self.create_publisher(String, '/shape_recognition/shape', 10)
        self.target_shape_sub = self.create_subscription(
            String,
            '/shape_recognition/target_shape',
            self.target_shape_callback,
            10,
        )

        self.ldp_client = None
        self.ldp_service_name = None
        self.pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.pose_target_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')

        self.create_service(Trigger, '~/pick', self.pick_callback)
        self.create_service(Trigger, '~/calibration_flat', self.calibration_flat_callback)
        self.create_service(Trigger, '~/calibration_dist', self.calibration_dist_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(Trigger, '~/start', self.start_callback)
        self.create_service(Trigger, '~/close', self.close_callback)

        def _init_and_disable_ldp():
            time.sleep(0.5)  # 等待节点 spin 启动
            self.get_logger().info('后台线程：开始初始化 LDP 服务')
            self.ldp_client, self.ldp_service_name = self._init_ldp_service()
            if self.ldp_client and self.ldp_service_name:
                self.get_logger().info(f'后台线程：找到 LDP 服务 {self.ldp_service_name}')
                req = SetBool.Request()
                req.data = False
                result = self.call_service(self.ldp_client, req, timeout=5.0)
                if result:
                    self.get_logger().info(f'{self.ldp_service_name} 已关闭 LDP')
                else:
                    self.get_logger().warn(f'关闭 {self.ldp_service_name} LDP 失败，超时或服务不可用')
            else:
                self.get_logger().warn('后台线程：未找到 LDP 服务，跳过 LDP 关闭')

        threading.Thread(target=_init_and_disable_ldp, daemon=True).start()

        self.hand2cam_tf_matrix = [
            [0.0, 0.0, 1.0, -0.105],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.044],
            [0.0, 0.0, 0.0, 1.0],
        ]

        self.line_compensation = LinearRegression()
        self.line_compensation.fit([[20], [200], [350]], [[self.shape_flat[0]], [1], [self.shape_flat[1]]])
        self.line_depth_compensation = [self.line_compensation.predict([[i]]) for i in range(400)]

        self.rgb_sub = None
        self.depth_sub = None
        self.sync = None
      
        qos_cam = QoSProfile(depth=1)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/gemini_camera/depth/camera_info',
            self.camera_info_callback,
            qos_cam,
        )

        threading.Thread(target=self.goto_default, daemon=True).start()
        if self.debug_display:
            self.debug_timer = self.create_timer(0.05, self._debug_timer_cb)
        self.publish_status('start')
        self.get_logger().info('shape_recognition node ready')

        # Enable dynamic updates for parameters like:
        #   ros2 param set /shape_recognition target_shape cylinder
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        try:
            for p in params:
                if p.name == 'target_shape':
                    new_val = str(p.value)
                    if not new_val:
                        return SetParametersResult(successful=False, reason='target_shape cannot be empty')
                    self.target_shape = new_val
                elif p.name == 'debug_display':
                    new_debug = bool(p.value)
                    self.debug_display = new_debug
                    # Manage debug timer/window without requiring restart.
                    if self.debug_display and self.debug_timer is None:
                        self.debug_timer = self.create_timer(0.05, self._debug_timer_cb)
                    if not self.debug_display:
                        if self.debug_timer is not None:
                            try:
                                self.debug_timer.cancel()
                            except Exception:
                                pass
                            self.debug_timer = None
                        self._close_debug_window()
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    # 基础工具
    # ------------------------------------------------------------------ #
    def publish_status(self, value):
        self.status = value
        self.status_pub.publish(String(data=value))

    def publish_shape(self, value):
        if self.current_shape_msg != value:
            self.current_shape_msg = value
            self.shape_pub.publish(String(data=value))

    def target_shape_callback(self, msg: String):
        self.target_shape = msg.data    

    def camera_info_callback(self, msg: CameraInfo):
        with self.camera_info_lock:
            self.depth_camera_info = msg

    def _get_camera_info(self):
        with self.camera_info_lock:
            return self.depth_camera_info

    def call_service(self, client, request, timeout=None):
        if rclpy.ok():
            future = client.call_async(request)
            start = time.time()
            while rclpy.ok():
                if future.done():
                    return future.result()
                if timeout and (time.time() - start) > timeout:
                    self.get_logger().warn('服务调用超时')
                    return None
                time.sleep(0.05)
            return None
        else:
            return None

    def set_servos(self, duration, positions):
        set_servo_position(self.servos_pub, float(duration), tuple(positions))

    def _run_async_task(self, key, description, target):
        def runner():
            try:
                target()
            except Exception as exc:
                self.get_logger().error(f'{description} 执行异常: {exc}')
            finally:
                with self._async_lock:
                    self._async_tasks.pop(key, None)

        with self._async_lock:
            existing = self._async_tasks.get(key)
            if existing and existing.is_alive():
                self.get_logger().warn(f'{description} 已在执行，忽略重复请求')
                return False
            thread = threading.Thread(target=runner, daemon=True)
            self._async_tasks[key] = thread
            thread.start()
            return True

    def _update_line_compensation(self):
        self.line_compensation.fit(
            [[20], [200], [350]],
            [[self.shape_flat[0]], [1], [self.shape_flat[1]]],
        )
        self.line_depth_compensation = [
            self.line_compensation.predict([[i]]) for i in range(400)
        ]

    def destroy_subscribers(self):
        for attr in ('rgb_sub', 'depth_sub'):
            sub = getattr(self, attr)
            if sub and hasattr(sub, 'sub'):
                sub.sub.destroy()
            setattr(self, attr, None)

        self.sync = None
        with self.debug_frame_lock:
            self.debug_frame = None

    def destroy_node(self):
        self._close_debug_window()
        super().destroy_node()

    def _store_debug_frame(self, frame):
        if not (self.debug_display and self.debug_active):
            return
        with self.debug_frame_lock:
            self.debug_frame = frame.copy()

    def _debug_timer_cb(self):
        if not (self.debug_display and self.debug_active):
            return
        with self.debug_frame_lock:
            frame = None if self.debug_frame is None else self.debug_frame.copy()
        if frame is None:
            return
        if not self.debug_window_open:
            cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
            self.debug_window_open = True
        cv2.imshow(self.debug_window, frame)
        cv2.waitKey(1)

    def _close_debug_window(self):
        if self.debug_window_open:
            try:
                cv2.destroyWindow(self.debug_window)
                cv2.waitKey(1)
            except cv2.error:
                pass
            self.debug_window_open = False
        with self.debug_frame_lock:
            self.debug_frame = None

    def _log_detection(self, shape, depth_mm, center, angle, area, num_contours=None, min_dist_mm=None):
        now = time.time()
        if shape != 'None':
            if now - self.last_detect_log < 0.5:
                return
            self.last_detect_log = now
            depth_cm = (float(depth_mm) / 10.0) if depth_mm is not None else 0.0
            if center is not None:
                cx, cy = float(center[0]), float(center[1])
            else:
                cx, cy = -1.0, -1.0
            self.get_logger().info(
                f'[形状识别] shape={shape} depth={depth_cm:.1f}cm '
                f'center=({int(cx)},{int(cy)}) area={float(area):.0f} angle={float(angle):.1f}'
            )
        else:
            if now - self.last_idle_log < 1.5:
                return
            self.last_idle_log = now
            hint = ''
            if num_contours is not None or min_dist_mm is not None:
                parts = []
                if num_contours is not None:
                    parts.append(f'轮廓数={num_contours}')
                if min_dist_mm is not None:
                    parts.append(f'最近距离={min_dist_mm:.0f}mm(shape_dist≈{self.shape_dist:.0f})')
                if parts:
                    hint = '（' + ' '.join(parts) + '）'
            # self.get_logger().info(f'[形状识别] 未检测到目标，继续等待{hint}')

    def _init_ldp_service(self, timeout=10.0):
        candidates = ['/gemini_camera/set_ldp', '/gemini_camera/set_ldp_enable']
        clients = [(name, self.create_client(SetBool, name)) for name in candidates]
        selected = None
        start_time = time.time()

        while selected is None:
            # 检查超时
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'等待 LDP 服务超时（{timeout}秒），服务可能不可用')
                return None, None

            for name, client in clients:
                if client.wait_for_service(timeout_sec=0.5):
                    selected = (client, name)
                    break
            if selected is None:
                self.get_logger().info('等待相机 LDP 服务...')
        return selected

    # ------------------------------------------------------------------ #
    # 服务回调
    # ------------------------------------------------------------------ #
    def start_callback(self, request, response):
        qos = QoSProfile(depth=1)
        self.rgb_sub = message_filters.Subscriber(
            self, RosImage, '/gemini_camera/rgb/image_raw', qos_profile=qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, RosImage, '/gemini_camera/depth/image_raw', qos_profile=qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1
        )
        self.sync.registerCallback(self.multi_callback)
        self.debug_active = True
        response.success = True
        return response

    def stop_callback(self, request, response):
        self.get_logger().info('停止形状识别')
        self.destroy_subscribers()
        self.debug_active = False
        self._close_debug_window()
        response.success = True
        return response

    def close_callback(self, request, response):
        self.get_logger().info('关闭形状识别')
        self.close = True
        self.destroy_subscribers()
        self.debug_active = False
        self._close_debug_window()
        response.success = True
        return response

    def pick_callback(self, request, response):
        self.get_logger().info(f'夹取流程：target_shape={self.target_shape}，机械臂进入向下识别姿态')
        self.set_servos(1.0, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
        time.sleep(2)
        self.pick_state = True
        self.publish_status('pick')
        response.success = True
        return response

    def calibration_dist_callback(self, request, response):
        self.target_shape = 'None'
        self.calibration_dist = True
        self.set_servos(1.0, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
        time.sleep(2)
        response.success = True
        return response

    def calibration_flat_callback(self, request, response):
        self.target_shape = 'box'
        self.calibration_flat = True
        self.set_servos(1.0, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
        time.sleep(2)
        response.success = True
        return response

    # ------------------------------------------------------------------ #
    def goto_default(self):
        while rclpy.ok() and not self.close:
            req = GetRobotPose.Request()
            result = self.call_service(self.pose_client, req, timeout=None)
            if result and result.success:
                pose_t = result.pose.position
                pose_r = result.pose.orientation
                self.endpoint = xyz_quat_to_mat(
                    [pose_t.x, pose_t.y, pose_t.z],
                    [pose_r.w, pose_r.x, pose_r.y, pose_r.z],
                )
            time.sleep(0.5)

    def multi_callback(self, ros_rgb_image, ros_depth_image):
        if self.close or self.moving:
            return
        depth_camera_info = self._get_camera_info()
        if depth_camera_info is None:
            now = time.time()
            if now - self.last_camera_info_warn > 2.0:
                self.get_logger().warn('等待 /gemini_camera/depth/camera_info 数据...')
                self.last_camera_info_warn = now
            return
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image, depth_camera_info))
            threading.Thread(target=self.image_proc, daemon=True).start()

    def move(self, shape, pose_t, angle):
        time.sleep(0.5)
        pose_up = pose_t.copy()
        pose_up[2] += 0.02
        ret1 = self._solve_pose_target(pose_up, 85)
        if not self._is_valid_pose(ret1):
            self._handle_move_failure('上升位姿求解失败')
            return
        positions = tuple((idx + 1, int(ret1.pulse[idx])) for idx in range(len(ret1.pulse)))
        self.set_servos(1.5, positions)
        time.sleep(1.5)

        pose_down = pose_up.copy()
        pose_down[2] -= 0.05
        ret2 = self._solve_pose_target(pose_down, 85)
        if not self._is_valid_pose(ret2):
            self._handle_move_failure('下降位姿求解失败')
            return

        rpy = getattr(ret2, 'rpy', None)
        if angle != 0 and rpy and len(rpy) > 0:
            angle = angle % 180
            angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
            angle = 500 + int(1000 * (angle + rpy[-1]) / 240)
        else:
            angle = 500

        self.set_servos(0.5, ((5, angle),))
        time.sleep(0.5)

        positions = tuple(
            (idx + 1, int(ret2.pulse[idx]))
            for idx in range(len(ret2.pulse))
            if (idx + 1) != 5
        )
        self.set_servos(1.0, positions)
        time.sleep(1.0)
        self.set_servos(0.6, ((10, 750),))
        time.sleep(0.6)

        positions = tuple(
            (idx + 1, int(ret1.pulse[idx]))
            for idx in range(len(ret1.pulse))
            if (idx + 1) != 5
        )
        self.set_servos(1.0, positions)
        time.sleep(1.0)

        self.set_servos(1.0, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
        time.sleep(1.0)
        self.publish_status('stop')
        self.pick_state = False
        self.moving = False

    def _solve_pose_target(self, position, pitch):
        req = kinematics_control.set_pose_target(position, pitch)
        return self.call_service(self.pose_target_client, req, timeout=2.0)

    def _is_valid_pose(self, result):
        return result is not None and getattr(result, 'success', False) and len(result.pulse) > 0

    def _handle_move_failure(self, reason):
        self.get_logger().warn(f'[形状识别] {reason}，终止本次夹取')
        self.publish_status('stop')
        self.pick_state = False
        self.moving = False

    def _try_fetch_endpoint_once(self, timeout=1.0):
        req = GetRobotPose.Request()
        result = self.call_service(self.pose_client, req, timeout=timeout)
        if result is not None and getattr(result, 'success', False) and getattr(result, 'solution', True):
            pose_t = result.pose.position
            pose_r = result.pose.orientation
            self.endpoint = xyz_quat_to_mat(
                [pose_t.x, pose_t.y, pose_t.z],
                [pose_r.w, pose_r.x, pose_r.y, pose_r.z],
            )
            return True
        return False

    def get_queue(self):
        try:
            return self.queue.get(timeout=0.1)
        except queue.Empty:
            return None

    def image_proc(self):
        data = self.get_queue()
        if data is None:
            return
        ros_rgb_image, ros_depth_image, depth_camera_info = data

        try:
            if self.pick_state:
                rgb_image = np.ndarray(
                    shape=(ros_rgb_image.height, ros_rgb_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_rgb_image.data,
                )
                depth_image = np.ndarray(
                    shape=(ros_depth_image.height, ros_depth_image.width),
                    dtype=np.uint16,
                    buffer=ros_depth_image.data,
                )

                ih, iw = depth_image.shape[:2]
                depth_image = depth_image.copy()
                n_comp = min(len(self.line_depth_compensation), ih)
                for j in range(n_comp):
                    depth_image[j] = depth_image[j] * self.line_depth_compensation[j]

                margin = 50
                depth_image[:, 0:min(margin, iw)] = 1000
                if iw > margin:
                    depth_image[:, max(0, iw - margin) : iw] = 1000
                if ih > 80:
                    depth_image[max(0, ih - 80) : ih, :] = 1000

                depth = np.copy(depth_image).reshape((-1,))
                depth[depth <= 0] = 55555
                min_index = np.argmin(depth)
                min_y = int(min_index // iw)
                min_x = int(min_index - min_y * iw)
                min_dist = float(depth_image[min_y, min_x])

                sim_depth_image = np.clip(depth_image, 0, 300).astype(np.float64) / 300 * 255
                depth_image = np.where(depth_image > min_dist + 17, 0, depth_image)
                sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
                depth_gray = sim_depth_image_sort.astype(np.uint8)
                depth_gray = cv2.GaussianBlur(depth_gray, (3, 3), 0)
                _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
                depth_bit = cv2.erode(depth_bit, np.ones((3, 3), np.uint8))
                depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))

                contours, _ = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                shape = 'None'
                contour = None
                log_center = None
                log_angle = 0.0
                log_area = 0.0
                log_depth = 0.0
                for obj in contours:
                    if min_dist > self.shape_dist - 2:
                        break
                    area = cv2.contourArea(obj)
                    if area < 3000 or area > 15000 or self.moving:
                        continue
                    perimeter = cv2.arcLength(obj, True)
                    approx = cv2.approxPolyDP(obj, 0.04 * perimeter, True)
                    corner_num = len(approx)

                    x, y, w, h = cv2.boundingRect(approx)
                    x_contour_depth = depth_image[
                        y + int(h / 2) - 2: y + int(h / 2) + 2, x: x + w
                    ]
                    y_contour_depth = depth_image[
                        y: y + h, x + int(w / 2) - 2: x + int(w / 2) + 2
                    ]
                    x_depth = np.where(x_contour_depth == 0, np.nan, x_contour_depth)
                    y_depth = np.where(y_contour_depth == 0, np.nan, y_contour_depth)
                    x_depth_std = np.nanstd(x_depth)
                    y_depth_std = np.nanstd(y_depth)

                    obj_type = 'None'
                    if x_depth_std <= 0.9 and y_depth_std <= 1.15 and corner_num == 4:
                        obj_type = 'cuboid_1'
                        self.shape = 'box'
                    else:
                        if (abs(w / h) > 1.7 or abs(h / w) > 1.7) and corner_num == 4:
                            if x_depth_std <= 0.9 and y_depth_std <= 1.15:
                                obj_type = 'cuboid_2'
                                self.shape = 'box'
                            else:
                                if w > 120 or h > 120:
                                    obj_type = 'cuboid_3'
                                    self.shape = 'box'
                                else:
                                    obj_type = 'cylinder_2'
                                    self.shape = 'cylinder'
                        else:
                            if abs(x_depth_std - y_depth_std) > 0.5:
                                if x_depth_std <= 0.9 and y_depth_std <= 1.15:
                                    obj_type = 'cuboid_4'
                                    self.shape = 'box'
                                else:
                                    if w > 120 or h > 120:
                                        obj_type = 'cuboid_5'
                                        self.shape = 'box'
                                    else:
                                        obj_type = 'cylinder_3'
                                        self.shape = 'cylinder'
                            else:
                                if w > 120 or h > 120:
                                    obj_type = 'cuboid_6'
                                    self.shape = 'box'
                                else:
                                    obj_type = 'cylinder_4'
                                    self.shape = 'cylinder'

                    shape = obj_type
                    contour = obj
                    if obj_type != 'None':
                        log_center = (int(x + w / 2), int(y + h / 2))
                        log_area = area
                        log_depth = float(min_dist)
                    if 'cubo' in obj_type:
                        if abs(w - h) < 10:
                            if w > 100 or h > 100:
                                obj_type = 'cube_1'
                                self.shape = 'cube'
                            else:
                                obj_type = 'box_1'
                                self.shape = 'box'
                        else:
                            obj_type = 'cube_2'
                            self.shape = 'cube'

                    if self.shape == self.target_shape:
                        break
                    else:
                        self.shape = 'None'

                if self.last_shape == shape and shape != 'None' and self.shape == self.target_shape:
                    self.count += 1
                    self.shape = 'None'

                if contour is not None:
                    angle = 0.0
                    if shape in ('cylinder_1', 'cuboid_1'):
                        _center, (_w, _h), angle = cv2.minAreaRect(contour)
                        angle = float(angle)
                        width, height = float(_w), float(_h)
                        if angle < -45:
                            angle += 90
                        if width > height and width / height > 1.5:
                            angle = angle + 90
                    log_angle = float(angle)
                    if self.count > 3:
                        if self.endpoint is None:
                            self._try_fetch_endpoint_once(timeout=1.0)
                        if self.endpoint is None:
                            now = time.time()
                            if now - self.last_endpoint_warn > 1.0:
                                self.get_logger().warn('等待机械臂当前位姿，暂不执行移动')
                                self.last_endpoint_warn = now
                            self.shape = 'None'
                            self.publish_shape('None')
                            self.count = 0
                            contour = None
                        else:
                            (cx, cy), _ = cv2.minEnclosingCircle(contour)
                            log_center = (int(cx), int(cy))
                            K = getattr(depth_camera_info, 'k', None)
                            if K is None:
                                K = getattr(depth_camera_info, 'K', None)
                            if K is None or len(K) < 9:
                                self.get_logger().warn('camera_info 内参 K 无效，跳过本次夹取')
                                self.count = 0
                                contour = None
                            else:
                                position = depth_pixel_to_camera(
                                    (cx, cy), min_dist / 1000.0, (K[0], K[4], K[2], K[5])
                                )
                                position[0] -= 0.0171
                                pose_end = np.matmul(self.hand2cam_tf_matrix, xyz_euler_to_mat(position, (0, 0, 0)))
                                world_pose = np.matmul(self.endpoint, pose_end)
                                pose_t, _ = mat_to_xyz_euler(world_pose)
                                pose_t[0] += self.offset_x
                                pose_t[1] += self.offset_y
                                pose_t[2] += self.offset_z
                                self.count = 0
                                self.moving = True
                                s, pt, a = shape[:-2], pose_t, angle
                                if not self._run_async_task('move', '夹取移动', lambda: self.move(s, pt, a)):
                                    self.moving = False
                                self.publish_shape(self.shape)
                    else:
                        self.publish_shape('None')

                self.last_shape = shape
                self._log_detection(
                    shape,
                    log_depth,
                    log_center,
                    log_angle,
                    log_area,
                    num_contours=len(contours) if contours is not None else 0,
                    min_dist_mm=min_dist if min_dist is not None else None,
                )
                if self.debug_display:
                    debug_frame = rgb_image.copy()
                    if contour is not None:
                        cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 2)
                    self._store_debug_frame(debug_frame)
                self.fps.update()

            elif self.calibration_flat:
                self.calibration_flat = False
                ih, iw = ros_depth_image.height, ros_depth_image.width
                depth_image = np.ndarray(
                    shape=(ih, iw),
                    dtype=np.uint16,
                    buffer=ros_depth_image.data,
                )
                if ih < 351 or iw < 10:
                    self.get_logger().warn('深度图尺寸过小，无法做平面校准')
                else:
                    calibration_20 = float(np.max(depth_image[200]) / (np.max(depth_image[20]) + 1e-6))
                    calibration_350 = float(np.max(depth_image[200]) / (np.max(depth_image[350]) + 1e-6))
                    calibration_compensation = LinearRegression()
                    calibration_compensation.fit([[20], [200], [350]], [[calibration_20], [1], [calibration_350]])
                    n_rows = min(400, ih)
                    calibration_depth_compensation = [
                        calibration_compensation.predict([[i]]) for i in range(n_rows)
                    ]
                    transition_depth_image = np.zeros((ih, iw), dtype=float)
                    for j in range(min(399, ih - 1)):
                        transition_depth_image[j] = depth_image[j] * calibration_depth_compensation[j]

                    if self.config is None:
                        self.config = self._load_config()
                    if self.config is None:
                        self.get_logger().error('配置文件不可用，平面校准已取消')
                    else:
                        if self.config.get('shape_flat') is None or not isinstance(self.config.get('shape_flat'), (list, tuple)) or len(self.config['shape_flat']) < 2:
                            self.config['shape_flat'] = [1.0, 1.0]
                        self.config['shape_flat'][0] = calibration_20
                        self.config['shape_flat'][1] = calibration_350
                        common.save_yaml_data(self.config, self.config_path)
                        self.shape_flat = [self.config['shape_flat'][0], self.config['shape_flat'][1]]
                        self._update_line_compensation()
                        self.get_logger().info(f'[校准] 平面校准已保存: shape_flat={self.shape_flat} -> {self.config_path}')
                        self.get_logger().info('[校准] 已退出平面校准，订阅保持；可调用 ~/stop 停止识别')
                    # 与 automatic_pick 一致：校准后不销毁订阅，避免 InvalidHandle

            elif self.calibration_dist:
                self.calibration_dist = False
                depth_image = np.ndarray(
                    shape=(ros_depth_image.height, ros_depth_image.width),
                    dtype=np.uint16,
                    buffer=ros_depth_image.data,
                )
                ih, iw = depth_image.shape[:2]
                depth_image = depth_image.copy()
                n_comp = min(len(self.line_depth_compensation), ih)
                for j in range(n_comp):
                    depth_image[j] = depth_image[j] * self.line_depth_compensation[j]
                margin = 50
                depth_image[:, 0:min(margin, iw)] = 1000
                if iw > margin:
                    depth_image[:, max(0, iw - margin) : iw] = 1000
                if ih > 80:
                    depth_image[max(0, ih - 80) : ih, :] = 1000
                depth = np.copy(depth_image).reshape((-1,))
                depth[depth <= 0] = 55555
                min_index = np.argmin(depth)
                min_y = int(min_index // iw)
                min_x = int(min_index - min_y * iw)
                min_dist = float(depth_image[min_y, min_x])

                if self.config is None:
                    self.config = self._load_config()
                if self.config is None:
                    self.get_logger().error('配置文件不可用，距离校准已取消')
                else:
                    self.config['shape_dist'] = float(min_dist)
                    common.save_yaml_data(self.config, self.config_path)
                    self.shape_dist = self.config['shape_dist']
                    self.get_logger().info(f'[校准] 距离校准已保存: shape_dist={self.shape_dist:.0f}mm -> {self.config_path}')
                    self.get_logger().info('[校准] 已退出距离校准，订阅保持；可调用 ~/stop 停止识别')
                # 与 automatic_pick 一致：校准后不销毁订阅，避免 InvalidHandle

            else:
                time.sleep(0.01)

        except Exception as exc:
            tb = traceback.format_exc()
            self.get_logger().error(f'callback error: {exc}\n{tb}')


def main(args=None):
    rclpy.init(args=args)
    node = RgbDepthImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('shape_recognition shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
