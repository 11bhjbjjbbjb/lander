#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2
import time
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String

from sdk.pid import PID
from cv_bridge import CvBridge
import sdk.common as common
from xf_mic_asr_offline import voice_play
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition


def _extract_front_scan(msg, half_angle_deg=15.0, front_center_deg=180.0):
    half_rad = math.radians(half_angle_deg)
    center_rad = math.radians(front_center_deg)
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    ranges = msg.ranges
    range_min = msg.range_min
    range_max = msg.range_max
    angles_out, ranges_out = [], []
    for i, r in enumerate(ranges):
        angle_rad = angle_min + i * angle_increment
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        diff = angle_rad - center_rad
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        if abs(diff) > half_rad:
            continue
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        angles_out.append(angle_rad)
        ranges_out.append(r)
    return angles_out, ranges_out


def _polar_to_cartesian(angles_rad, ranges):
    a = np.asarray(angles_rad, dtype=float)
    r = np.asarray(ranges, dtype=float)
    x = r * np.cos(a)
    y = r * np.sin(a)
    return x, y


def _fit_line(angles_rad, ranges):
    if len(ranges) < 2:
        return None, None, None
    x, y = _polar_to_cartesian(angles_rad, ranges)
    x, y = np.asarray(x), np.asarray(y)
    cx, cy = float(np.mean(x)), float(np.mean(y))
    cov = np.cov(x, y)
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
        direction = eigvecs[:, np.argmax(eigvals)].flatten()
    except np.linalg.LinAlgError:
        return None, None, None
    t = (x - cx) * direction[0] + (y - cy) * direction[1]
    t_min, t_max = float(np.min(t)), float(np.max(t))
    x_ends = np.array([cx + t_min * direction[0], cx + t_max * direction[0]])
    y_ends = np.array([cy + t_min * direction[1], cy + t_max * direction[1]])
    return x_ends, y_ends, direction


def _alignment_deviation_rad(direction):
    if direction is None or len(direction) < 2:
        return None
    dx, dy = float(direction[0]), float(direction[1])
    theta = math.atan2(dy, dx)
    if theta < 0:
        theta += math.pi
    deviation = theta - math.pi / 2
    if deviation > math.pi:
        deviation -= 2 * math.pi
    if deviation <= -math.pi:
        deviation += 2 * math.pi
    return deviation


def _is_outlier(depth_val, center_avg, invalid_depth=0, depth_tolerance=10):
    if depth_val == invalid_depth or np.isnan(depth_val):
        return True
    return depth_val > center_avg + depth_tolerance or depth_val < center_avg - depth_tolerance


def check_ramp_alignment(
    depth_image,
    align_center_x,
    align_center_y,
    center_window=10,
    outlier_count_threshold=3,
    invalid_depth=0,
    ):
    if depth_image is None or depth_image.size == 0:
        return None, None
    h, w = depth_image.shape
    if align_center_y < 0 or align_center_y >= h:
        return None, None

    vertical_half = 2 
    center_x_list = []
    center_depth_list = []

    half = center_window // 2
    left_idx = max(0, align_center_x - half)
    right_idx = min(w, align_center_x + half)

    for row_y in range(align_center_y - vertical_half, align_center_y + vertical_half + 1):
        if row_y < 0 or row_y >= h:
            continue
        row = np.asarray(depth_image[row_y, :], dtype=np.float64)
        center_slice = row[left_idx:right_idx]
        valid = center_slice[(center_slice > invalid_depth) & np.isfinite(center_slice)]
        if valid.size == 0:
            continue
        center_avg = float(np.mean(valid))

        left_bound, count_left = 0, 0
        for x in range(align_center_x - 1, -1, -1):
            if _is_outlier(row[x], center_avg, invalid_depth=invalid_depth):
                count_left += 1
                if count_left >= outlier_count_threshold:
                    left_bound = x
                    break
        else:
            left_bound = 0

        right_bound, count_right = w - 1, 0
        for x in range(align_center_x + 1, w):
            if _is_outlier(row[x], center_avg, invalid_depth=invalid_depth):
                count_right += 1
                if count_right >= outlier_count_threshold:
                    right_bound = x
                    break
        else:
            right_bound = w - 1

        if left_bound > right_bound:
            continue

        ramp_center_x_row = (left_bound + right_bound) / 2.0
        center_depth_val_row = row[int(round(ramp_center_x_row))]
        if center_depth_val_row == invalid_depth or not np.isfinite(center_depth_val_row):
            center_depth_val_row = None
        else:
            center_depth_val_row = float(center_depth_val_row)

        center_x_list.append(ramp_center_x_row)
        if center_depth_val_row is not None:
            center_depth_list.append(center_depth_val_row)

    if not center_x_list:
        return None, None

    ramp_center_x = float(np.mean(center_x_list))
    center_depth_val = float(np.mean(center_depth_list)) if center_depth_list else None
    offset_x = ramp_center_x - align_center_x
    return center_depth_val, offset_x


class RampNode(Node):
    size = (640, 400)

    def __init__(self):
        super().__init__('ramp')

        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        self.machine_type = os.environ.get('MACHINE_TYPE', '')

        share_dir = get_package_share_directory('competition')
        self.config_path = os.path.join(share_dir, 'config', 'config.yaml')
        cfg = common.get_yaml_data(self.config_path)
        if cfg is None:
            self.get_logger().warn(f'配置文件 {self.config_path} 加载失败，使用默认参数')
            cfg = {}
        self.config = cfg

        self.posture_align_tolerance_deg = float(cfg.get('posture_align_tolerance_deg', 5.0))
        self.ramp_align_offset_tolerance_px = int(cfg.get('ramp_align_offset_tolerance_px', 10))
        self.ramp_up_pixel_coordinate = cfg.get('ramp_up_pixel_coordinate', [400, 150])

        self.start_pick = False
        self.linear_base_speed = 0.007
        self.angular_base_speed = 0.005
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.yaw_angle = 0.0
        self.pick_stop_x = 320
        self.pick_stop_y = 388

        self.status = 'approach'
        self.count_stop = 0
        self.count_turn = 0
        self.calibration_ramp_center = False
        self.close = False
        self.stop_flag = True

        self.start_posture_align = False
        self._posture_align_stop_count = 0
        self.start_ramp_align_only = False
        self._ramp_align_stop_count = 0
        self._front_angle_deg = 15.0
        self._front_center_deg = 180.0
        self._last_deviation_rad = None
        self.bridge = CvBridge()
        self._color_frame_calibration = None
        self._calibration_window_name = 'ramp_center_calibration'
        self._calibration_display_timer = None
        self._worker_image = np.zeros((400, 640), dtype=np.uint16)
        self._worker_lock = threading.Lock()
        self._worker_event = threading.Event()
        self._worker_running = True
        self._worker_last_process = 0.0
        self._worker_min_interval = 1.0 / 20.0
        self.transition_depth_image = np.zeros((400, 640), dtype=float)

        self.linear_pid = PID(P=0.001, I=0, D=0)
        self.angular_pid = PID(P=0.0015, I=0, D=0)
        self.yaw_pid = PID(P=0.015, I=0, D=0.001)
        self.posture_pid = PID(P=0.3, I=0, D=0.02)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.status_pub = self.create_publisher(String, '/ramp/status', 10)

        self.ldp_client = None
        self.ldp_service_name = None

        def _init_and_disable_ldp():
            time.sleep(0.5)
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
        threading.Thread(target=self._worker_loop, daemon=True).start()

        self.image_sub = None
        self.scan_sub = None
        self.color_sub = None
        
        self.create_service(Trigger, '~/start', self.start_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(Trigger, '~/scan_start', self.scan_start_callback)
        self.create_service(Trigger, '~/scan_stop', self.scan_stop_callback)
        self.create_service(Trigger, '~/start_posture_align', self.start_posture_align_callback)
        self.create_service(Trigger, '~/calibration_ramp_center', self.calibration_ramp_center_callback)
        self.create_service(Trigger, '~/start_ramp_align', self.start_ramp_align_callback)
        self.create_service(Trigger, '~/up', self.up_callback)
        self._up_running = False

        self._calibration_display_timer = self.create_timer(0.05, self._calibration_display_tick)

        self.publish_status('stop')
        self.cmd_pub.publish(Twist())
        self.get_logger().info('ramp node ready')

    # ------------------------------------------------------------------ #
    def publish_status(self, value):
        self.status = value
        self.status_pub.publish(String(data=value))

    def set_servos(self, duration, positions):
        set_servo_position(self.joints_pub, float(duration), tuple(positions))

    def play(self, name):
        voice_play.play(name, language=self.language)

    def call_service(self, client, request, timeout=None):
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

    def destroy_image_sub(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None

    def destroy_scan_sub(self):
        if self.scan_sub is not None:
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None

    def destroy_color_sub(self):
        if self.color_sub is not None:
            self.destroy_subscription(self.color_sub)
            self.color_sub = None

    def _worker_loop(self):
        while self._worker_running:
            self._worker_event.wait(timeout=0.05)
            if not self._worker_running:
                break
            self._worker_event.clear()
            now = time.time()
            if now - self._worker_last_process < self._worker_min_interval:
                continue
            if not self.start_pick and not self.start_ramp_align_only:
                continue
            with self._worker_lock:
                depth_image = self._worker_image.copy()
            self._worker_last_process = now

            self.transition_depth_image[:] = depth_image
            if self.start_ramp_align_only:
                self._ramp_align_only_step()
            else:
                self.stop_flag = True
                self.ramp_align(self.transition_depth_image)

    def _init_ldp_service(self, timeout=10.0):
        candidates = ['/gemini_camera/set_ldp', '/gemini_camera/set_ldp_enable']
        clients = [(name, self.create_client(SetBool, name)) for name in candidates]
        selected = None
        start_time = time.time()
        while selected is None:
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
    def start_callback(self, request, response):
        if self.image_sub is None:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.image_sub = self.create_subscription(
                Image,
                '/gemini_camera/depth/image_raw',
                self.image_callback,
                qos,
            )
        response.success = True
        return response

    def _ensure_scan_subscription(self):
        if self.scan_sub is None:
            qos_scan = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.scan_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                qos_scan,
            )
            self.get_logger().info('雷达订阅已开启: /scan')

    def scan_start_callback(self, request, response):
        self._ensure_scan_subscription()
        response.success = True
        return response

    def scan_stop_callback(self, request, response):
        self.start_posture_align = False
        self._posture_align_stop_count = 0
        self.destroy_scan_sub()

        self.cmd_pub.publish(Twist())
        self.get_logger().info('雷达订阅已关闭: /scan，并停止姿态摆正')
        response.success = True
        return response

    def _reload_tolerance_config(self):
        cfg = common.get_yaml_data(self.config_path)
        if cfg is not None:
            self.posture_align_tolerance_deg = float(cfg.get('posture_align_tolerance_deg', 5.0))
            self.ramp_align_offset_tolerance_px = int(cfg.get('ramp_align_offset_tolerance_px', 10))
            self.ramp_up_pixel_coordinate = cfg.get('ramp_up_pixel_coordinate', self.ramp_up_pixel_coordinate)

    def start_posture_align_callback(self, request, response):
        self._reload_tolerance_config()
        self.start_posture_align = True  
        self._posture_align_stop_count = 0
        self.posture_pid.clear()
        self.posture_pid.SetPoint = 0.0
        response.success = True
        return response

    def scan_callback(self, msg):
        if not self.start_posture_align:
            return
        angles, ranges = _extract_front_scan(
            msg,
            half_angle_deg=self._front_angle_deg,
            front_center_deg=self._front_center_deg,
        )
        if len(ranges) < 2:
            self.cmd_pub.publish(Twist())
            return
        _, _, direction = _fit_line(angles, ranges)
        dev_rad = _alignment_deviation_rad(direction)
        self._last_deviation_rad = dev_rad
        if dev_rad is None:
            self.cmd_pub.publish(Twist())
            return
        dev_deg = math.degrees(dev_rad)

        self.posture_pid.update(-dev_rad)
        ang = float(np.clip(self.posture_pid.output, -0.4, 0.4))
        twist = Twist()
        twist.angular.z = ang
        self.cmd_pub.publish(twist)

        if abs(dev_deg) <= self.posture_align_tolerance_deg:
            self._posture_align_stop_count += 1
            if self._posture_align_stop_count > 15:
                self.start_posture_align = False
                self._posture_align_stop_count = 0
                self.cmd_pub.publish(Twist())
                self.publish_status('posture_aligned')

    def _color_image_callback(self, msg):
        if self.calibration_ramp_center:
            self._color_frame_calibration = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_ramp_center_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        self.ramp_up_pixel_coordinate = [int(x), int(y)]
        config = common.get_yaml_data(self.config_path)
        if config is None:
            config = {}
        config['ramp_up_pixel_coordinate'] = self.ramp_up_pixel_coordinate
        common.save_yaml_data(config, self.config_path)
        self.get_logger().info(
            f'[斜坡中心标定] 已保存 ramp_up_pixel_coordinate={self.ramp_up_pixel_coordinate} -> {self.config_path}'
        )
        self.calibration_ramp_center = False
        try:
            cv2.destroyWindow(self._calibration_window_name)
        except Exception:
            pass

    def _calibration_display_tick(self):
        if not self.calibration_ramp_center:
            return
        if self._color_frame_calibration is None:
            return
        try:
            cv2.imshow(self._calibration_window_name, self._color_frame_calibration)
            cv2.waitKey(1)
        except Exception:
            pass

    def calibration_ramp_center_callback(self, request, response):
        self.get_logger().info('start ramp center calibration: click in window to set center')
        if self.color_sub is None:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.color_sub = self.create_subscription(
                Image,
                '/gemini_camera/rgb/image_raw',
                self._color_image_callback,
                qos,
            )
        self.calibration_ramp_center = True
        self._color_frame_calibration = None
        try:
            cv2.namedWindow(self._calibration_window_name)
            cv2.setMouseCallback(self._calibration_window_name, self._on_ramp_center_click)
        except Exception as e:
            self.get_logger().warn(f'创建标定窗口失败: {e}')
        response.success = True
        return response

    def stop_callback(self, request, response):
        self.start_posture_align = False
        self.start_ramp_align_only = False
        self.calibration_ramp_center = False
        self.destroy_image_sub()
        self.destroy_scan_sub()
        self.destroy_color_sub()
        if getattr(self, '_calibration_window_name', None):
            try:
                cv2.destroyWindow(self._calibration_window_name)
            except Exception:
                pass
        response.success = True
        return response

    def start_ramp_align_callback(self, request, response):
        self._reload_tolerance_config()
        self.get_logger().info(
            f'start ramp align (depth only), offset tolerance=±{self.ramp_align_offset_tolerance_px:d}'
        )
        self.start_ramp_align_only = True
        self._ramp_align_stop_count = 0
        self.angular_pid.clear()
        self.angular_pid.SetPoint = float(self.ramp_up_pixel_coordinate[0])
        response.success = True
        return response

    def up_callback(self, request, response):
        if self._up_running:
            self.get_logger().warn('~/up 已在运行，忽略重复请求')
            response.success = False
            response.message = '~/up is already running'
            return response

        self._up_running = True

        def _up_sequence():
            try:
                if self.image_sub is None:
                    qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=1,
                    )
                    self.image_sub = self.create_subscription(
                        Image,
                        '/gemini_camera/depth/image_raw',
                        self.image_callback,
                        qos,
                    )
                self._ensure_scan_subscription()

                self.get_logger().info('开始姿态摆正')
                self._reload_tolerance_config()
                self.start_posture_align = True
                self._posture_align_stop_count = 0
                self.posture_pid.clear()
                self.posture_pid.SetPoint = 0.0

                while rclpy.ok() and self.start_posture_align:
                    time.sleep(0.05)

                self.get_logger().info('姿态摆正完成，开始坡面对齐')

                self._reload_tolerance_config()
                self.start_ramp_align_only = True
                self._ramp_align_stop_count = 0
                self.angular_pid.clear()
                self.angular_pid.SetPoint = float(self.ramp_up_pixel_coordinate[0])

                while rclpy.ok() and self.start_ramp_align_only:
                    time.sleep(0.05)
                self.get_logger().info('坡面对齐完成')
            finally:
                self._up_running = False

        threading.Thread(target=_up_sequence, daemon=True).start()
        response.success = True
        return response

    def _ramp_align_only_step(self):
        ax, ay = self.ramp_up_pixel_coordinate[0], self.ramp_up_pixel_coordinate[1]
        center_depth, offset_x = check_ramp_alignment(
            self.transition_depth_image.astype(np.float64),
            ax, ay,
        )
        twist = Twist()
        if center_depth is None or offset_x is None:
            self.cmd_pub.publish(twist)
            return
        self.angular_pid.SetPoint = float(ax)
        self.angular_pid.update(ax + offset_x)  
        ang = float(np.clip(self.angular_pid.output, -0.4, 0.4))  

        twist.angular.z = 0.0
        twist.linear.x = 0.0
        twist.linear.y = ang
        self.cmd_pub.publish(twist)

        if abs(offset_x) <= self.ramp_align_offset_tolerance_px:
            self._ramp_align_stop_count += 1  
            if self._ramp_align_stop_count > 20:  
                self.start_ramp_align_only = False
                self._ramp_align_stop_count = 0

                self.cmd_pub.publish(Twist())
                self.publish_status('ramp_aligned')

    def image_callback(self, ros_image):
        if self.close:
            return
        depth_image = np.ndarray(
            shape=(ros_image.height, ros_image.width),
            dtype=np.uint16,
            buffer=ros_image.data,
        )
        depth_image = depth_image.copy()

        if self.start_pick or self.start_ramp_align_only:
            with self._worker_lock:
                self._worker_image[:] = depth_image
            self._worker_event.set()
            if not self.start_pick:
                return
            now = time.time()
            if now - getattr(self, '_last_depth_log', 0) >= 2.0:
                self._last_depth_log = now
                self.get_logger().info('[坡面] 收到深度帧，已送入对齐')
            return

    # ------------------------------------------------------------------ #
    def ramp_align(self, depth_image):
        depth_gray = np.clip(depth_image, 300, 1500).astype(np.float64)
        depth_gray = ((depth_gray - 300) / 1200 * 255).astype(np.uint8)
        
        blurred = cv2.GaussianBlur(depth_gray, (5, 5), 0)
        _, depth_bit = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((5, 5), np.uint8)
        depth_bit = cv2.morphologyEx(depth_bit, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        twist = Twist()
        if contours:
            areaMaxContour, area_max = common.get_area_max_contour(contours, 1000)
            if areaMaxContour is not None and area_max > 1000:
                rect = cv2.minAreaRect(areaMaxContour)
                box = cv2.boxPoints(rect)
                center_x, center_y = rect[0]
                angle = rect[2]

                width, height = rect[1]
                if width < height:
                    angle = angle - 90
                
                self.yaw_pid.SetPoint = 0 
                self.yaw_pid.update(angle)
                yaw_output = self.yaw_pid.output

                self.angular_pid.SetPoint = self.pick_stop_x
                self.angular_pid.update(center_x)
                
                ang_vel = self.angular_pid.output + yaw_output
                self.angular_speed = np.clip(ang_vel, -0.5, 0.5)

                self.linear_pid.SetPoint = self.pick_stop_y
                self.linear_pid.update(center_y)
                lin_vel = self.linear_base_speed - self.linear_pid.output
                self.linear_speed = np.clip(lin_vel, -0.15, 0.15)
                
                if abs(self.angular_pid.last_error) < 15: self.angular_speed = 0.0
                if abs(self.linear_pid.last_error) < 10: self.linear_speed = 0.0

                now = time.time()
                if now - getattr(self, '_last_align_log', 0) >= 0.5:
                    self._last_align_log = now
                    self.get_logger().info(
                        f'[坡面对齐] center=({center_x:.0f},{center_y:.0f}) '
                        f'目标=({self.pick_stop_x},{self.pick_stop_y}) '
                        f'angle={angle:.1f}° area={area_max:.0f} | '
                        f'linear={self.linear_speed:.3f} angular={self.angular_speed:.3f}'
                    )
            else:
                self.linear_speed = 0.0
                self.angular_speed = 0.0
        
        twist.linear.x = float(self.linear_speed)
        twist.angular.z = float(self.angular_speed)
        self.cmd_pub.publish(twist)

        if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
            self.count_stop += 1
        else:
            self.count_stop = 0

        if self.count_stop > 20:
            self.publish_status('stop')
            self.start_pick = False
            self.count_stop = 0

    # ------------------------------------------------------------------ #
    def spin_once(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ramp shutting down')
    finally:
        node._worker_running = False
        node._worker_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
