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
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String

from sdk.pid import PID
import sdk.misc as misc
import sdk.common as common
from xf_mic_asr_offline import voice_play
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition


class AutomaticPickNode(Node):
    size = (320, 240)

    def __init__(self):
        super().__init__('position_correction')

        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        self.machine_type = os.environ.get('MACHINE_TYPE', '')

        share_dir = get_package_share_directory('competition')
        self.config_path = os.path.join(share_dir, 'config', 'config.yaml')
        self.config = common.get_yaml_data(self.config_path)
        lab_cfg_path = os.path.join('/home/ubuntu/software/lab_tool/config', 'lab_config.yaml')
        self.lab_data = common.get_yaml_data(lab_cfg_path)

        self.declare_parameter('debug', False)
        self.declare_parameter('debug_display', False)
        self.declare_parameter('image_topic', '/gemini_camera/rgb/image_raw')

        self.broadcast = False
        self.calibration = False
        self.close = False

        self.start_pick = False
        self.start_place = False
        self.pick = False
        self.place = False
        self.stop_flag = True
        self.target_color = ''
        self.broadcast_status = ''
        self.status = 'stop'
        self.debug = bool(self.get_parameter('debug').value)

        self.linear_base_speed = 0.007
        self.angular_base_speed = 0.03

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.yaw_angle = 90.0

        self.pick_stop_x = self.config.get('pick_stop_pixel_coordinate', [320, 388])[0]
        self.pick_stop_y = self.config.get('pick_stop_pixel_coordinate', [320, 388])[1]
        self.place_stop_x = self.config.get('place_stop_pixel_coordinate', [320, 388])[0]
        self.place_stop_y = self.config.get('place_stop_pixel_coordinate', [320, 388])[1]

        self.d_x = 10
        self.d_y = 10
        self.count_stop = 0
        self.count_turn = 0
        self.count_debug = 0

        self.linear_pid = PID(P=0.0018, I=0, D=0)
        self.angular_pid = PID(P=0.003, I=0, D=0)
        self.yaw_pid = PID(P=0.015, I=0, D=0.000)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.status_pub = self.create_publisher(String, '/position_correction/status', 10)

        self.image_sub = None

        self.create_service(Trigger, '~/start', self.start_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(Trigger, '~/close', self.close_callback)
        self.create_service(SetBool, '~/calibration', self.calibration_callback)
        self.create_service(Trigger, '~/pick_1', self.start_pick_callback)
        self.create_service(Trigger, '~/pick_2', self.start_pick_2_callback)
        self.create_service(Trigger, '~/pick_3', self.start_pick_3_callback)
        self.create_service(Trigger, '~/place_1', self.start_place_callback)
        self.create_service(Trigger, '~/place_2', self.start_place_2_callback)
        self.create_service(Trigger, '~/place_3', self.start_place_3_callback)

        self.publish_status('stop')
        self.cmd_pub.publish(Twist())
        self.get_logger().info('position_correction node ready')
        self.debug_display = bool(self.get_parameter('debug_display').value)
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.debug_window = 'automatic_pick_debug'
        self.debug_window_open = False
        self._debug_display_size = (640, 480)
        if self.debug_display:
            qos = QoSProfile(depth=1)
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos,
            )
        self.last_pick_log = 0.0
        self.last_align_log = 0.0
        self.last_lost_log = 0.0
        self._async_tasks = {}
        self._async_lock = threading.Lock()
        self.align_hold_threshold = 30  # frames with zero velocity before finishing
        self.align_stable_frames = 0

    # ------------------------------------------------------------------ #
    # 工具方法
    # ------------------------------------------------------------------ #
    def publish_status(self, value):
        self.status = value
        self.status_pub.publish(String(data=value))

    def set_servos(self, duration, positions):
        set_servo_position(self.joints_pub, float(duration), tuple(positions))

    def play(self, name):
        voice_play.play(name, language=self.language)

    def destroy_node(self):
        self._close_debug_window()
        super().destroy_node()

    def destroy_image_sub(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        self._close_debug_window()

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

    def _show_debug_frame(self, frame):
        if not self.debug_display:
            return
        if not self.debug_window_open:
            cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.debug_window,
                self._debug_display_size[0],
                self._debug_display_size[1],
            )
            self.debug_window_open = True
        if frame.shape[:2] != (self._debug_display_size[1], self._debug_display_size[0]):
            frame = cv2.resize(frame, self._debug_display_size, interpolation=cv2.INTER_AREA)
        cv2.imshow(self.debug_window, frame)
        cv2.waitKey(1)

    def _close_debug_window(self):
        if self.debug_display and self.debug_window_open:
            try:
                cv2.destroyWindow(self.debug_window)
                cv2.waitKey(1)
            except cv2.error:
                pass
            self.debug_window_open = False

    def _log_alignment(self, center_x, center_y, angle):
        if not (self.start_pick or self.calibration):
            return
        now = time.time()
        if center_x > 0:
            if now - self.last_align_log < 0.5:
                return
            self.last_align_log = now
            msg = (
                f'[视觉对齐] center=({center_x},{center_y}) '
                f'target=({self.pick_stop_x},{self.pick_stop_y}) '
                f'linear={self.linear_speed:.3f} angular={self.angular_speed:.3f} '
                f'angle={angle:.1f} status={self.status}'
            )
            self.get_logger().info(msg)

    # ------------------------------------------------------------------ #
    # 服务回调
    # ------------------------------------------------------------------ #
    def start_callback(self, request, response):
        if self.image_sub is None:
            qos = QoSProfile(depth=1)
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos,
            )
        response.success = True
        return response

    def stop_callback(self, request, response):
        self.destroy_image_sub()
        response.success = True
        return response

    def close_callback(self, request, response):
        self.close = True
        self.destroy_image_sub()
        response.success = True
        return response

    def calibration_callback(self, request, response):
        self.calibration = True
        self.set_servos(2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
        time.sleep(2)
        if request.data:
            self.reset_motion()
            param = self.config.get('pick_stop_pixel_coordinate', [self.pick_stop_x, self.pick_stop_y])
            self.pick_stop_x, self.pick_stop_y = param[0], param[1]
            self.d_x = 5
            self.d_y = 5
            self.status = 'approach'
            self.target_color = 'box'
            self.linear_pid.clear()
            self.angular_pid.clear()
            self.align_stable_frames = 0
            self.count_debug = 0
            self.start_pick = True
            self.get_logger().info(
                '[校准] 正在保存数据，请将目标(box)置于视野内，debug=True 时稳定约 10 帧后自动保存'
            )
        response.success = True
        return response

    def start_pick_callback(self, request, response):
        self.get_logger().info('start pick_1')
        self.publish_status('pick')
        self.set_servos(2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
        time.sleep(2)
        self.reset_motion()
        param = self.config.get('pick_stop_pixel_coordinate', [self.pick_stop_x, self.pick_stop_y])
        self.pick_stop_x, self.pick_stop_y = param
        self.d_x = 5
        self.d_y = 5
        self.status = 'approach'
        self.get_logger().info('[视觉对齐] 进入 approach 状态，开始寻找目标')
        self.target_color = 'box'
        self.broadcast_status = 'find_target'
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.align_stable_frames = 0
        self.start_pick = True
        response.success = True
        return response

    def start_pick_2_callback(self, request, response):
        self.get_logger().info('start pick_2')
        ok = self._run_async_task('pick_2', 'pick_2', self._pick_2_sequence)
        response.success = ok
        if not ok:
            response.message = 'pick_2 busy'
        return response

    def start_pick_3_callback(self, request, response):
        self.get_logger().info('start pick_3')
        self.publish_status('pick')
        self.reset_motion()
        self.pick_stop_x = 320
        self.pick_stop_y = 180
        self.d_x = 5
        self.d_y = 5
        self.status = 'approach'
        self.get_logger().info('[视觉对齐] 进入 approach 状态(备用)，开始寻找目标')
        self.target_color = 'box'
        self.broadcast_status = 'find_target'
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.align_stable_frames = 0
        self.start_pick = True
        response.success = True
        return response

    def start_place_callback(self, request, response):
        self.get_logger().info('start place_1')
        self.publish_status('place')
        self.set_servos(2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
        time.sleep(2)
        self.reset_motion()
        self.place_stop_x = 360
        self.place_stop_y = 240
        self.d_x = 5
        self.d_y = 5
        self.status = 'approach'
        self.target_color = 'orange'
        self.broadcast_status = 'find_target'
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.align_stable_frames = 0
        self.start_pick = True
        response.success = True
        return response

    def start_place_2_callback(self, request, response):
        self.get_logger().info('start place_2')
        self.publish_status('place')
        self.set_servos(2, ((1, 500), (2, 720), (3, 100), (4, 100), (5, 500), (10, 600)))
        time.sleep(2)
        self.reset_motion()
        self.place_stop_x = 320
        self.place_stop_y = 200
        self.d_x = 5
        self.d_y = 5
        self.status = 'approach'
        self.target_color = 'orange'
        self.broadcast_status = 'find_target'
        self.linear_pid.clear()
        self.angular_pid.clear()
        self.align_stable_frames = 0
        self.start_pick = True
        response.success = True
        return response

    def start_place_3_callback(self, request, response):
        self.get_logger().info('start place_3')
        self.set_servos(2, ((1, 500), (2, 210), (3, 320), (4, 350), (5, 500), (10, 650)))
        time.sleep(2)
        self.set_servos(0.5, ((10, 200),))
        time.sleep(0.5)
        self.set_servos(2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
        time.sleep(2)
        response.success = True
        return response

    # ------------------------------------------------------------------ #
    def reset_motion(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.yaw_angle = 90.0
        self.stop_flag = True
        self.pick = False
        self.place = False
        self.count_stop = 0
        self.count_turn = 0
        self.align_stable_frames = 0

    def color_detect(self, img):
        img_h, img_w = img.shape[:2]
        debug_frame = img.copy()
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        lab_cfg = self.lab_data['lab']['gemini_camera']
        if self.target_color not in lab_cfg:
            return -1, -1, -1
        frame_mask = cv2.inRange(
            frame_lab,
            tuple(lab_cfg[self.target_color]['min']),
            tuple(lab_cfg[self.target_color]['max'])
        )
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        center_x, center_y, angle = -1, -1, -1
        if contours:
            area_max_contour, area_max = common.get_area_max_contour(contours, 10)
            if area_max_contour is not None and area_max > 10:
                rect = cv2.minAreaRect(area_max_contour)
                angle = rect[2]
                box = np.int0(cv2.boxPoints(rect))
                for j in range(4):
                    box[j, 0] = int(misc.val_map(box[j, 0], 0, self.size[0], 0, img_w))
                    box[j, 1] = int(misc.val_map(box[j, 1], 0, self.size[1], 0, img_h))
                (pt1x, pt1y) = box[0]
                (pt3x, pt3y) = box[2]
                center_x = int((pt1x + pt3x) / 2)
                center_y = int((pt1y + pt3y) / 2)
                if self.debug_display:
                    cv2.drawContours(debug_frame, [box], -1, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (center_x, center_y), 4, (0, 0, 255), -1)
        if self.debug_display:
            self._show_debug_frame(debug_frame)
        if self.calibration:
            return center_x, center_y, angle, img
        return center_x, center_y, angle

    def image_callback(self, ros_image):
        if self.close:
            return
        rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data,
        )
        bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.start_pick:
            self.stop_flag = True
            result = self.pick_handle(bgr)
            if self.calibration and result is not None:
                self._show_debug_frame(result)
        else:
            if self.stop_flag:
                self.stop_flag = False
            if self.debug_display:
                self._show_debug_frame(bgr)
            self._log_alignment(-1, -1, -1)
            return

    def pick_handle(self, image):
        img_center_x = image.shape[1] / 2
        img_center_y = image.shape[0] / 2
        twist = Twist()

        if not self.pick or self.debug:  
            result = self.color_detect(image)
            if self.calibration:
                center_x, center_y, angle, rgb = result
            else:
                center_x, center_y, angle = result
                rgb = None
            # 仅在「校准且开启保存」时写入 config；pick_1 只做对齐测试，不保存
            if self.debug and self.calibration and center_x > 0:
                self.count_debug += 1
                if self.count_debug > 10:
                    self.count_debug = 0
                    self.pick_stop_y = center_y
                    self.pick_stop_x = center_x
                    self.config['pick_stop_pixel_coordinate'] = [self.pick_stop_x, self.pick_stop_y]
                    common.save_yaml_data(self.config, self.config_path)
                    self.get_logger().info(
                        f'[校准] 保存完毕: pick_stop=({self.pick_stop_x}, {self.pick_stop_y}) -> {self.config_path}'
                    )
                    self.debug = False
                    if self.calibration:
                        self.calibration = False
                        self.start_pick = False
                        self.get_logger().info('[校准] 已退出停靠点保存模式')
            elif center_x > 0:
                if self.calibration:
                    # 仅校准/保存模式：只打日志，不发布速度，不改变 pick/start_pick
                    self._log_alignment(center_x, center_y, angle)
                else:
                    if self.broadcast and self.broadcast_status == 'find_target':
                        self.broadcast_status = 'crawl_succeeded'
                        self.play('find_target')

                    self.linear_pid.SetPoint = self.pick_stop_y
                    if abs(center_y - self.pick_stop_y) <= self.d_y:
                        center_y = self.pick_stop_y
                    if self.status != 'align':
                        self.linear_pid.update(center_y)
                        tmp = self.linear_base_speed + self.linear_pid.output
                        self.linear_speed = max(min(tmp, 0.15), -0.15)
                        if abs(tmp) <= 0.0075:
                            self.linear_speed = 0.0

                    self.angular_pid.SetPoint = self.pick_stop_x
                    if abs(center_x - self.pick_stop_x) <= self.d_x:
                        center_x = self.pick_stop_x
                    if self.status != 'align':
                        self.angular_pid.update(center_x)
                        tmp = self.angular_base_speed + self.angular_pid.output
                        self.angular_speed = max(min(tmp, 1.2), -1.2)
                        if abs(tmp) <= 0.038:
                            self.angular_speed = 0.0

                    is_stationary = abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0

                    if is_stationary and self.status not in ('align', 'adjust'):
                        self.count_stop = 0

                    if is_stationary and self.target_color != 'box':
                        if not self.pick:
                            self.get_logger().info('[视觉对齐] 目标已锁定，触发抓取')
                        self.pick = True
                        self.start_pick = False
                        self.publish_status('stop')
                    elif is_stationary and self.target_color == 'box':
                        self.count_turn += 1
                        if self.count_turn > 5:
                            self.count_turn = 5
                            if self.status not in ('align', 'adjust'):
                                self.get_logger().info('[视觉对齐] 进入 align 状态，开始姿态对齐')
                                self.count_stop = 0
                                self.align_stable_frames = 0
                            self.status = 'align'
                            if self.count_stop < 10:
                                if angle < 40:
                                    angle += 90
                                self.yaw_pid.SetPoint = 90
                                if abs(angle - 90) <= 1:
                                    angle = 90
                                self.yaw_pid.update(angle)
                                self.yaw_angle = self.yaw_pid.output
                                if angle != 90:
                                    if abs(self.yaw_angle) <= 0.038:
                                        self.count_stop += 1
                                    else:
                                        self.count_stop = 0
                                    twist.linear.y = -2 * 0.3 * math.sin(self.yaw_angle / 2)
                                    twist.angular.z = self.yaw_angle
                                else:
                                    self.count_stop += 1
                            elif self.count_stop <= self.align_hold_threshold:
                                self.d_x = 5
                                self.d_y = 5
                                self.count_stop += 1
                                if self.status != 'adjust':
                                    self.get_logger().info('[视觉对齐] 进入 adjust 状态，微调位置')
                                self.status = 'adjust'
                            else:
                                self.align_stable_frames += 1
                                if self.align_stable_frames < self.align_hold_threshold:
                                    return rgb
                                self.count_stop = 0
                                self.align_stable_frames = 0
                                if not self.pick:
                                    self.get_logger().info('[视觉对齐] 对齐完成')
                                self.pick = True
                                self.start_pick = False
                                self.publish_status('stop')
                    else:
                        self.count_stop = 0
                        self.count_turn = 0
                        if self.status != 'align':
                            twist.linear.x = self.linear_speed
                            twist.angular.z = self.angular_speed

                    self.cmd_pub.publish(twist)
                    self._log_alignment(center_x, center_y, angle)
            else:
                self.align_stable_frames = 0
                self._log_alignment(-1, -1, -1)
            return rgb
        else:
            self.cmd_pub.publish(Twist())
            self._log_alignment(-1, -1, -1)
            return None

    # ------------------------------------------------------------------ #
    def _pick_2_sequence(self):
        self.publish_status('pick')
        self.get_logger().info('[执行] pick_2 阶段：移动机械臂抓取目标')
        self.set_servos(1, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
        time.sleep(2)
        self.publish_status('stop')
        self.get_logger().info('[执行] pick_2 完成，等待上层流程进行识别/夹取')


def main(args=None):
    rclpy.init(args=args)
    node = AutomaticPickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('position_correction shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
