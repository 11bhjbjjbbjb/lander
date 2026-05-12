#!/usr/bin/env python3
# encoding: utf-8

import os
import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_srvs.srv import Trigger
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import sdk.common as common
from xf_mic_asr_offline import voice_play
from servo_controller.bus_servo_control import set_servo_position


def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Pose().orientation
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class VoiceControlNavNode(Node):
    def __init__(self):
        super().__init__('voice_control_nav')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('slope_surface', True)
        self.declare_parameter('debug', False)
        self.map_frame = self.get_parameter('map_frame').value
        self.slope_surface = bool(self.get_parameter('slope_surface').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        self.costmap = '/local_costmap/costmap'

        share_dir = get_package_share_directory('competition')
        config_path = os.path.join(share_dir, 'config', 'config.yaml')
        cfg = common.get_yaml_data(config_path)
        self.pick_location_time = cfg.get('pick_location_time', [3.0])
        self.up_ramp_time = cfg.get('up_ramp_time', [3.5])

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.servo_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.target_shape_pub = self.create_publisher(String, '/shape_recognition/target_shape', 10)

        self.voice_sub = self.create_subscription(String, '/asr_node/voice_words', self.words_callback, qos)
        self.angle_sub = self.create_subscription(Int32, '/awake_node/angle', self.angle_callback, qos)  
        self.shape_status_sub = self.create_subscription(
            String, '/shape_recognition/status', self.shape_status_callback, qos)
        self.pick_status_sub = self.create_subscription(
            String, '/position_correction/status', self.pick_status_callback, qos)
        self.ramp_status_sub = self.create_subscription(
            String, '/ramp/status', self.ramp_status_callback, qos)
        self.yolo_shape_sub = self.create_subscription(
            String, '/yolov5/shape', self.yolo_shape_callback, qos)
        self.vllm_status_sub = self.create_subscription(
            String, '/vllm_scene_understand_service/status', self.vllm_status_callback, qos)

        self.angle = None
        self.running = True
        self.mission_started = False

        self.wait_nav2_active()  
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')  

        self.shape_status = 'start'
        self.pick_status = 'stop'
        self.ramp_status = 'stop'
        self.yolo_shape = 'None'
        self.detected_shape = 'None'
        self.vllm_status = 'stop'

        self.trigger_clients = {}
        self.service_names = [
            '/shape_recognition/start',
            '/shape_recognition/stop',
            '/shape_recognition/pick',
            '/shape_recognition/close',
            '/position_correction/start',
            '/position_correction/stop',
            '/position_correction/close',
            '/position_correction/pick_1',
            '/position_correction/pick_2',
            '/position_correction/pick_3',
            '/position_correction/place_3',
            '/yolov5/start',
            '/yolov5/stop',
            '/ramp/start',
            '/ramp/start_posture_align',
            '/ramp/start_ramp_align',
            '/vllm_scene_understand_service/recognize_card_1',
            '/vllm_scene_understand_service/recognize_card_2',
            '/vllm_scene_understand_service/recognize_card_3',
            '/vllm_scene_understand_service/trigger_report',
        ]  
        for name in self.service_names:
            self.trigger_clients[name] = self.create_client(Trigger, name)  

        self.init_servos()
        self.play('running')

    # ------------------------------------------------------------------ #
    # Subscriptions / Callbacks
    # ------------------------------------------------------------------ #
    def words_callback(self, msg):
        word = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            word = word.replace(' ', '')
        self.get_logger().info(f'语音识别: {word}')

        if word == '唤醒成功(wake-up-success)':
            self.play('awake')
            return
        if word == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
            return

        command = '开始安全任务' if not self.slope_surface else '开始执行任务'
        if word == command:
            if self.mission_started:  
                self.get_logger().info('已收到任务指令，忽略重复语音')
                return
            self.mission_started = True
            self.get_logger().info('收到开始任务指令，准备执行比赛流程')
            if self.voice_sub is not None:
                self.destroy_subscription(self.voice_sub)
                self.voice_sub = None
            threading.Thread(target=self.start_mission, daemon=True).start()

    def angle_callback(self, msg):
        self.angle = msg.data

    def shape_status_callback(self, msg):
        self.shape_status = msg.data

    def pick_status_callback(self, msg):
        self.pick_status = msg.data

    def ramp_status_callback(self, msg):
        self.ramp_status = msg.data

    def yolo_shape_callback(self, msg):
        self.yolo_shape = msg.data

    def vllm_status_callback(self, msg):
        self.vllm_status = msg.data

    # ------------------------------------------------------------------ #
    def init_servos(self):
        self.set_servos(2, ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)))
        time.sleep(2)

    def set_servos(self, duration, positions):
        set_servo_position(self.servo_pub, float(duration), tuple(positions))

    def play(self, name):
        voice_play.play(name, language=self.language)

    def call_trigger(self, name, timeout=None):
        client = self.trigger_clients.get(name)
        if client is None:
            client = self.create_client(Trigger, name)
            self.trigger_clients[name] = client
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info(f'等待服务 {name} ...')
        req = Trigger.Request()
        future = client.call_async(req)
        start = time.time()
        while rclpy.ok():
            if future.done():
                result = future.result()
                if result:
                    return result.success
                return False
            if timeout is not None and (time.time() - start) > timeout:
                self.get_logger().error(f'服务 {name} 超时')
                return False
            time.sleep(0.05)
        return False

    def wait_for_status(self, getter, target='stop', timeout=None, desc='status'):
        last_log = 0.0
        while rclpy.ok():
            current = getter()
            if current == target:
                return True
            now = time.time()
            if now - last_log > 2.0:
                self.get_logger().info(f'等待{desc}，当前状态: {current}')
                last_log = now
            time.sleep(0.2)
        return False

    def wait_nav2_active(self, timeout=None):
        """Ensure lifecycle_manager_navigation is ACTIVE before creating BasicNavigator."""
        client = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')  
        while rclpy.ok():
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('等待导航生命周期管理器服务...')
            else:
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done():
                    result = future.result()
                    if result and result.success:
                        self.get_logger().info('导航生命周期管理器已激活')
                        return True
                    msg = result.message if result else '未知状态'
                    self.get_logger().info(f'导航未激活：{msg}')
            time.sleep(1.0)
        return False

    def start_mission(self):
        self.get_logger().info('开始任务')
        self.play('1')
        twist = Twist()
        twist.linear.x = 0.3
        self.cmd_pub.publish(twist)
        time.sleep(3)
        self.cmd_pub.publish(Twist())
        twist.angular.z = -0.5
        self.cmd_pub.publish(twist)
        time.sleep(3)
        self.cmd_pub.publish(Twist())

        self.control(1.5, -0.3, 90, 'detect')
        self.control(0.95, -3.1, 0, 'pick1')
        self.control(1.28, -0.33, 35, 'place')
        self.control(0.95, -3.1, -180, 'pick2')
        self.control(1.28, -0.33, 35, 'place')
        self.control(1.5, -0.3, 0, 'recognize_card_1')
        self.control(0.95, -3.1, -45, 'recognize_card_2')
        self.control(0.95, -3.1, -180, 'recognize_card_3')

        self.call_trigger('/position_correction/close')
        self.call_trigger('/shape_recognition/close', timeout=None)

        if self.slope_surface:
            self.call_trigger('/ramp/start')
            self.control(1.0, 0.0, -160, 'back')
            self.call_trigger('/vllm_scene_understand_service/trigger_report')  
            self.wait_for_status(lambda: self.vllm_status, 'stop', desc='report')
            self.play('11')
        else:
            self.control(0.1, -0.3, 0, 'back')
            twist = Twist()
            twist.linear.y = 0.1
            self.cmd_pub.publish(twist)
            time.sleep(3)
            self.cmd_pub.publish(Twist())
            self.play('mission_accomplished')

        self.get_logger().info('任务结束')

    # ------------------------------------------------------------------ #
    def nav_position(self, x, y, yaw_deg):
        if self.debug:
            self.get_logger().info(f'[Debug] 模拟导航到 ({x:.2f}, {y:.2f}, {yaw_deg:.1f}deg)')
            time.sleep(3.0)
            self.get_logger().info('[Debug] 视为到达目标')
            return True

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('导航动作服务器不可用')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = rpy2qua(0.0, 0.0, math.radians(yaw_deg))

        self.get_logger().info(f'导航目标: ({x:.2f}, {y:.2f}, {yaw_deg:.1f}deg)')
        send_future = self.nav_client.send_goal_async(goal)  
        while rclpy.ok() and not send_future.done():  
            time.sleep(0.1)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:  
            self.get_logger().error('导航目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.2)

        result = result_future.result()
        if result is None:
            self.get_logger().error('导航结果未知')
            return False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('导航完成')
            return True
        self.get_logger().error(f'导航失败，状态码 {result.status}')
        return False


    def control(self, x, y, yaw, action):
        self.nav_position(x, y, yaw)
        time.sleep(1.0)

        if action in ('pick1', 'pick2'):
            self.execute_pick_sequence(action)
        elif action == 'place':
            self.execute_place_sequence()
        elif action == 'detect':
            self.execute_detect_sequence()
        elif action == 'back':
            self.execute_back_sequence()
        elif action in ('recognize_card_1', 'recognize_card_2', 'recognize_card_3'):
            self.execute_recognize_card_sequence(action)

    def execute_recognize_card_sequence(self, label):
        self.call_trigger(f'/vllm_scene_understand_service/{label}')
        self.wait_for_status(lambda: self.vllm_status, 'stop', desc=f'{label} recognition')

    def execute_pick_sequence(self, label):
        set_status = 'pick'
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_pub.publish(twist)
        time.sleep(2)
        self.cmd_pub.publish(Twist())
        self.call_trigger('/position_correction/start')
        self.call_trigger(f'/position_correction/{set_status}_1')
        time.sleep(1)
        self.get_logger().info('等待底盘对齐完成...')
        self.wait_for_status(lambda: self.pick_status, 'stop', desc='position alignment')
        self.call_trigger(f'/position_correction/{set_status}_2')
        time.sleep(1)
        self.get_logger().info('等待机械臂初步动作完成...')
        self.wait_for_status(lambda: self.pick_status, 'stop', desc='position action')
        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_pub.publish(twist)
        time.sleep(float(self.pick_location_time[0]))
        self.cmd_pub.publish(Twist())
        twist.angular.z = -0.5
        self.cmd_pub.publish(twist)
        time.sleep(0.1)
        self.cmd_pub.publish(Twist())
        self.call_trigger('/position_correction/stop')

        if self.detected_shape and self.detected_shape != 'None':
            self.get_logger().info(f'根据 YOLO 结果设置 target_shape={self.detected_shape}')
            self.target_shape_pub.publish(String(data=self.detected_shape))
            time.sleep(0.2)
        else:
            self.get_logger().warn(
                f'当前 detected_shape={self.detected_shape}，未能自动设置 target_shape，沿用旧值'
            )

        self.get_logger().info('启动形状识别节点...')
        self.call_trigger('/shape_recognition/start', timeout=None)
        self.call_trigger('/shape_recognition/pick', timeout=None)
        time.sleep(1)
        self.get_logger().info('等待形状识别完成...')
        self.wait_for_status(lambda: self.shape_status, 'stop', desc='pick_status')
        self.play('7')
        self.call_trigger('/shape_recognition/stop', timeout=None)
        twist = Twist()
        twist.linear.x = -0.2
        self.cmd_pub.publish(twist)
        time.sleep(3)
        self.cmd_pub.publish(Twist())

    def execute_place_sequence(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.cmd_pub.publish(twist)
        time.sleep(2.1)
        self.cmd_pub.publish(Twist())
        self.call_trigger('/position_correction/place_3')
        time.sleep(1)
        self.wait_for_status(lambda: self.pick_status, 'stop', desc='place_status')
        self.play('9')

    def execute_detect_sequence(self):
        if not self.slope_surface:
            self.play('reached_explosion-proof_warehouse')
        else:
            self.play('2')
        self.call_trigger('/yolov5/start')
        time.sleep(1)
        self.wait_for_shape()
        detected = self.yolo_shape
        self.detected_shape = detected
        if not self.slope_surface:
            if detected == 'box':
                self.play('ruled_out_cuboid_explosive')
            elif detected == 'cube':
                self.play('ruled_out_cube_explosive')
            else:
                self.play('ruled_out_cylinder_explosive')
        else:
            if detected == 'box':
                self.play('4')
            elif detected == 'cube':
                self.play('3')
            else:
                self.play('5')
        if detected:
            self.target_shape_pub.publish(String(data=detected))
        self.call_trigger('/yolov5/stop')
        time.sleep(2)

    def execute_back_sequence(self):
        if self.slope_surface:
            self.call_trigger('/ramp/up')
            time.sleep(1)
            self.wait_for_status(lambda: self.ramp_status, 'ramp_aligned', desc='ramp')
            twist = Twist()
            twist.linear.y = -0.1
            self.cmd_pub.publish(twist)
            time.sleep(1.0)
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
            twist = Twist()
            twist.linear.x = 0.3
            self.cmd_pub.publish(twist)
            time.sleep(float(self.up_ramp_time[0]))
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            time.sleep(6)
            self.cmd_pub.publish(Twist())

    def wait_for_shape(self):
        notified = False
        while rclpy.ok():
            if self.yolo_shape and self.yolo_shape != 'None':
                return True
            if not notified:
                self.get_logger().info('等待yolov5识别结果...')
                notified = True
            time.sleep(0.3)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('voice control shutting down')
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
