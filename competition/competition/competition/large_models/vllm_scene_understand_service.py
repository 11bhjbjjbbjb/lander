#!/usr/bin/env python3

import os
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from std_msgs.msg import String

from speech import speech

from .config import (
    api_key,
    base_url,
    default_tts_model,
    default_vllm_model,
)

'''
# 三种卡片识别服务（识别过程中不播报）
ros2 service call /vllm_scene_understand_service/recognize_card_1 std_srvs/srv/Trigger
ros2 service call /vllm_scene_understand_service/recognize_card_2 std_srvs/srv/Trigger
ros2 service call /vllm_scene_understand_service/recognize_card_3 std_srvs/srv/Trigger

# 触发播报三种卡片识别结果
ros2 service call /vllm_scene_understand_service/trigger_report std_srvs/srv/Trigger

ros2 service call /vllm_scene_understand_service/trigger_scene_understand std_srvs/srv/Trigger
ros2 topic pub --once /vllm_scene_understand_service/scene_understand_prompt std_msgs/msg/String "data: '你是一个月球探索爱好者，帮我看看手机里的照片描述的是什么，回答有关于月球探索相关的专有名称，如宇航员、月球陨石、太空卫星等等'"
'''

class VLLMSceneUnderstandService(Node):

    def __init__(self) -> None:
        super().__init__('vllm_scene_understand_service')
        self.declare_parameter('image_topic', '/astra_camera/rgb/image_raw')
        self.declare_parameter('window_name', 'VLLM Scene View')
        self.declare_parameter('default_prompt', '你是一个月球探索爱好者，帮我看看这张卡片中描述的是什么场景元素。请只回答场景元素的名称，例如：宇航员、月球坑、月球陨石、太空卫星、月球车、空间站、火箭、地球、月球、月球土壤、探月器等。如果卡片中没有明显的月球探索相关元素，请回答"未识别到相关元素"。')
        self.declare_parameter('enable_preview', True)  
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._window = self.get_parameter('window_name').get_parameter_value().string_value
        self._default_prompt = self.get_parameter('default_prompt').get_parameter_value().string_value
        self._enable_preview = self.get_parameter('enable_preview').get_parameter_value().bool_value

        self.vllm_scene_understand_service_status_pub = self.create_publisher(String, '/vllm_scene_understand_service/status', 10)
        
        if self._enable_preview:
            cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window, 640, 480)

        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._last_frame_ts = 0.0
        self._no_frame_logged = False

        self.create_subscription(Image, image_topic, self._image_callback, 10)
        
        if self._enable_preview:
            self.create_timer(0.03, self._show_frame)

        self.client = speech.OpenAIAPI(api_key, base_url)
        # 使用None作为log参数，减少TTS内部日志输出
        self.tts = speech.RealTimeTTS(log=None)

        self._card_results = {
            1: None,  # 第一个任务点
            2: None,  # 第二个任务点
            3: None,  # 第三个任务点
        }
        self._card_lock = threading.Lock()

        # 创建服务：识别第一个卡片
        self.create_service(
            Trigger,
            '~/recognize_card_1',
            lambda req, res: self._recognize_card_callback(req, res, 1)
        )
        
        # 创建服务：识别第二个卡片
        self.create_service(
            Trigger,
            '~/recognize_card_2',
            lambda req, res: self._recognize_card_callback(req, res, 2)
        )
        
        # 创建服务：识别第三个卡片
        self.create_service(
            Trigger,
            '~/recognize_card_3',
            lambda req, res: self._recognize_card_callback(req, res, 3)
        )
        
        # 创建服务：触发播报三种卡片的结果
        self.create_service(
            Trigger,
            '~/trigger_report',
            self._trigger_report_callback
        )

        # 保留原有接口以兼容
        self.create_service(
            Trigger,
            '~/trigger_scene_understand',
            self._trigger_scene_understand_callback
        )
        
        self.create_subscription(
            String,
            '~/scene_understand_prompt',
            self._prompt_callback,
            10
        )

        self._is_processing = False
        self._initial_status_published = False

        # 简化启动日志，只保留关键信息
        self.get_logger().info('VLLM场景理解服务节点已启动')
        
        # 使用定时器延迟发布初始状态，确保发布器已准备好
        def publish_initial_status():
            if not self._initial_status_published:
                self.publish_status('stop')
                self._initial_status_published = True
                # 停止定时器
                if hasattr(self, '_init_timer'):
                    self._init_timer.cancel()
        self._init_timer = self.create_timer(1.0, publish_initial_status)

    def destroy_node(self) -> bool:
        if self._enable_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()

    def publish_status(self, value):
        self.status = value
        self.vllm_scene_understand_service_status_pub.publish(String(data=value))

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return
        with self._frame_lock:
            self._latest_frame = frame
            self._last_frame_ts = time.time()
            self._no_frame_logged = False

    def _show_frame(self) -> None:
        with self._frame_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            last_ts = self._last_frame_ts
        if frame is None:
            if time.time() - last_ts > 1.0 and not self._no_frame_logged:
                self.get_logger().warning('未获取到摄像头画面，请确认摄像头节点是否已启动')
                self._no_frame_logged = True
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            cv2.imshow(self._window, frame)
            cv2.waitKey(1)
        except cv2.error as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'OpenCV显示失败: {exc}')

    def _recognize_card_callback(self, request, response, card_number: int):
        """识别指定编号的卡片"""
        if self._is_processing:
            response.success = False
            response.message = f'场景理解正在进行中，请稍候'
            print(f'\n[卡片识别] ⚠️  第{card_number}个任务点：场景理解正在进行中，忽略新的请求', flush=True)
            self.get_logger().warn(f'第{card_number}个任务点：场景理解正在进行中，忽略新的请求')
            return response
        
        self._is_processing = True
        response.success = True
        response.message = f'开始识别第{card_number}个任务点卡片'
        
        print(f'\n[卡片识别] 🚀 第{card_number}个任务点识别中...', flush=True)
        self.publish_status('start')
        
        # 在后台线程中执行场景理解，避免阻塞服务响应
        # 第三个参数False表示不进行语音播报
        threading.Thread(
            target=self._execute_scene_understand, 
            args=(self._default_prompt, False, card_number), 
            daemon=True
        ).start()
        
        return response

    def _trigger_report_callback(self, request, response):
        """触发播报三种卡片识别结果"""
        with self._card_lock:
            results = {
                1: self._card_results[1],
                2: self._card_results[2],
                3: self._card_results[3],
            }
        
        # 检查是否所有卡片都已识别
        missing_cards = [i for i in [1, 2, 3] if results[i] is None]
        if missing_cards:
            response.success = False
            response.message = f'还有任务点未识别：{missing_cards}'
            print(f'\n[播报] ⚠️  无法播报：还有任务点未识别 {missing_cards}', flush=True)
            self.get_logger().warn(f'无法播报：还有任务点未识别 {missing_cards}')
            return response
        
        response.success = True
        response.message = '开始播报识别结果'
        
        print('\n[播报] 🚀 开始播报识别结果', flush=True)
        self.publish_status('start')
        
        # 在后台线程中执行播报
        threading.Thread(target=self._execute_report, args=(results,), daemon=True).start()
        
        return response

    def _trigger_scene_understand_callback(self, request, response):
        """使用默认提示词触发场景理解（保留原有接口）"""
        if self._is_processing:
            response.success = False
            response.message = '场景理解正在进行中，请稍候'
            print('\n[场景理解] ⚠️  场景理解正在进行中，忽略新的请求', flush=True)
            self.get_logger().warn('场景理解正在进行中，忽略新的请求')
            return response
        
        self._is_processing = True
        response.success = True
        response.message = '开始场景理解'
        
        print('\n[场景理解] 🚀 开始场景理解...', flush=True)
        
        # 在后台线程中执行场景理解，避免阻塞服务响应
        # 使用True表示需要语音播报（保持原有行为）
        threading.Thread(target=self._execute_scene_understand, args=(self._default_prompt, True, None), daemon=True).start()
        
        return response

    def _prompt_callback(self, msg: String) -> None:
        """通过话题接收自定义提示词并触发场景理解"""
        prompt = msg.data.strip()
        if not prompt:
            self.get_logger().warn('收到空的提示词，使用默认提示词')
            prompt = self._default_prompt
        else:
            self.get_logger().debug(f'收到自定义提示词: {prompt}')
        
        if self._is_processing:
            print('[场景理解] ⚠️  场景理解正在进行中，忽略新的请求', flush=True)
            self.get_logger().warn('场景理解正在进行中，忽略新的请求')
            return
        
        print('\n[场景理解] 🚀 开始场景理解...', flush=True)
        
        # 在后台线程中执行场景理解
        # 使用True表示需要语音播报（保持原有行为）
        threading.Thread(target=self._execute_scene_understand, args=(prompt, True, None), daemon=True).start()

    def _execute_scene_understand(self, prompt: str, enable_tts: bool, card_number: Optional[int] = None) -> None:
        """执行场景理解的核心逻辑
        
        Args:
            prompt: 提示词
            enable_tts: 是否启用语音播报
            card_number: 卡片编号（1-3），如果为None则表示不是卡片识别任务
        """
        try:
            # 获取最新帧
            frame = self._get_latest_frame()
            if frame is None:
                error_msg = '暂时没有摄像头画面，请稍后再试'
                if card_number:
                    print(f'[卡片识别] ❌ 第{card_number}个任务点：{error_msg}', flush=True)
                else:
                    print(f'[场景理解] ❌ {error_msg}', flush=True)
                self.get_logger().error(error_msg)
                if enable_tts:
                    self.tts.tts(error_msg, model=default_tts_model)
                self._is_processing = False
                self.publish_status('stop')
                return
            
            # 调用VLLM进行场景理解
            try:
                if card_number:
                    self.get_logger().debug(f'正在识别第{card_number}个任务点...')
                else:
                    self.get_logger().debug('正在调用VLLM进行图像理解...')
                answer = self.client.vllm(prompt, frame, prompt='', model=default_vllm_model)
            except Exception as exc:  # pylint: disable=broad-except
                error_msg = '图像理解失败，请稍后再试'
                if card_number:
                    print(f'[卡片识别] ❌ 第{card_number}个任务点：{error_msg}', flush=True)
                else:
                    print(f'[场景理解] ❌ {error_msg}', flush=True)
                self.get_logger().error(f'VLLM调用失败: {exc}')
                if enable_tts:
                    self.tts.tts(error_msg, model=default_tts_model)
                self._is_processing = False
                self.publish_status('stop')
                return
            
            # 记录结果
            if card_number:
                # 保存识别结果
                with self._card_lock:
                    self._card_results[card_number] = answer
                
                print(f'[卡片识别] ✅ 第{card_number}个任务点: {answer}', flush=True)
                self.get_logger().info(f'第{card_number}个任务点识别结果: {answer}')
            else:
                print(f'[场景理解] ✅ 结果: {answer}', flush=True)
                self.get_logger().info(f'场景理解结果: {answer}')
                
                if enable_tts:
                    print('[场景理解] 🔊 正在播报...', flush=True)
                    self.tts.tts(answer, model=default_tts_model)
            
        except Exception as exc:  # pylint: disable=broad-except
            if card_number:
                print(f'\n[卡片识别] ❌ 第{card_number}个任务点识别过程出错: {exc}\n', flush=True)
                self.get_logger().error(f'第{card_number}个任务点识别过程出错: {exc}')
            else:
                print(f'\n[场景理解] ❌ 场景理解过程出错: {exc}\n', flush=True)
                self.get_logger().error(f'场景理解过程出错: {exc}')
        finally:
            self._is_processing = False
            self.publish_status('stop')

    def _execute_report(self, results: dict) -> None:
        """执行播报三种卡片识别结果
        
        Args:
            results: 包含三个任务点识别结果的字典 {1: result1, 2: result2, 3: result3}
        """
        try:
            # 提取场景元素名称（简化处理，直接使用识别结果）
            element_1 = results[1] if results[1] else '未识别'
            element_2 = results[2] if results[2] else '未识别'
            element_3 = results[3] if results[3] else '未识别'
            
            # 生成三句播报文本，分别播报
            report_texts = [
                f'第一个任务点识别到{element_1}',
                f'第二个任务点识别到{element_2}',
                f'第三个任务点识别到{element_3}'
            ]
            
            # 显示播报内容摘要
            print(f'[播报] 📊 {element_1} | {element_2} | {element_3}', flush=True)
            
            # 依次播报三句话（静默播报，不输出详细日志）
            for i, text in enumerate(report_texts, 1):
                self.get_logger().debug(f'播报第{i}句: {text}')
                self.tts.tts(text, model=default_tts_model)
                # 等待播报完成（避免三句话重叠）
                if i < len(report_texts):
                    time.sleep(0.5)
            
            print('[播报] ✅ 播报完成\n', flush=True)
            
        except Exception as exc:  # pylint: disable=broad-except
            print(f'\n[播报] ❌ 播报过程出错: {exc}\n', flush=True)
            self.get_logger().error(f'播报过程出错: {exc}')
        finally:
            self.publish_status('stop')

    def _get_latest_frame(self) -> Optional[np.ndarray]:
        """获取最新的摄像头帧"""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLLMSceneUnderstandService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
