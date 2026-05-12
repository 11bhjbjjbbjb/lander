import argparse
import queue
import threading

import cv2
from ultralytics import YOLO


class RealTimeDetector:
    def __init__(
        self,
        model_path="best.pt",
        conf_threshold=0.5,
        source=0,
        max_window_width=960,
        max_window_height=720,
    ):
        """
        使用YOLO进行实时目标检测（无需ROS）

        Args:
            model_path: 训练模型的路径
            conf_threshold: 置信度阈值
            source: 视频源 (摄像头索引或视频文件路径)
        """
        print("[DETECT_INIT] 初始化实时检测器...")
        print(f"[DETECT_INIT] 模型路径: {model_path}")
        print(f"[DETECT_INIT] 置信度阈值: {conf_threshold}")
        print(f"[DETECT_INIT] 视频源: {source}")

        self.model = YOLO(model_path)
        print("[DETECT_MODEL] YOLO模型加载成功")

        self.conf_threshold = conf_threshold
        self.source = source
        self.frame_count = 0
        self.window_name = "YOLOv8 Real-Time Detection"
        self.max_window_width = max_window_width
        self.max_window_height = max_window_height

        self.class_names = {
            0: "box",
            1: "cube",
            2: "cylinder",
        }
        self.colors = {
            0: (255, 0, 0),
            1: (0, 255, 0),
            2: (0, 0, 255),
        }

        print(f"[DETECT_CLASS] 类别映射: {self.class_names}")
        print(f"[DETECT_COLOR] 颜色映射: {self.colors}")

        self.frame_queue = queue.Queue(maxsize=2)
        self.latest_frame_lock = threading.Lock()
        self.latest_processed_frame = None
        self.stop_event = threading.Event()
        self.window_initialized = False
        self._queue_warning_shown = False

        self.worker_thread = threading.Thread(
            target=self._inference_worker, name="yolo_inference_worker", daemon=True
        )
        self.worker_thread.start()

    def process_frame(self, frame):
        """
        对单帧图像执行YOLO推理并绘制可视化内容
        """
        if frame is None:
            print("[DETECT_PROC] 帧为空，跳过")
            return None

        self.frame_count += 1
        print(
            f"[DETECT_PROC] 开始处理帧{self.frame_count}: "
            f"{frame.shape[1]}x{frame.shape[0]}"
        )

        display_frame = frame.copy()
        results = self.model(display_frame, conf=self.conf_threshold, verbose=False)
        print(f"[DETECT_INFERENCE] 推理完成，结果数量: {len(results)}")

        display_frame = self.draw_detections(display_frame, results)
        display_frame = self.add_info_panel(display_frame)

        cv2.putText(
            display_frame,
            f"Frame: {self.frame_count}",
            (10, display_frame.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        return display_frame

    def draw_detections(self, frame, results):
        """
        在图像上绘制检测框、标签等信息
        """
        if results[0].boxes is None:
            print("[DETECT_RESULT] 未检测到任何对象")
            return frame

        boxes = results[0].boxes.xyxy.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
        print(f"[DETECT_RESULT] 检测到 {len(boxes)} 个对象")

        for i, (box, conf, class_id) in enumerate(zip(boxes, confidences, class_ids)):
            print(
                f"[DETECT_RESULT] 对象{i+1}: 类别ID={class_id}, "
                f"置信度={conf:.3f}, 坐标={box}"
            )

            if conf < self.conf_threshold:
                print(
                    f"[DETECT_RESULT] 置信度过低 ({conf:.3f} < "
                    f"{self.conf_threshold})，跳过"
                )
                continue

            x1, y1, x2, y2 = map(int, box)
            class_name = self.class_names.get(class_id, f"Class_{class_id}")
            color = self.colors.get(class_id, (255, 255, 255))

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label = f"{class_name}: {conf:.2f}"
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )

            cv2.rectangle(
                frame,
                (x1, y1 - text_height - 10),
                (x1 + text_width, y1),
                color,
                -1,
            )

            cv2.putText(
                frame,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

        return frame

    def add_info_panel(self, frame):
        """
        在左上角叠加文字信息
        """
        info_text = [
            "YOLOv8 Object Detection",
            "Source: Camera/Video Stream",
            "Classes: 0-box, 1-cube, 2-cylinder",
            "Press 'q' to quit, 's' to save screenshot",
        ]

        y_offset = 30
        for text in info_text:
            cv2.putText(
                frame,
                text,
                (10, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            y_offset += 25

        return frame

    def submit_frame(self, frame):
        """
        将最新帧投递到推理线程；当队列已满时丢弃旧数据以保持实时性
        """
        if frame is None:
            return

        try:
            self.frame_queue.put_nowait(frame)
        except queue.Full:
            if not self._queue_warning_shown:
                print("[DETECT_RUN] 推理线程繁忙，自动丢弃部分帧以保持窗口流畅")
                self._queue_warning_shown = True
            try:
                _ = self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                pass

    def get_latest_processed_frame(self):
        with self.latest_frame_lock:
            return self.latest_processed_frame

    def resize_for_display(self, frame):
        """
        根据屏幕限制缩放显示图像，避免OpenCV窗口过大
        """
        if frame is None:
            return None

        height, width = frame.shape[:2]
        max_w = max(1, int(self.max_window_width))
        max_h = max(1, int(self.max_window_height))
        scale = min(max_w / float(width), max_h / float(height), 1.0)

        if scale >= 0.999:
            return frame

        new_size = (max(1, int(width * scale)), max(1, int(height * scale)))
        return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)

    def _inference_worker(self):
        """
        后台线程：持续从队列取帧并运行YOLO推理，避免阻塞主GUI线程
        """
        while not self.stop_event.is_set():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            processed_frame = self.process_frame(frame)
            if processed_frame is not None:
                with self.latest_frame_lock:
                    self.latest_processed_frame = processed_frame

    def run(self):
        """
        打开视频源并持续执行检测
        """
        print("[DETECT_RUN] 开始读取视频源...")
        cap = cv2.VideoCapture(self.source)

        if not cap.isOpened():
            raise RuntimeError(f"无法打开视频源: {self.source}")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[DETECT_RUN] 读取视频帧失败，结束检测")
                    break

                self.submit_frame(frame)
                processed_frame = self.get_latest_processed_frame()
                display_frame = processed_frame if processed_frame is not None else frame
                display_frame = self.resize_for_display(display_frame)

                if not self.window_initialized:
                    cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                    self.window_initialized = True

                cv2.imshow(self.window_name, display_frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    print("[DETECT_RUN] 用户请求退出")
                    break
                if key == ord("s"):
                    screenshot_name = f"detection_screenshot_{self.frame_count}.jpg"
                    frame_to_save = (
                        processed_frame if processed_frame is not None else frame
                    )
                    cv2.imwrite(screenshot_name, frame_to_save)
                    print(f"[DETECT_RUN] 截图已保存: {screenshot_name}")

        except KeyboardInterrupt:
            print("[DETECT_RUN] 检测被用户中断")

        finally:
            self.stop_event.set()
            if self.worker_thread.is_alive():
                self.worker_thread.join(timeout=1.0)
            cap.release()
            cv2.destroyAllWindows()
            print("[DETECT_RUN] 资源已释放，程序结束")


def parse_args():
    parser = argparse.ArgumentParser(
        description="实时YOLOv8检测（移除ROS依赖，纯Python实现）"
    )
    parser.add_argument(
        "--model-path",
        default="best.pt",
        help="YOLO模型路径 (默认: best.pt)",
    )
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=0.5,
        help="置信度阈值 (默认: 0.5)",
    )
    parser.add_argument(
        "--source",
        default="0",
        help="视频源：摄像头索引或视频文件路径 (默认: 0)",
    )
    parser.add_argument(
        "--max-window-width",
        type=int,
        default=960,
        help="OpenCV窗口最大宽度 (默认: 960)",
    )
    parser.add_argument(
        "--max-window-height",
        type=int,
        default=720,
        help="OpenCV窗口最大高度 (默认: 720)",
    )
    return parser.parse_args()


def resolve_source(value):
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return value


def main():
    args = parse_args()
    detector = RealTimeDetector(
        model_path=args.model_path,
        conf_threshold=args.conf_threshold,
        source=resolve_source(args.source),
        max_window_width=args.max_window_width,
        max_window_height=args.max_window_height,
    )
    detector.run()


if __name__ == "__main__":
    main()
