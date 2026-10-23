#!/usr/bin/env python3
"""服务触发的 YOLO 检测：首次发现目标后，连续丢失超过 1 秒结束。"""

from pathlib import Path
import math
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from ultralytics import YOLO

from yolo_detect_box.srv import CameraUse
from yolo_detect_box.msg import BoxDetection


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_box')
        # 与控制节点使用同一组 launch 参数，窗口中的边界随控制范围变化。
        for name, default in (
                ('center_left_ratio', 0.4), ('center_right_ratio', 0.6),
                ('enable_forward_control', True),
                ('center_top_ratio', 0.5), ('center_bottom_ratio', 0.7)):
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
        if not 0.0 <= self.center_left_ratio < self.center_right_ratio <= 1.0:
            raise ValueError('中心范围必须满足 0 <= left_ratio < right_ratio <= 1')
        if not 0.0 <= self.center_top_ratio < self.center_bottom_ratio <= 1.0:
            raise ValueError('纵向中心范围必须满足 0 <= top_ratio < bottom_ratio <= 1')
        model_path = Path(get_package_share_directory('yolo_detect_box')) / 'model' / 'best.pt'
        self.declare_parameter('conf', 0.6)
        self.conf = self.get_parameter('conf').value
        self.model = YOLO(str(model_path), task='detect')
        self.declare_parameter('camera_index', 0)
        camera_index = self.get_parameter('camera_index').value
        # 直接打开指定摄像头；待机期间不读取画面。
        self.cap = cv2.VideoCapture(camera_index)
        self.declare_parameter('missing_timeout', 1.0)
        self.declare_parameter('camera_timeout', 2.0)
        self.declare_parameter('show_window', True)
        self.declare_parameter('detection_topic', '/yolo/box_detection')
        self.declare_parameter('box_height_min_ratio', 0.8)
        self.declare_parameter('box_height_max_ratio', 0.9)
        self.missing_timeout = float(self.get_parameter('missing_timeout').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        self.center_left_ratio = float(
            self.get_parameter('center_left_ratio').value)
        self.center_right_ratio = float(
            self.get_parameter('center_right_ratio').value)
        self.box_height_min_ratio = float(
            self.get_parameter('box_height_min_ratio').value)
        self.box_height_max_ratio = float(
            self.get_parameter('box_height_max_ratio').value)
        for name, value in (
                ('missing_timeout', self.missing_timeout),
                ('camera_timeout', self.camera_timeout)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} 必须为有限正数')
        ratios = (
            self.center_left_ratio,
            self.center_right_ratio,
            self.box_height_min_ratio,
            self.box_height_max_ratio,
        )
        if not all(math.isfinite(ratio) for ratio in ratios):
            raise ValueError('画面标注比例必须为有限数值')
        if not 0.0 <= self.center_left_ratio < self.center_right_ratio <= 1.0:
            raise ValueError('中心范围必须满足 0 <= left < right <= 1')
        if not (
                0.0 < self.box_height_min_ratio
                < self.box_height_max_ratio <= 1.0):
            raise ValueError('框高比例必须满足 0 < min < max <= 1')
        self.show_window = self.get_parameter('show_window').value
        self.detection_pub = self.create_publisher(
            BoxDetection, self.get_parameter('detection_topic').value, 1)
        self.active = False
        self.seen_target = False
        self.missing_since = None
        self.finished = None
        self.session_id = 0
        self.last_fx = -1.0
        self.last_fy = -1.0
        self.read_failed_since = None

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        # 服务等待期间，采集定时器仍需执行，因此使用独立的回调组。
        self.service_group = ReentrantCallbackGroup()
        self.service = self.create_service(
            CameraUse, '/camara_use', self.use_callback,
            callback_group=self.service_group)
        self.get_logger().info('待机中，等待 /camara_use 请求 use=1')

    def draw_alignment_guides(self, image):
        image_height, image_width = image.shape[:2]
        for ratio in (self.center_left_ratio, self.center_right_ratio):
            x = min(round(ratio * image_width), image_width - 1)
            cv2.line(image, (x, 0), (x, image_height - 1), (0, 255, 255), 1)
        for ratio in (self.box_height_min_ratio, self.box_height_max_ratio):
            y = min(round((1.0 - ratio) * image_height), image_height - 1)
            cv2.line(image, (0, y), (image_width - 1, y), (0, 255, 255), 1)

    async def use_callback(self, request, response):
        response.fx = -1.0
        response.fy = -1.0
        # 每次只进行一轮检测；其他请求直接返回状态 0。
        if request.use != 1 or self.active:
            response.success = 0
            return response

        self.seen_target = False
        self.missing_since = None
        self.read_failed_since = None
        self.last_fx = -1.0
        self.last_fy = -1.0
        self.session_id = request.session_id
        # 保存本次 Future，防止下一轮启动后覆盖本次响应所需的数据。
        finished = Future(executor=self.executor)
        self.finished = finished
        self.active = True
        self.get_logger().info('开始检测，等待首次出现目标')
        # 异步等待本轮结束，不阻塞采集定时器；此时还不返回服务响应。
        response.success, response.fx, response.fy = await finished
        return response

    def finish_detection(self, status):
        """结束本轮检测，状态 1 表示完成，状态 0 表示失败。"""
        self.active = False
        if self.show_window:
            cv2.destroyAllWindows()
        if self.finished is not None and not self.finished.done():
            self.finished.set_result((status, self.last_fx, self.last_fy))

    def draw_debug_overlay(self, frame, detection):
        """画出框中心的左右、上下判定边界；所有判断使用原图浮点坐标。"""
        height, width = frame.shape[:2]
        green, yellow, red = (0, 255, 0), (0, 220, 255), (0, 80, 255)
        white = (255, 255, 255)
        scale = max(0.35, min(width / 1080.0, height / 720.0))
        font_scale = 0.65 * scale
        line_height = max(16, round(26 * scale))
        margin = max(6, round(12 * scale))

        def label(text, point, color=white):
            # 黑色描边保证亮色目标和复杂背景上也能看清。
            cv2.putText(frame, text, point, cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, point, cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, color, 1, cv2.LINE_AA)

        left, right = self.center_left_ratio * width, self.center_right_ratio * width
        top, bottom = self.center_top_ratio * height, self.center_bottom_ratio * height
        valid_center = (detection.detected and 0.0 <= detection.fx < width
                        and 0.0 <= detection.fy < height)
        in_center = valid_center and left <= detection.fx <= right
        in_vertical = (valid_center
                       and self.center_top_ratio <= detection.fy / height <= self.center_bottom_ratio)
        in_position = in_center and (not self.enable_forward_control or in_vertical)
        boundary_color = green if in_center else yellow
        for ratio, x in ((self.center_left_ratio, left), (self.center_right_ratio, right)):
            pixel_x = min(width - 1, round(x))
            cv2.line(frame, (pixel_x, 0), (pixel_x, height - 1), boundary_color, 2)
            label(f'{ratio:.0%}', (min(pixel_x + 4, width - 50), height - margin),
                  boundary_color)

        # 白色十字为画面中心，虚线为当前配置的水平目标中点。
        target_x = min(width - 1, round((left + right) / 2.0))
        for y in range(0, height, max(12, round(24 * scale))):
            cv2.line(frame, (target_x, y),
                     (target_x, min(height - 1, y + max(6, round(12 * scale)))),
                     (180, 180, 180), 1)
        cv2.drawMarker(frame, (width // 2, height // 2), white,
                       cv2.MARKER_CROSS, max(12, round(24 * scale)), 2)
        if not valid_center:
            state, state_color = 'NO TARGET', yellow
        elif in_position:
            state, state_color = 'IN POSITION', green
        elif not in_center:
            state, state_color = 'ALIGN X', red
        else:
            state = ('ALIGN Y (ABOVE)' if detection.fy / height < self.center_top_ratio
                     else 'ALIGN Y (BELOW)')
            state_color = red
        label(state, (margin, line_height), state_color)
        center_text = f'{detection.fx:.1f}' if valid_center else '--'
        label(f'X: {center_text}  [{left:.1f}, {right:.1f}] px',
              (margin, 2 * line_height), green if in_center else yellow)
        vertical_text = f'{detection.fy:.1f}' if valid_center else '--'
        label(f'Y: {vertical_text}  [{top:.1f}, {bottom:.1f}] px'
              if self.enable_forward_control else 'Y control: OFF (X only)',
              (margin, 3 * line_height), green if in_vertical else yellow)
        label('Cross: image center | Dot: box center', (margin, 4 * line_height))

        if self.enable_forward_control:
            # 与 cx 的竖线一样，cy 的两条横线固定在画面比例位置，不跟随检测框。
            vertical_color = green if in_vertical else yellow
            for name, ratio in (('TOP', self.center_top_ratio),
                                ('BOTTOM', self.center_bottom_ratio)):
                y = min(height - 1, round(ratio * height))
                cv2.line(frame, (0, y), (width - 1, y), vertical_color, 2)
                label(f'Y {name} {ratio:.1%}',
                      (margin, max(line_height, y - margin)), vertical_color)
        # 最后画中心点，避免判定线覆盖点的红/绿到位状态。
        if valid_center:
            point = (min(width - 1, round(detection.fx)),
                     min(height - 1, round(detection.fy)))
            color = green if in_position else red
            cv2.circle(frame, point, max(4, round(6 * scale)), color, -1, cv2.LINE_AA)
            cv2.circle(frame, point, max(6, round(9 * scale)), white, 1, cv2.LINE_AA)
        return frame

    def process_frame(self):
        # 待机时不读取摄像头、不推理、不显示。
        if not self.active:
            return

        ret, frame = self.cap.read()
        if not ret:
            # 读图失败不是“方块被拿走”，不能据此返回成功。
            self.missing_since = None
            now = time.monotonic()
            if self.read_failed_since is None:
                self.read_failed_since = now
            if now - self.read_failed_since >= self.camera_timeout:
                self.get_logger().error('摄像头连续读图失败，本轮检测失败')
                self.finish_detection(0)
            return
        self.read_failed_since = None
        message = BoxDetection()
        message.stamp = self.get_clock().now().to_msg()
        message.session_id = self.session_id
        message.image_height, message.image_width = frame.shape[:2]
        message.fx = -1.0
        message.fy = -1.0
        message.x1 = message.y1 = message.x2 = message.y2 = -1.0
        try:
            result = self.model.predict(
                # 过滤低置信度结果，每帧最多保留一个框。
                frame, device='cpu', conf=self.conf, max_det=1, verbose=False)[0]
            message.detected = len(result.boxes) > 0
            if message.detected:
                # xyxy 是原图像素坐标，不使用归一化坐标或模型输入尺寸。
                x1, y1, x2, y2 = result.boxes.xyxy[0].tolist()
                message.x1, message.y1 = float(x1), float(y1)
                message.x2, message.y2 = float(x2), float(y2)
                message.fx = float((x1 + x2) / 2.0)
                message.fy = float((y1 + y2) / 2.0)
            if self.show_window:
                debug_frame = self.draw_debug_overlay(result.plot(), message)
                cv2.imshow('YOLO Detection', debug_frame)
                cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f'YOLO 推理或显示失败: {exc}')
            self.finish_detection(0)
            return
        self.detection_pub.publish(message)

        if message.detected:
            self.last_fx = message.fx
            self.last_fy = message.fy
            self.seen_target = True
            # 目标再次出现，取消之前的丢失计时。
            self.missing_since = None
        elif self.seen_target:
            now = time.monotonic()
            if self.missing_since is None:
                self.missing_since = now
            elif now - self.missing_since >= self.missing_timeout:
                self.get_logger().info(
                    f'首次识别后连续 {self.missing_timeout:.2f} 秒无目标，回到待机状态')
                self.finish_detection(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
