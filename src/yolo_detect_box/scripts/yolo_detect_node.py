#!/usr/bin/env python3
"""服务触发的 YOLO 检测：首次发现目标后，连续丢失超过 1 秒结束。"""

from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from ultralytics import YOLO

from yolo_detect_box.srv import CameraUse


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_box')
        model_path = Path(get_package_share_directory('yolo_detect_box')) / 'model' / 'best.pt'
        self.declare_parameter('conf', 0.6)
        self.conf = self.get_parameter('conf').value
        self.model = YOLO(str(model_path), task='detect')
        self.declare_parameter('camera_index', 0)
        camera_index = self.get_parameter('camera_index').value
        # 直接打开指定摄像头；待机期间不读取画面。
        self.cap = cv2.VideoCapture(camera_index)
        self.active = False
        self.seen_target = False
        self.missing_since = None
        self.finished = None

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        # 服务等待期间，采集定时器仍需执行，因此使用独立的回调组。
        self.service_group = ReentrantCallbackGroup()
        self.service = self.create_service(
            CameraUse, '/camara_use', self.use_callback,
            callback_group=self.service_group)
        self.get_logger().info('待机中，等待 /camara_use 请求 use=1')

    async def use_callback(self, request, response):
        # 每次只进行一轮检测；其他请求直接返回 false。
        if request.use != 1 or self.active:
            response.success = False
            return response

        self.seen_target = False
        self.missing_since = None
        self.finished = Future(executor=self.executor)
        self.active = True
        self.get_logger().info('开始检测，等待首次出现目标')
        # 异步等待本轮结束，不阻塞采集定时器；此时还不返回服务响应。
        response.success = await self.finished
        return response

    def process_frame(self):
        # 待机时不读取摄像头、不推理、不显示。
        if not self.active:
            return

        ret, frame = self.cap.read()
        if not ret:
            return
        result = self.model.predict(
            # 过滤低置信度结果，每帧最多保留置信度最高的一个框。
            frame, device='cpu', conf=self.conf, max_det=1, verbose=False)[0]
        cv2.imshow('YOLO Detection', result.plot())
        cv2.waitKey(1)

        if len(result.boxes) > 0:
            self.seen_target = True
            # 目标再次出现，取消之前的丢失计时。
            self.missing_since = None
        elif self.seen_target:
            now = time.monotonic()
            if self.missing_since is None:
                self.missing_since = now
            elif now - self.missing_since > 1.0:
                self.active = False
                cv2.destroyAllWindows()
                self.get_logger().info('连续超过 1 秒无目标，回到待机状态')
                self.finished.set_result(True)


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
