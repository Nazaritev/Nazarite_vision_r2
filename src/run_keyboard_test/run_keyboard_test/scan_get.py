#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanCounter(Node):
    def __init__(self):
        super().__init__('scan_counter')
        self.count = 0

        # LaserScan 发布者通常用 BEST_EFFORT（传感器数据默认 QoS），
        # 这里保持兼容；若你的发布者是 RELIABLE，也可改成 RELIABLE。
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos,
        )
        self.get_logger().info('scan_counter started, waiting for /scan ...')

    def scan_callback(self, msg: LaserScan):
        self.count += 1
        point_num = len(msg.ranges)
        self.get_logger().info(f'frame {self.count}, points: {point_num}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanCounter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
