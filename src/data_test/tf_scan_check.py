#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data

class TFScanChecker(Node):

    def __init__(self):
        super().__init__('tf_scan_checker')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 自动获取 /scan 的 frame_id
        self.scan_frame = None

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 每 1 秒打印一次
        self.timer = self.create_timer(1.0, self.check_all)

        self.get_logger().info(
            'TF/Scan 检测启动，等待 /scan 和 TF...'
        )

    def scan_callback(self, msg):
        if self.scan_frame != msg.header.frame_id:
            self.scan_frame = msg.header.frame_id

            self.get_logger().info(
                f'/scan frame_id = {self.scan_frame}'
            )

    def print_tf(self, target, source, name):

        try:
            tf = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time()
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            roll, pitch, yaw = euler_from_quaternion([
                q.x,
                q.y,
                q.z,
                q.w
            ])

            print(
                f'{name:<28} | '
                f'XYZ=({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}) | '
                f'RPY=({math.degrees(roll):+7.2f}, '
                f'{math.degrees(pitch):+7.2f}, '
                f'{math.degrees(yaw):+7.2f}) deg'
            )

        except Exception as e:
            print(
                f'{name:<28} | TF unavailable: {str(e)[:80]}'
            )

    def check_all(self):

        print('\n' + '=' * 100)

        if self.scan_frame:
            print(f'/scan frame_id: {self.scan_frame}')
        else:
            print('/scan frame_id: 等待 /scan ...')

        print('-' * 100)

        # 1. base_link -> footprint_link
        self.print_tf(
            'base_link',
            'footprint_link',
            'base_link <- footprint_link'
        )

        # 2. base_link -> scan frame
        if self.scan_frame:
            self.print_tf(
                'base_link',
                self.scan_frame,
                f'base_link <- {self.scan_frame}'
            )

        # 3. map -> odom
        self.print_tf(
            'map',
            'odom',
            'map <- odom'
        )

        # 4. odom -> base_link
        self.print_tf(
            'odom',
            'base_link',
            'odom <- base_link'
        )

        print('=' * 100)


def main(args=None):
    rclpy.init(args=args)

    node = TFScanChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()