#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main(args=None):
    rclpy.init(args=args)
    node = Node('wait_for_localization')
    target_frame = node.declare_parameter('target_frame', 'map').value
    source_frame = node.declare_parameter('source_frame', 'footprint_link').value
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    exit_code = 1
    node.get_logger().info(
        f'等待定位 TF {target_frame} <- {source_frame}；'
        '请在 RViz 使用 2D Pose Estimate 设置初始位姿。'
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if buffer.can_transform(target_frame, source_frame, Time()):
                node.get_logger().info('定位 TF 已就绪，启动导航、栅格转换和堆叠检测。')
                exit_code = 0
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
