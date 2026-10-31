#查看雷达和imu时间是否同步
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

from collections import deque
import statistics
import time


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TimeSyncCheck(Node):

    def __init__(self):
        super().__init__('time_sync_check')

        # 更适合传感器数据
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.imu_times = deque(maxlen=5000)
        self.lidar_times = deque(maxlen=1000)

        self.results = deque(maxlen=1000)

        self.imu_count = 0
        self.lidar_count = 0
        self.start_time = time.monotonic()

        self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            qos
        )

        self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.lidar_callback,
            qos
        )

        # 每秒进行一次匹配
        self.create_timer(1.0, self.process)

        # 每5秒输出统计
        self.create_timer(5.0, self.print_stats)

        self.get_logger().info(
            'Time sync checker started: /livox/lidar + /livox/imu'
        )

    def imu_callback(self, msg):
        self.imu_count += 1
        self.imu_times.append(
            stamp_to_sec(msg.header.stamp)
        )

    def lidar_callback(self, msg):
        self.lidar_count += 1
        self.lidar_times.append(
            stamp_to_sec(msg.header.stamp)
        )

    def process(self):

        if not self.imu_times:
            return

        if not self.lidar_times:
            return

        imu_list = list(self.imu_times)

        while self.lidar_times:

            lidar_t = self.lidar_times.popleft()

            nearest_imu = min(
                imu_list,
                key=lambda t: abs(t - lidar_t)
            )

            diff_ms = (
                lidar_t - nearest_imu
            ) * 1000.0

            self.results.append(diff_ms)

    def print_stats(self):

        runtime = time.monotonic() - self.start_time

        print("\n========================================")
        print(f"运行时间       : {runtime:.1f} s")
        print(f"LiDAR接收帧数  : {self.lidar_count}")
        print(f"IMU接收帧数    : {self.imu_count}")

        # 这一段很重要，可以判断到底有没有订阅成功
        if self.lidar_count == 0:
            print("⚠ 没有收到 LiDAR")
        if self.imu_count == 0:
            print("⚠ 没有收到 IMU")

        if not self.results:
            print("⚠ 暂时没有可用的时间匹配结果")
            print("========================================")
            return

        data = list(self.results)

        abs_data = [
            abs(x) for x in data
        ]

        mean_signed = statistics.mean(data)
        mean_abs = statistics.mean(abs_data)
        max_abs = max(abs_data)

        print("----------------------------------------")
        print(f"匹配数量       : {len(data)}")
        print(f"平均时间差     : {mean_signed:+.3f} ms")
        print(f"平均绝对差     : {mean_abs:.3f} ms")
        print(f"最大绝对差     : {max_abs:.3f} ms")

        # 看最近100个样本，更适合判断后期变化
        recent = data[-100:]
        recent_abs = [abs(x) for x in recent]

        print("----------------------------------------")
        print(f"最近平均绝对差 : "
              f"{statistics.mean(recent_abs):.3f} ms")

        print(f"最近最大绝对差 : "
              f"{max(recent_abs):.3f} ms")

        if statistics.mean(recent_abs) < 3:
            print("状态           : 时间匹配正常")
        elif statistics.mean(recent_abs) < 10:
            print("状态           : 时间差偏大，继续观察")
        else:
            print("状态           : ⚠ 时间同步异常")

        print("========================================")


def main(args=None):

    rclpy.init(args=args)

    node = TimeSyncCheck()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n检测结束")

    finally:
        node.print_stats()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()