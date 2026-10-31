#!/usr/bin/env python3
# 订阅yaw roll pitch的数据
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class FastLioAttitudeCheck(Node):

    def __init__(self):
        super().__init__('fastlio_attitude_check')

        self.odom_topic = '/Odometry'
        self.imu_topic = '/livox/imu'

        # 使用 BEST_EFFORT 确保能收到 Livox 和 FAST-LIO 的数据
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, sensor_qos
        )

        # 数据记录
        self.start_time = None
        self.initial_pose = None
        self.odom_received = 0
        self.last_print_time = 0.0
        self.print_interval = 0.5  # 每 0.5 秒打印一次

        print("============================================================")
        print("FAST-LIO 全姿态 / 位移 漂移检测")
        print("============================================================")
        print("重点观察：")
        print("  1. Roll 单方向累计 -> 导致雷达左右倾斜，点云远端左右翘边")
        print("  2. Yaw  单方向累计 -> 导致雷达原地缓慢旋转")
        print("  3. Z 单方向累计    -> 导致点云整体上下起伏")
        print("------------------------------------------------------------")

        # 表头
        print(
            f"{'Time':>6} | "
            f"{'Roll(Δ)':>15} | "
            f"{'Pitch(Δ)':>15} | "
            f"{'Yaw(Δ)':>15} | "
            f"{'X':>8} | "
            f"{'Y':>8} | "
            f"{'Z':>8}"
        )
        print("-" * 92)

    def odom_callback(self, msg):
        self.odom_received += 1
        now = time.monotonic()

        if self.start_time is None:
            self.start_time = now

        if now - self.last_print_time < self.print_interval:
            return
        self.last_print_time = now

        # 获取四元数
        q = msg.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w

        # 四元数转欧拉角 (Roll, Pitch, Yaw)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 转换为角度
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # 获取位置
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z

        # 初始化参考值
        if self.initial_pose is None:
            self.initial_pose = {
                'roll': roll_deg,
                'pitch': pitch_deg,
                'yaw': yaw_deg,
                'x': pos_x,
                'y': pos_y,
                'z': pos_z
            }
            print("\n========== 初始参考值 ==========")
            print(f"Roll: {roll_deg:+.4f}° | Pitch: {pitch_deg:+.4f}° | Yaw: {yaw_deg:+.4f}°")
            print(f"X: {pos_x:+.4f} | Y: {pos_y:+.4f} | Z: {pos_z:+.4f}")
            print("================================\n")

        # 计算变化量
        d_roll = roll_deg - self.initial_pose['roll']
        d_pitch = pitch_deg - self.initial_pose['pitch']
        d_yaw = yaw_deg - self.initial_pose['yaw']

        elapsed = now - self.start_time

        print(
            f"{elapsed:5.1f}s | "
            f"{roll_deg:+7.3f}({d_roll:+.3f}) | "
            f"{pitch_deg:+7.3f}({d_pitch:+.3f}) | "
            f"{yaw_deg:+7.3f}({d_yaw:+.3f}) | "
            f"{pos_x:+8.4f} | "
            f"{pos_y:+8.4f} | "
            f"{pos_z:+8.4f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FastLioAttitudeCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()