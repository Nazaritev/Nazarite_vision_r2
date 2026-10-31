#!/usr/bin/env python3
# 雷达fastlio发布的odometry修正之后的yaw数据
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time


class YawCheck(Node):

    def __init__(self):
        super().__init__('fastlio_yaw_check')

        self.start_time = time.monotonic()
        self.last_print = 0.0

        self.first_yaw = None
        self.last_yaw = None
        self.unwrapped_yaw = None

        self.create_subscription(
            Odometry,
            '/Odometry',
            self.callback,
            10
        )

        print("\n====================================")
        print("FAST-LIO 姿态漂移检测")
        print("Topic: /Odometry")
        print("机器人静止时重点观察 ΔYaw")
        print("====================================\n")

    def callback(self, msg):

        q = msg.pose.pose.orientation

        # quaternion -> roll pitch yaw
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)

        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)

        yaw_deg = math.degrees(yaw)

        # 第一次数据作为参考
        if self.first_yaw is None:
            self.first_yaw = yaw_deg
            self.last_yaw = yaw_deg
            self.unwrapped_yaw = yaw_deg

        # 处理 +180/-180 跳变
        diff = yaw_deg - self.last_yaw

        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        self.unwrapped_yaw += diff
        self.last_yaw = yaw_deg

        delta_yaw = self.unwrapped_yaw - self.first_yaw

        now = time.monotonic()

        # 每秒打印一次
        if now - self.last_print >= 1.0:

            runtime = now - self.start_time

            print(
                f"{runtime:7.1f}s | "
                f"Roll {math.degrees(roll):+8.3f}° | "
                f"Pitch {math.degrees(pitch):+8.3f}° | "
                f"Yaw {yaw_deg:+8.3f}° | "
                f"ΔYaw {delta_yaw:+8.3f}°"
            )

            self.last_print = now


def main():

    rclpy.init()

    node = YawCheck()

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