#!/usr/bin/env python3

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


class FastLioPitchCheck(Node):

    def __init__(self):
        super().__init__('fastlio_pitch_check')

        # ============================================================
        # Topic 配置
        # ============================================================

        self.odom_topic = '/Odometry'
        self.imu_topic = '/livox/imu'

        # ============================================================
        # QoS
        #
        # FAST-LIO / Livox 这类高频传感器数据通常使用 BEST_EFFORT。
        # 使用普通 depth=10 有时会因为 QoS 不兼容而收不到数据。
        # ============================================================

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )

        # ============================================================
        # ROS2 Subscription
        # ============================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        # ============================================================
        # 数据
        # ============================================================

        self.start_time = None

        self.initial_pitch = None
        self.initial_z = None

        self.current_gyro_y = float('nan')

        self.odom_received = 0
        self.imu_received = 0

        # 每 0.5 秒输出一次
        self.last_print_time = 0.0
        self.print_interval = 0.5

        # ============================================================
        # 启动信息
        # ============================================================

        print()
        print("============================================================")
        print("FAST-LIO Pitch / Z 漂移检测")
        print("============================================================")
        print(f"Odometry : {self.odom_topic}")
        print(f"IMU      : {self.imu_topic}")
        print("QoS      : RELIABLE / VOLATILE / KEEP_LAST")
        print("============================================================")
        print()
        print("测试方法：")
        print("  1. 启动 FAST-LIO")
        print("  2. 机器人在水平地面静止约 10 秒")
        print("  3. 保持方向基本不变，直线行驶 5~10 m")
        print("  4. 停车")
        print("  5. 继续静止约 10 秒")
        print()
        print("重点观察：")
        print("  ΔPitch : FAST-LIO Pitch 是否单方向累计")
        print("  ΔZ     : 高度是否随着 Pitch 一起漂")
        print("  gyro_y : IMU Y 轴角速度是否存在持续偏置")
        print("============================================================")
        print()

        print(
            f"{'Time':>8} | "
            f"{'Pitch':>10} | "
            f"{'ΔPitch':>10} | "
            f"{'Z':>9} | "
            f"{'ΔZ':>9} | "
            f"{'gyro_y':>11}"
        )

        print("-" * 82)


        # 10 秒后检查 Topic 是否成功订阅到数据
        self.topic_check_timer = self.create_timer(
            10.0,
            self.check_topic_subscription
        )

        self.topic_check_done = False

    def check_topic_subscription(self):

        if self.topic_check_done:
            return

        self.topic_check_done = True

        print()
        print("============================================================")
        print("10 秒 Topic 订阅检查")
        print("============================================================")

        if self.odom_received == 0:
            print("❌ /Odometry：10 秒内没有收到任何数据")
            print("   检查：ros2 topic info /Odometry -v")
        else:
            print(
                f"✓ /Odometry：订阅正常 "
                f"({self.odom_received} 条)"
            )

        if self.imu_received == 0:
            print("❌ /livox/imu：10 秒内没有收到任何数据")
            print("   检查：ros2 topic info /livox/imu -v")
        else:
            print(
                f"✓ /livox/imu：订阅正常 "
                f"({self.imu_received} 条)"
            )

        print("============================================================")
        print()

        # 只检查一次
        self.topic_check_timer.cancel()
    # ================================================================
    # IMU callback
    # ================================================================

    def imu_callback(self, msg):

        self.imu_received += 1

        # IMU 绕 Y 轴角速度
        # 通常对应 Pitch 方向角速度
        self.current_gyro_y = msg.angular_velocity.y

    # ================================================================
    # Odometry callback
    # ================================================================

    def odom_callback(self, msg):

        self.odom_received += 1

        now = time.monotonic()

        if self.start_time is None:
            self.start_time = now

        # ------------------------------------------------------------
        # 限制打印频率
        # ------------------------------------------------------------

        if now - self.last_print_time < self.print_interval:
            return

        self.last_print_time = now

        # ------------------------------------------------------------
        # Quaternion
        # ------------------------------------------------------------

        q = msg.pose.pose.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # ------------------------------------------------------------
        # Quaternion -> Roll
        # ------------------------------------------------------------

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)

        roll = math.atan2(
            sinr_cosp,
            cosr_cosp
        )

        # ------------------------------------------------------------
        # Quaternion -> Pitch
        # ------------------------------------------------------------

        sinp = 2.0 * (w * y - z * x)

        # 防止浮点误差导致 asin 输入超过 [-1, 1]
        sinp = max(-1.0, min(1.0, sinp))

        pitch = math.asin(sinp)

        # ------------------------------------------------------------
        # Quaternion -> Yaw
        # ------------------------------------------------------------

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        # ------------------------------------------------------------
        # 转换为角度
        # ------------------------------------------------------------

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # ------------------------------------------------------------
        # FAST-LIO Z
        # ------------------------------------------------------------

        current_z = msg.pose.pose.position.z

        # ------------------------------------------------------------
        # 第一次收到 Odometry 时记录基准
        # ------------------------------------------------------------

        if self.initial_pitch is None:

            self.initial_pitch = pitch_deg
            self.initial_z = current_z

            print()
            print("========== 初始参考值 ==========")
            print(f"Pitch : {self.initial_pitch:+.6f} deg")
            print(f"Z     : {self.initial_z:+.6f} m")
            print("================================")
            print()

        # ------------------------------------------------------------
        # 相对于初始状态的变化
        # ------------------------------------------------------------

        delta_pitch = pitch_deg - self.initial_pitch
        delta_z = current_z - self.initial_z

        elapsed = now - self.start_time

        # ------------------------------------------------------------
        # 输出
        # ------------------------------------------------------------

        print(
            f"{elapsed:7.1f}s | "
            f"{pitch_deg:+9.4f}° | "
            f"{delta_pitch:+9.4f}° | "
            f"{current_z:+8.4f} | "
            f"{delta_z:+8.4f} | "
            f"{self.current_gyro_y:+10.6f}"
        )


# ====================================================================
# main
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = FastLioPitchCheck()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print()
        print()
        print("============================================================")
        print("检测结束")
        print("============================================================")
        print(f"收到 Odometry : {node.odom_received} 条")
        print(f"收到 IMU      : {node.imu_received} 条")

        if node.odom_received == 0:

            print()
            print("警告：没有收到 /Odometry")
            print("请检查：")
            print("  ros2 topic info /Odometry")
            print("  ros2 topic echo /Odometry --once")

        if node.imu_received == 0:

            print()
            print("警告：没有收到 /livox/imu")
            print("请检查：")
            print("  ros2 topic info /livox/imu")
            print("  ros2 topic echo /livox/imu --once")

        print("============================================================")

    finally:

        node.destroy_node()

        # 某些情况下 Ctrl+C 后 context 已经 shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()