#!/usr/bin/env python3
import math
import sys
import threading
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from stm32_driver.r2_mode1 import (
    SERIAL_COMMANDS,
    ChassisMode,
    NavTarget,
    Point3d,
    RobotState,
)
from stm32_driver.r1_mode1_challenge_blue import R1Mode1ChallengeBlue


class R1Mode1ChallengeBlueTest(R1Mode1ChallengeBlue):
    CHALLENGE_NAVIGATION_OFFSET = Point3d(-0.80, -0.570, 0.0, 0.0)
    CHALLENGE_BARCODE_POINTS = (
        CHALLENGE_NAVIGATION_OFFSET,
    )
    CHALLENGE_RELATIVE_OFFSETS = {}
    CHALLENGE_SCAN_CMD8_TO_CMD9_DELAY = 0.7
    CONTACT_APPROACH_MODE_VALUE = 4
    MODE2_AFTER_CMD9_REPLY_COUNT = 3
    FORCE_MODE9_TIMEOUT_SEC = 120.0

    def __init__(self, node_name: str = "r1_mode1_challenge_blue_test"):
        self._direct_contact_active = False
        self._direct_contact_timer = None
        self._direct_contact_start_time = None
        self._direct_contact_start_pose = None
        self._direct_contact_round_number = None
        self._active_contact_approach_speed_x = None
        self._active_contact_approach_timeout_sec = None
        self._active_contact_approach_max_distance = None
        self._active_contact_approach_command_period_sec = None
        self._cmd9_reply_count = 0
        self._force_mode9_finished = False
        self._force_mode9_waiting_reply = False
        self._force_mode9_timer = None
        super().__init__(node_name=node_name)

        self.declare_parameter("contact_approach_cmd_vel_topic", "/contact_cmd_vel")
        self.declare_parameter("contact_approach_speed_x", -0.40)
        self.declare_parameter("contact_approach_timeout_sec", 2.5)
        self.declare_parameter("contact_approach_max_distance", 0.32)
        self.declare_parameter("contact_approach_command_period_sec", 0.05)
        self.declare_parameter("contact_approach_third_speed_x", -0.40)
        self.declare_parameter("contact_approach_third_timeout_sec", 1.8)
        self.declare_parameter("contact_approach_third_max_distance", 0.32)
        self.declare_parameter("contact_approach_third_command_period_sec", 0.05)
        self.declare_parameter(
            "force_mode9_timeout_sec",
            self.FORCE_MODE9_TIMEOUT_SEC,
        )

        self.contact_approach_cmd_vel_topic = str(
            self.get_parameter("contact_approach_cmd_vel_topic").value
        ).strip() or "/contact_cmd_vel"
        self.contact_approach_speed_x = float(
            self.get_parameter("contact_approach_speed_x").value
        )
        if self.contact_approach_speed_x > 0.0:
            self.get_logger().warn(
                f"contact_approach_speed_x={self.contact_approach_speed_x:.3f} 为正，"
                "固定顶靠只允许后退，自动改为负值"
            )
            self.contact_approach_speed_x = -self.contact_approach_speed_x
        self.contact_approach_third_speed_x = float(
            self.get_parameter("contact_approach_third_speed_x").value
        )
        if self.contact_approach_third_speed_x > 0.0:
            self.get_logger().warn(
                f"contact_approach_third_speed_x={self.contact_approach_third_speed_x:.3f} 为正，"
                "第3轮固定顶靠只允许后退，自动改为负值"
            )
            self.contact_approach_third_speed_x = -self.contact_approach_third_speed_x
        self.contact_approach_timeout_sec = max(
            0.1, float(self.get_parameter("contact_approach_timeout_sec").value)
        )
        self.contact_approach_max_distance = max(
            0.0, float(self.get_parameter("contact_approach_max_distance").value)
        )
        self.contact_approach_command_period_sec = max(
            0.02, float(self.get_parameter("contact_approach_command_period_sec").value)
        )
        self.contact_approach_third_timeout_sec = max(
            0.1, float(self.get_parameter("contact_approach_third_timeout_sec").value)
        )
        self.contact_approach_third_max_distance = max(
            0.0, float(self.get_parameter("contact_approach_third_max_distance").value)
        )
        self.contact_approach_third_command_period_sec = max(
            0.02,
            float(self.get_parameter("contact_approach_third_command_period_sec").value),
        )
        self.force_mode9_timeout_sec = max(
            0.1, float(self.get_parameter("force_mode9_timeout_sec").value)
        )
        self.direct_contact_cmd_pub = self.create_publisher(
            Twist,
            self.contact_approach_cmd_vel_topic,
            10,
        )
        self.get_logger().info(
            "蓝方挑战测试模式已启用：仅保留第 1 个相对当前位置导航点；"
            "第 1/2 次命令9回包后直接进入固定后退顶靠；"
            f"第 {self.MODE2_AFTER_CMD9_REPLY_COUNT} 次命令9回包后发布 /mode=2 "
            f"timeout_force_mode11={self.force_mode9_timeout_sec:.1f}s "
            f"topic={self.contact_approach_cmd_vel_topic} "
            f"speed_x={self.contact_approach_speed_x:.3f}m/s "
            f"timeout={self.contact_approach_timeout_sec:.2f}s "
            f"max_distance={self.contact_approach_max_distance:.3f}m "
            f"third_speed_x={self.contact_approach_third_speed_x:.3f}m/s "
            f"third_timeout={self.contact_approach_third_timeout_sec:.2f}s "
            f"third_max_distance={self.contact_approach_third_max_distance:.3f}m"
        )

    def start_initial_navigation(self):
        if getattr(self, "_blue_initial_navigation_deferred", False):
            return

        with self.lock:
            if self.initial_navigation_started:
                return
            if self.current_odom_pose is None:
                self.get_logger().warn(
                    "蓝方挑战测试：尚未收到 /Odometry，等待里程计后再发布相对导航点"
                )
                return

            offset = self.CHALLENGE_NAVIGATION_OFFSET
            current_pose = self.current_odom_pose
            first_point = Point3d(
                current_pose.x + offset.x,
                current_pose.y + offset.y,
                current_pose.z + offset.z,
                math.atan2(
                    math.sin(current_pose.yaw + offset.yaw),
                    math.cos(current_pose.yaw + offset.yaw),
                ),
            )

            self.publish_mode1_and_enable_contact_logic("挑战蓝方测试启动")
            self.initial_navigation_started = True
            self._challenge_point_cursor = 0
            self._challenge_phase = "goto_challenge_barcode_point"
            self.pending_shift_return_reason = "挑战蓝方测试启动后首个相对导航点"
            self.pending_shift_return_force_final = False
            self.pending_shift_return_action = "final"
            self.active_nav_target = NavTarget.SHIFT_POINT_RETURN
            self.active_nav_index = self.point_index
            self.current_state = RobotState.NAVIGATING
            self.set_chassis_mode(
                ChassisMode.NAVIGATION,
                "挑战蓝方测试启动后前往相对当前位置的导航点 1",
            )
            self.publish_goal(first_point, "挑战蓝方测试相对导航点 1")
            self.send_serial("8")
            self.stop_serial("8")

        self.get_logger().info(
            "蓝方挑战测试：已基于当前 odom 发布相对导航点 1，"
            f"current=({current_pose.x:.2f}, {current_pose.y:.2f}, "
            f"{current_pose.z:.1f}, {current_pose.yaw:.2f})，"
            f"offset=({offset.x:.2f}, {offset.y:.2f}, "
            f"{offset.z:.1f}, {offset.yaw:.2f})，"
            f"goal=({first_point.x:.2f}, {first_point.y:.2f}, "
            f"{first_point.z:.1f}, {first_point.yaw:.2f})，并同步发送命令8（无需回包）"
        )
        self.start_force_mode9_timer_after_match_start()

    def odom_callback(self, msg):
        super().odom_callback(msg)
        if self.start_serial_trigger_received and not self.initial_navigation_started:
            self.start_initial_navigation()

    def start_force_mode9_timer_after_match_start(self):
        with self.lock:
            if (
                self._force_mode9_timer is not None
                or self._force_mode9_finished
                or self._force_mode9_waiting_reply
                or not self.initial_navigation_started
            ):
                return

            self._force_mode9_timer = self.create_timer(
                self.force_mode9_timeout_sec,
                self.force_mode9_and_publish_mode2,
            )
        self.get_logger().info(
            "蓝方挑战测试：收到开始比赛信息后开始两分钟计时，"
            f"{self.force_mode9_timeout_sec:.1f}s 后强制发送命令11"
        )

    def _timed_out_or_finished(self) -> bool:
        return (
            self._force_mode9_waiting_reply
            or self._force_mode9_finished
            or self.current_state == RobotState.FINISHED
        )

    def force_mode9_and_publish_mode2(self):
        with self.lock:
            if self._force_mode9_finished or self._force_mode9_waiting_reply:
                return

            self._force_mode9_waiting_reply = True
            if self._force_mode9_timer is not None:
                self._force_mode9_timer.cancel()
                self._force_mode9_timer = None

            if self._direct_contact_timer is not None:
                self._direct_contact_timer.cancel()
                self._direct_contact_timer = None
            self._direct_contact_active = False
            self._direct_contact_round_number = None
            self._active_contact_approach_speed_x = None
            self._active_contact_approach_timeout_sec = None
            self._active_contact_approach_max_distance = None
            self._active_contact_approach_command_period_sec = None
            self._challenge_phase = None
            self._challenge_after_cmd9_action = None
            self._challenge_waiting_intermediate_final_reply = False
            self.active_nav_target = NavTarget.NONE
            self.active_nav_index = None
            self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY

            self.publish_direct_contact_stop()
            self.send_serial("11")
            self.get_logger().warn(
                "蓝方挑战测试：启动两分钟计时到达，已强制发送区域1模式11，等待回包后发布 /mode=2"
            )

    def handle_serial_frame(self, frame: dict):
        if self._force_mode9_finished:
            return

        with self.lock:
            if (
                self._force_mode9_waiting_reply
                and frame["mode"] == SERIAL_COMMANDS["11"]["mode"]
                and frame["task_complete"] == 1
            ):
                self.stop_serial("11")
                self._force_mode9_waiting_reply = False
                self._force_mode9_finished = True
                self.current_state = RobotState.FINISHED
                self.publish_mode2_and_release_hold(
                    "蓝方挑战测试：启动两分钟超时，区域1模式11回包完成，发布 /mode=2"
                )
                self.stop_serial_reader_after_mode2()
                self.get_logger().warn(
                    "蓝方挑战测试：超时区域1模式11已收到完成回包，已发布 /mode=2，状态 -> FINISHED"
                )
                return

            if self._force_mode9_waiting_reply:
                return

            if (
                self._challenge_after_cmd9_action == "finish"
                and frame["mode"] == SERIAL_COMMANDS["9"]["mode"]
                and frame["task_complete"] == 1
            ):
                self.stop_serial("9")
                self._challenge_after_cmd9_action = None
                self._challenge_phase = None
                self._cmd9_reply_count += 1
                if self._cmd9_reply_count >= self.MODE2_AFTER_CMD9_REPLY_COUNT:
                    self.publish_mode2_after_third_cmd9_reply()
                    return

                self.start_direct_contact_approach()
                self.get_logger().info(
                    f"蓝方挑战测试：收到第 {self._cmd9_reply_count}/"
                    f"{self.MODE2_AFTER_CMD9_REPLY_COUNT} 次命令9完成回包，"
                    "直接触发固定后退顶靠"
                )
                return

        super().handle_serial_frame(frame)

    def nav_callback(self, msg):
        if self._timed_out_or_finished():
            return
        super().nav_callback(msg)

    def barcode_callback(self, msg):
        if self._timed_out_or_finished():
            return
        super().barcode_callback(msg)

    def send_challenge_scan_cmd9_after_cmd8_delay(self):
        if self._timed_out_or_finished():
            return
        super().send_challenge_scan_cmd9_after_cmd8_delay()

    def send_challenge_cmd7_after_cmd1_delay(self, reason: str, point_number: int):
        if self._timed_out_or_finished():
            return
        super().send_challenge_cmd7_after_cmd1_delay(reason, point_number)

    def start_direct_contact_approach(self):
        if self._timed_out_or_finished():
            return

        if self._direct_contact_active:
            return

        self._direct_contact_round_number = self._cmd9_reply_count + 1
        if self._direct_contact_round_number == 3:
            self._active_contact_approach_speed_x = self.contact_approach_third_speed_x
            self._active_contact_approach_timeout_sec = self.contact_approach_third_timeout_sec
            self._active_contact_approach_max_distance = self.contact_approach_third_max_distance
            self._active_contact_approach_command_period_sec = (
                self.contact_approach_third_command_period_sec
            )
        else:
            self._active_contact_approach_speed_x = self.contact_approach_speed_x
            self._active_contact_approach_timeout_sec = self.contact_approach_timeout_sec
            self._active_contact_approach_max_distance = self.contact_approach_max_distance
            self._active_contact_approach_command_period_sec = (
                self.contact_approach_command_period_sec
            )

        self._direct_contact_active = True
        self._challenge_phase = "direct_contact_approach"
        self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
        self._direct_contact_start_time = self.get_clock().now().nanoseconds / 1e9
        self._direct_contact_start_pose = self.current_odom_pose
        self.mode_pub.publish(Int32(data=self.CONTACT_APPROACH_MODE_VALUE))
        self.publish_direct_contact_velocity()
        self._direct_contact_timer = self.create_timer(
            self._active_contact_approach_command_period_sec,
            self.handle_direct_contact_approach,
        )
        self.get_logger().warn(
            f"蓝方挑战测试：已切换底盘 CONTACT_APPROACH 并开始发布第 "
            f"{self._direct_contact_round_number} 轮固定后退顶靠速度 "
            f"speed_x={self._active_contact_approach_speed_x:.3f}m/s "
            f"timeout={self._active_contact_approach_timeout_sec:.2f}s "
            f"max_distance={self._active_contact_approach_max_distance:.3f}m"
        )

    def publish_direct_contact_velocity(self):
        msg = Twist()
        msg.linear.x = (
            self._active_contact_approach_speed_x
            if self._active_contact_approach_speed_x is not None
            else self.contact_approach_speed_x
        )
        self.direct_contact_cmd_pub.publish(msg)

    def publish_direct_contact_stop(self):
        msg = Twist()
        for _ in range(20):
            self.direct_contact_cmd_pub.publish(msg)

    def get_direct_contact_distance(self):
        if self._direct_contact_start_pose is None or self.current_odom_pose is None:
            return None

        return math.hypot(
            self.current_odom_pose.x - self._direct_contact_start_pose.x,
            self.current_odom_pose.y - self._direct_contact_start_pose.y,
        )

    def handle_direct_contact_approach(self):
        if self._timed_out_or_finished() or not self._direct_contact_active:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self._direct_contact_start_time
        traveled = self.get_direct_contact_distance()
        active_timeout = (
            self._active_contact_approach_timeout_sec
            if self._active_contact_approach_timeout_sec is not None
            else self.contact_approach_timeout_sec
        )
        active_max_distance = (
            self._active_contact_approach_max_distance
            if self._active_contact_approach_max_distance is not None
            else self.contact_approach_max_distance
        )
        if elapsed >= active_timeout:
            self.finish_direct_contact_approach(
                f"固定后退顶靠超时 {elapsed:.2f}s"
            )
            return

        if (
            traveled is not None
            and active_max_distance > 0.0
            and traveled >= active_max_distance
        ):
            self.finish_direct_contact_approach(
                f"固定后退顶靠距离达到 {traveled:.3f}m"
            )
            return

        self.publish_direct_contact_velocity()

    def finish_direct_contact_approach(self, reason: str):
        if not self._direct_contact_active:
            return

        self._direct_contact_active = False
        if self._direct_contact_timer is not None:
            self._direct_contact_timer.cancel()
            self._direct_contact_timer = None
        self.publish_direct_contact_stop()
        self.mode_pub.publish(Int32(data=int(ChassisMode.HOLD)))
        self.current_chassis_mode = ChassisMode.HOLD
        self._direct_contact_round_number = None
        self._active_contact_approach_speed_x = None
        self._active_contact_approach_timeout_sec = None
        self._active_contact_approach_max_distance = None
        self._active_contact_approach_command_period_sec = None
        self.get_logger().warn(f"蓝方挑战测试：{reason}，已停车并切回 HOLD，进入正常夹取流程")
        self.start_post_contact_grab()

    def start_post_contact_grab(self):
        if self._timed_out_or_finished():
            return

        self._challenge_phase = "waiting_challenge_cmd1_delay"
        self.current_state = RobotState.WAITING_SHIFT_PREPARE_RESULT
        point_number = self._cmd9_reply_count + 1
        reason = f"蓝方挑战测试第 {point_number} 轮顶靠完成"
        self.set_chassis_mode(
            ChassisMode.HOLD,
            f"{reason}，先执行命令1再延时发送命令7",
        )
        self.send_serial("1")
        self.stop_serial("1")
        threading.Timer(
            self.CHALLENGE_CMD1_TO_CMD7_DELAY,
            self.send_challenge_cmd7_after_cmd1_delay,
            args=(reason, point_number),
        ).start()
        self.get_logger().info(
            f"蓝方挑战测试：顶靠结束后已发送命令1；"
            f"{self.CHALLENGE_CMD1_TO_CMD7_DELAY:.2f}s 后发送命令7，"
            "随后进入 TF 对位和扫码"
        )

    def publish_mode2_after_third_cmd9_reply(self):
        self._force_mode9_finished = True
        if self._force_mode9_timer is not None:
            self._force_mode9_timer.cancel()
            self._force_mode9_timer = None
        self.current_state = RobotState.FINISHED
        self.publish_mode2_and_release_hold(
            "蓝方挑战测试：第三次命令9回包完成，发布 /mode=2"
        )
        self.stop_serial_reader_after_mode2()
        self.get_logger().info(
            "蓝方挑战测试：第三次收到命令9回包，已发布 /mode=2，状态 -> FINISHED"
        )

    def destroy_node(self):
        if self._force_mode9_timer is not None:
            self._force_mode9_timer.cancel()
            self._force_mode9_timer = None
        if self._direct_contact_timer is not None:
            self._direct_contact_timer.cancel()
            self._direct_contact_timer = None
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = R1Mode1ChallengeBlueTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
