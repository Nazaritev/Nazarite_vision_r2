#!/usr/bin/env python3
import sys
import threading
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

import rclpy

from stm32_driver.r2_mode1 import (
    SERIAL_COMMANDS,
    ChassisMode,
    NavTarget,
    Point3d,
    RobotState,
)
from stm32_driver.r2_mode1_blue import R2_mode1_blue


class R1Mode1ChallengeBlue(R2_mode1_blue):
    CHALLENGE_BARCODE_POINTS = (
        Point3d(-0.80, -0.57, 0.0, 0.0),
        Point3d(-0.80, -0.80, 0.0, 0.0),
        Point3d(-0.80, -1.00, 0.0, 0.0),
    )
    CHALLENGE_RELATIVE_OFFSETS = {
        1: -0.39,
        2: -0.6,
    }
    CHALLENGE_CMD1_TO_CMD7_DELAY = 0.5
    CHALLENGE_SCAN_CMD8_TO_CMD9_DELAY = 0.5

    def __init__(self, node_name: str = "r1_mode1_challenge_blue"):
        self._challenge_phase = None
        self._challenge_point_cursor = 0
        self._challenge_pending_reason = None
        self._challenge_pending_force_final = False
        self._challenge_pending_action = "final"
        self._challenge_after_cmd9_action = None
        self._challenge_waiting_intermediate_final_reply = False
        super().__init__(node_name=node_name)

    def start_initial_navigation(self):
        if getattr(self, "_blue_initial_navigation_deferred", False):
            return
        with self.lock:
            if self.initial_navigation_started:
                return
            self.publish_mode1_and_enable_contact_logic("挑战蓝方启动")
            self.initial_navigation_started = True
            self._challenge_point_cursor = 0
            self._challenge_phase = "goto_challenge_barcode_point"
            self.pending_shift_return_reason = "挑战蓝方启动后首个导航点"
            self.pending_shift_return_force_final = False
            self.pending_shift_return_action = "final"
            self.active_nav_target = NavTarget.SHIFT_POINT_RETURN
            self.active_nav_index = self.point_index
            self.current_state = RobotState.NAVIGATING
            first_point = self.CHALLENGE_BARCODE_POINTS[0]
            self.set_chassis_mode(
                ChassisMode.NAVIGATION,
                "挑战蓝方启动后前往 challenge 导航点 1",
            )
            self.publish_goal(first_point, "挑战蓝方导航点 1")
            self.send_serial("8")
            self.stop_serial("8")
        self.get_logger().info(
            "挑战蓝方启动导航点使用 CHALLENGE_BARCODE_POINTS[0]，"
            f"已发布导航点 1: Point3d({first_point.x:.2f}, {first_point.y:.2f}, "
            f"{first_point.z:.1f}, {first_point.yaw:.1f})，并同步发送命令8（无需回包）"
        )

    def schedule_shift_return_after_barcode(
        self,
        reason: str,
        force_final: bool = False,
        action: str = "final",
    ):
        if self._challenge_phase == "waiting_challenge_barcode":
            self.handle_challenge_barcode_complete(reason, force_final, action)
            return

        self.get_logger().warn(
            f"忽略非挑战扫码等待阶段的扫码后调度: phase={self._challenge_phase} "
            f"state={self.current_state.name} reason={reason}"
        )

    def start_challenge_navigation_after_scan(
        self,
        reason: str,
        force_final: bool,
        action: str,
        send_scan_cmd9: bool,
    ):
        if self._challenge_point_cursor >= len(self.CHALLENGE_BARCODE_POINTS):
            self._challenge_phase = None
            R2_mode1_blue.schedule_shift_return_after_barcode(
                self,
                reason,
                force_final=force_final,
                action=action,
            )
            return

        self.pending_shift_return_reason = reason
        self.pending_shift_return_force_final = force_final
        self.pending_shift_return_index = self.point_index
        self.pending_shift_return_action = action
        self._challenge_pending_reason = reason
        self._challenge_pending_force_final = force_final
        self._challenge_pending_action = action
        challenge_point = self.CHALLENGE_BARCODE_POINTS[self._challenge_point_cursor]
        if self._challenge_point_cursor in self.CHALLENGE_RELATIVE_OFFSETS:
            challenge_point = self.build_relative_challenge_point(
                self._challenge_point_cursor
            )
            if challenge_point is None:
                return
        point_number = self._challenge_point_cursor + 1
        if send_scan_cmd9:
            self._challenge_phase = "waiting_cmd9_before_navigation"
            self._challenge_after_cmd9_action = "navigate"
            self.active_nav_target = NavTarget.NONE
            self.active_nav_index = None
            self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
            self.set_chassis_mode(
                ChassisMode.HOLD,
                f"挑战蓝方扫码后等待命令9回包，再前往挑战导航点 {point_number}",
            )
            self.send_serial("9")
            self.get_logger().info(
                f"{reason}，挑战蓝方扫码后已发送命令9，等待回包后再前往导航点 {point_number}: "
                f"Point3d({challenge_point.x:.2f}, {challenge_point.y:.2f}, "
                f"{challenge_point.z:.1f}, {challenge_point.yaw:.1f})"
            )
            return

        self._challenge_phase = "goto_challenge_barcode_point"
        self.active_nav_target = NavTarget.SHIFT_POINT_RETURN
        self.active_nav_index = self.point_index
        self.current_state = RobotState.NAVIGATING
        self.set_chassis_mode(
            ChassisMode.NAVIGATION,
            f"挑战蓝方扫码后前往挑战导航点 {point_number}",
        )
        self.publish_goal(challenge_point, f"挑战蓝方导航点 {point_number}")
        self.get_logger().info(
            f"{reason}，挑战蓝方命令9回包完成，正在前往导航点 {point_number}: "
            f"Point3d({challenge_point.x:.2f}, {challenge_point.y:.2f}, "
            f"{challenge_point.z:.1f}, {challenge_point.yaw:.1f})"
        )

    def build_relative_challenge_point(self, point_cursor: int) -> Point3d | None:
        if self.current_odom_pose is None:
            self.get_logger().warn(
                f"挑战蓝方导航点 {point_cursor + 1} 需要基于当前 odom 相对发布，"
                "但尚未收到 /Odometry，暂不发布导航点"
            )
            return None

        offset = self.CHALLENGE_RELATIVE_OFFSETS[point_cursor]
        base_point = self.CHALLENGE_BARCODE_POINTS[point_cursor]
        relative_point = Point3d(
            base_point.x,
            self.current_odom_pose.y + offset,
            base_point.z,
            base_point.yaw,
        )
        self.get_logger().info(
            f"挑战蓝方导航点 {point_cursor + 1} 保持 x={base_point.x:.2f}，"
            f"y 使用当前 odom 相对偏移 {offset:.2f}: "
            f"current_y={self.current_odom_pose.y:.2f} -> "
            f"goal=({relative_point.x:.2f}, {relative_point.y:.2f})"
        )
        return relative_point

    def barcode_callback(self, msg):
        with self.lock:
            if self._challenge_phase != "waiting_challenge_barcode":
                if self._challenge_phase is not None:
                    self.get_logger().debug(
                        f"挑战流程阶段 {self._challenge_phase} 中忽略重复扫码: {msg.data.strip()}"
                    )
                    return
                return super().barcode_callback(msg)

            barcode_data = msg.data.strip()
            if barcode_data not in ("1", "100", "200", "300"):
                return

            barcode_label = f"第{self._challenge_point_cursor + 1}个挑战点"
            if barcode_data == "300":
                reason = f"挑战蓝方{barcode_label}二维码 300 判定本次抓取失败"
                action = "retry_next"
                force_final = False
            elif barcode_data == "200":
                reason = f"挑战蓝方{barcode_label}特殊二维码 200，扫码流程完成"
                action = "final"
                force_final = True
            else:
                reason = f"挑战蓝方{barcode_label}普通二维码 100，扫码流程完成"
                action = self.pending_shift_return_action
                force_final = self.pending_shift_return_force_final

            self.handle_challenge_barcode_complete(reason, force_final, action)

    def handle_challenge_barcode_complete(
        self,
        reason: str,
        force_final: bool,
        action: str,
    ):
        is_last_challenge_point = (
            self._challenge_point_cursor >= len(self.CHALLENGE_BARCODE_POINTS) - 1
        )
        self._challenge_pending_reason = reason
        self._challenge_pending_force_final = force_final
        self._challenge_pending_action = action

        if is_last_challenge_point:
            self._challenge_phase = "waiting_cmd8_before_finish"
            self._challenge_after_cmd9_action = None
            self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
            self.send_serial("8")
            self.stop_serial("8")
            threading.Timer(
                self.CHALLENGE_SCAN_CMD8_TO_CMD9_DELAY,
                self.send_challenge_scan_cmd9_after_cmd8_delay,
            ).start()
            self.get_logger().info(
                f"{reason}，挑战蓝方最后一个导航点扫码后已发送命令8；"
                f"{self.CHALLENGE_SCAN_CMD8_TO_CMD9_DELAY:.1f}s 后发送命令9，"
                "等待命令9回包后发布 /mode=2 并结束"
            )
            return

        self._challenge_phase = "waiting_cmd8_before_next_navigation"
        self._challenge_after_cmd9_action = None
        self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
        self.send_serial("8")
        self.stop_serial("8")
        threading.Timer(
            self.CHALLENGE_SCAN_CMD8_TO_CMD9_DELAY,
            self.send_challenge_scan_cmd9_after_cmd8_delay,
        ).start()
        self.get_logger().info(
            f"{reason}，挑战蓝方导航点 {self._challenge_point_cursor + 1} "
            f"扫码后已发送命令8；{self.CHALLENGE_SCAN_CMD8_TO_CMD9_DELAY:.1f}s 后发送命令9，"
            "等待命令9回包后发布下一个导航点"
        )
        return
    
    def send_challenge_scan_cmd9_after_cmd8_delay(self):
        with self.lock:
            if self._challenge_phase == "waiting_cmd8_before_next_navigation":
                self._challenge_phase = "waiting_cmd9_before_next_navigation"
                self._challenge_after_cmd9_action = "next_navigation"
                self.send_serial("9")
                self.get_logger().info(
                    "挑战蓝方扫码后命令8延时完成，已发送命令9；"
                    "等待9回包后发布下一个导航点"
                )
                return

            if self._challenge_phase == "waiting_cmd8_before_finish":
                self._challenge_phase = "waiting_cmd9_before_finish"
                self._challenge_after_cmd9_action = "finish"
                self.send_serial("9")
                self.get_logger().info(
                    "挑战蓝方最后一个导航点扫码后命令8延时完成，已发送命令9；"
                    "等待9回包后发布 /mode=2 并结束"
                )

    def stop_serial_reader_after_mode2(self):
        self.node_running = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self.get_logger().info("已发布 /mode=2，关闭串口并退出串口读取线程")
        except Exception as e:
            self.get_logger().warn(f"关闭串口读取线程失败: {e}")

    def start_intermediate_post_sequence_after_cmd9(self):
        self._challenge_phase = "intermediate_post_sequence"
        self.send_serial("8")
        self.stop_serial("8")
        threading.Timer(
            self.BLUE_POST_BARCODE_CMD8_DELAY,
            self.finish_intermediate_post_barcode_after_cmd8,
        ).start()

    def finish_intermediate_post_barcode_after_cmd8(self):
        with self.lock:
            if self._challenge_phase != "intermediate_post_sequence":
                return
            self.send_serial("3")
            self.stop_serial("3")
            threading.Timer(
                self.BLUE_POST_BARCODE_CMD3_DELAY,
                self.finish_intermediate_post_barcode_after_cmd3,
            ).start()

    def finish_intermediate_post_barcode_after_cmd3(self):
        with self.lock:
            if self._challenge_phase != "intermediate_post_sequence":
                return
            final_cmd = self.get_final_result_cmd()
            self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
            self._challenge_waiting_intermediate_final_reply = True
            self.send_serial(final_cmd)
            self.get_logger().info(
                f"挑战蓝方中间后续指令已发送最终命令{final_cmd}，"
                "等待回包后发布下一个挑战导航点"
            )

    def send_challenge_cmd7_after_cmd1_delay(self, reason: str, point_number: int):
        with self.lock:
            if (
                self._challenge_phase != "waiting_challenge_cmd1_delay"
                or self.current_state != RobotState.WAITING_SHIFT_PREPARE_RESULT
            ):
                return
            self._challenge_phase = "waiting_challenge_barcode"
            self.send_serial("7")
            self.get_logger().info(
                f"{reason}，挑战蓝方导航点 {point_number} 命令1延时 "
                f"{self.CHALLENGE_CMD1_TO_CMD7_DELAY:.2f}s 结束，已发送命令7；"
                "等待回包后进入 TF 对位和扫码"
            )

    def handle_serial_frame(self, frame: dict):
        with self.lock:
            final_cmd = self.get_final_result_cmd()
            if (
                self._challenge_phase == "waiting_challenge_barcode"
                and frame["mode"] == SERIAL_COMMANDS["7"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_SHIFT_PREPARE_RESULT
            ):
                self.stop_serial("7")
                self.set_chassis_mode(
                    ChassisMode.HOLD,
                    "挑战蓝方命令7完成，准备进入 TF 对位和扫码",
                )
                self.current_state = RobotState.ALIGNING_TF
                self.get_logger().info(
                    "挑战蓝方收到命令7完成回包，开始 TF 对位，随后等待扫码"
                )
                return

            if (
                self._challenge_after_cmd9_action is not None
                and frame["mode"] == SERIAL_COMMANDS["9"]["mode"]
                and frame["task_complete"] == 1
            ):
                self.stop_serial("9")
                action_after_cmd9 = self._challenge_after_cmd9_action
                self._challenge_after_cmd9_action = None

                if action_after_cmd9 == "navigate":
                    self.start_challenge_navigation_after_scan(
                        self._challenge_pending_reason or "挑战蓝方命令9回包完成",
                        force_final=self._challenge_pending_force_final,
                        action=self._challenge_pending_action,
                        send_scan_cmd9=False,
                    )
                    return

                if action_after_cmd9 == "next_navigation":
                    self._challenge_point_cursor += 1
                    self.get_logger().info(
                        "挑战蓝方扫码后命令9回包完成，发布下一个挑战导航点"
                    )
                    self.start_challenge_navigation_after_scan(
                        self._challenge_pending_reason or "挑战蓝方扫码后命令9回包完成",
                        force_final=self._challenge_pending_force_final,
                        action=self._challenge_pending_action,
                        send_scan_cmd9=False,
                    )
                    return

                if action_after_cmd9 == "finish":
                    self._challenge_phase = None
                    self.current_state = RobotState.FINISHED
                    self.publish_mode2_and_release_hold(
                        "挑战蓝方最后一次命令9回包完成，释放底盘给后续导航"
                    )
                    self.stop_serial_reader_after_mode2()
                    self.get_logger().info(
                        "挑战蓝方最后一个导航点扫码后命令9回包完成，"
                        "已发布 /mode=2，状态 -> FINISHED"
                    )
                    return

                if action_after_cmd9 == "final_sequence":
                    self._challenge_phase = None
                    self.current_state = RobotState.FINISHED
                    self.publish_mode2_and_release_hold(
                        "挑战蓝方最后一次命令9回包完成，释放底盘给后续导航"
                    )
                    self.stop_serial_reader_after_mode2()
                    self.get_logger().info(
                        "挑战蓝方最后一个导航点扫码后命令9回包完成，"
                        "已发布 /mode=2，状态 -> FINISHED"
                    )
                    return

            if (
                self._challenge_waiting_intermediate_final_reply
                and frame["mode"] == SERIAL_COMMANDS[final_cmd]["mode"]
                and frame["task_complete"] == 1
            ):
                self.stop_serial(final_cmd)
                self._challenge_waiting_intermediate_final_reply = False
                self._challenge_point_cursor += 1
                self.get_logger().info(
                    f"挑战蓝方收到中间最终命令{final_cmd}回包，准备发布下一个挑战导航点"
                )
                self.start_challenge_navigation_after_scan(
                    self._challenge_pending_reason or "挑战蓝方中间流程完成",
                    force_final=self._challenge_pending_force_final,
                    action=self._challenge_pending_action,
                    send_scan_cmd9=False,
                )
                return

        super().handle_serial_frame(frame)

    def nav_callback(self, msg):
        with self.lock:
            if (
                msg.data
                and self._challenge_phase == "goto_challenge_barcode_point"
                and self.active_nav_target == NavTarget.SHIFT_POINT_RETURN
            ):
                self.active_nav_target = NavTarget.NONE
                self.active_nav_index = None
                reason = self.pending_shift_return_reason or "挑战蓝方扫码后过渡点到达"
                if self.active_serial_cmd == "9":
                    self.stop_serial("9")

                self._challenge_phase = "waiting_challenge_cmd1_delay"
                self.current_state = RobotState.WAITING_SHIFT_PREPARE_RESULT
                point_number = self._challenge_point_cursor + 1
                self.set_chassis_mode(
                    ChassisMode.HOLD,
                    f"挑战蓝方导航点 {point_number} 已到达，先执行命令1再延时发送命令7",
                )
                self.send_serial("1")
                self.stop_serial("1")
                threading.Timer(
                    self.CHALLENGE_CMD1_TO_CMD7_DELAY,
                    self.send_challenge_cmd7_after_cmd1_delay,
                    args=(reason, point_number),
                ).start()
                self.get_logger().info(
                    f"{reason}，挑战蓝方导航点 {point_number} 已到达，已发送命令1；"
                    f"{self.CHALLENGE_CMD1_TO_CMD7_DELAY:.2f}s 后发送命令7"
                )
                return

        super().nav_callback(msg)


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = R1Mode1ChallengeBlue()
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
