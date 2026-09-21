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
    R2_mode1,
    RobotState,
)


class R2_mode1_blue(R2_mode1):
    BLUE_DEFAULT_START_GRAB_INDEX = 2
    BLUE_CMD1_TO_CMD7_DELAY = 1.0
    BLUE_FINAL_CMD_DELAY = 15.0
    BLUE_POST_BARCODE_CMD8_DELAY = 1.2
    BLUE_POST_BARCODE_CMD3_DELAY = 0.5
    BLUE_QR2_CMD8_TO_CMD10_DELAY = 0.5
    BLUE_FINAL_MODE2_DELAY = 0.0
    BLUE_GRAB_POINTS = [
        Point3d(-0.80, 0.09, 0.0, 0.0),
        Point3d(-0.80, -0.11, 0.0, 0.0),
        Point3d(-0.80, -0.41, 0.0, 0.0),
        Point3d(-0.80, -0.73, 0.0, 0.0),
        Point3d(-0.80, -0.93, 0.0, 0.0),
        Point3d(-0.80, -1.10, 0.0, 0.0),
    ]

    def __init__(self, node_name: str = "r2_mode1_blue"):
        self._blue_initial_navigation_deferred = True
        try:
            super().__init__(node_name=node_name)
        except TypeError:
            super().__init__()

        self.apply_blue_grab_points()
        self.apply_blue_mirror_config()
        self.open_loop_grab_depart_delay = self.BLUE_CMD1_TO_CMD7_DELAY
        self.blue_qr2_serial_step = None
        self._blue_initial_navigation_deferred = False
        self.get_logger().info("蓝方等待串口启动帧: region=1 mode=0，收到后再发送首个抓取点")

    def apply_blue_grab_points(self):
        self.grab_points = [
            Point3d(point.x, point.y, point.z, point.yaw)
            for point in self.BLUE_GRAB_POINTS
        ]
        self.max_grab_count = len(self.grab_points)
        self.get_logger().info(
            f"蓝方独立抓取点已启用：{len(self.grab_points)} 个点位，y 轴已按蓝方取反"
        )

    def start_initial_navigation(self):
        if getattr(self, "_blue_initial_navigation_deferred", False):
            return
        with self.lock:
            if self.initial_navigation_started:
                return
            self.initial_navigation_started = True
            self.initial_grab_index = self.resolve_start_grab_index()
            self.current_state = RobotState.WAITING_INITIAL_CMD8_RESULT
            self.set_chassis_mode(ChassisMode.HOLD, "蓝方启动初始流程，先等待命令8回包")
        self.send_serial("8")
        self.get_logger().info(
            f"蓝方启动流程已发送命令8，等待回包后跳过首个导航点并直接顶靠，起始抓取点 {self.initial_grab_index}"
        )

    def resolve_start_grab_index(self) -> int:
        index = super().resolve_start_grab_index()
        if (
            index == 0
            and len(self.grab_points) > self.BLUE_DEFAULT_START_GRAB_INDEX
        ):
            default_point = self.grab_points[self.BLUE_DEFAULT_START_GRAB_INDEX]
            self.get_logger().info(
                "蓝方起始抓取点使用默认索引 "
                f"{self.BLUE_DEFAULT_START_GRAB_INDEX}: "
                f"({default_point.x:.2f}, {default_point.y:.2f}, "
                f"{default_point.z:.1f}, {default_point.yaw:.1f})"
            )
            return self.BLUE_DEFAULT_START_GRAB_INDEX
        return index

    def apply_blue_mirror_config(self):
        original_align_target_y = float(self.align_target_y)
        self.align_target_y = -original_align_target_y
        self.mirror_goal_points()
        self.get_logger().info(
            "蓝方镜像模式已启用："
            f"align_target_y {original_align_target_y:.3f} -> {self.align_target_y:.3f}，"
            "预设点位 y/yaw 与左右自旋底盘模式已反转"
        )

    def mirror_goal_points(self):
        for point in self.shift_points:
            point.y = -point.y
            point.yaw = -point.yaw
        self.final_point.y = -self.final_point.y
        self.final_point.yaw = -self.final_point.yaw

    def get_final_result_cmd(self) -> str:
        return "9"

    def barcode_callback(self, msg):
        barcode_data = msg.data.strip()
        if barcode_data != "2":
            super().barcode_callback(msg)
            return

        with self.lock:
            if self.current_state not in (
                RobotState.ALIGNING_TF,
                RobotState.WAITING_BARCODE,
            ):
                return

            if self.active_serial_cmd == "2":
                self.stop_serial("2")

            self.use_cmd10_for_next_shift_prepare = False
            self.blue_qr2_serial_step = "cmd8_delay"
            self.current_state = RobotState.WAITING_SHIFT_PREPARE_RESULT
            self.set_chassis_mode(
                ChassisMode.HOLD,
                "蓝方二维码2，执行串口序列8->10，收到10回包后前往下一个抓取点并顶靠",
            )
            self.send_serial("8")
            self.stop_serial("8")
            threading.Timer(
                self.BLUE_QR2_CMD8_TO_CMD10_DELAY,
                self.send_blue_qr2_cmd10_after_delay,
            ).start()
            self.get_logger().info(
                "蓝方二维码2：已发送命令8（无需回包）；"
                f"{self.BLUE_QR2_CMD8_TO_CMD10_DELAY:.1f}s 后发送命令10"
            )

    def send_blue_qr2_cmd10_after_delay(self):
        with self.lock:
            if self.blue_qr2_serial_step != "cmd8_delay":
                return

            self.blue_qr2_serial_step = "waiting_10"
            self.send_serial("10")
            self.get_logger().info(
                "蓝方二维码2：命令8后延时结束，已发送命令10，等待命令10回包"
            )

    def handle_serial_frame(self, frame: dict):
        if self.blue_qr2_serial_step is not None:
            if self.handle_blue_qr2_serial_frame(frame):
                return
        super().handle_serial_frame(frame)

    def handle_blue_qr2_serial_frame(self, frame: dict) -> bool:
        self.get_logger().info(
            "⬅ 收到有效回包: "
            f"region={frame['region']} mode={frame['mode']} "
            f"task_complete={frame['task_complete']} data={frame['data']}"
        )

        with self.lock:
            step = self.blue_qr2_serial_step
            if step is None:
                return False

            if frame["task_complete"] != 1:
                return True

            if step == "waiting_10":
                if frame["mode"] != SERIAL_COMMANDS["10"]["mode"]:
                    return True
                self.stop_serial("10")
                self.blue_qr2_serial_step = None
                self.use_cmd10_for_next_shift_prepare = False
                next_index = self.get_next_grab_index(self.point_index)
                if next_index is None:
                    self.get_logger().warn(
                        "蓝方二维码2：收到命令10完成回包，但没有下一个抓取点，转入后续目标处理"
                    )
                    self.advance_to_next_target(
                        "蓝方二维码2收到命令10完成回包但无下一个抓取点",
                        success_completed=False,
                    )
                    return True

                self.publish_mode1_and_enable_contact_logic("蓝方二维码2收到命令10回包，重新开启顶靠逻辑")
                self.navigate_to_grab_point(next_index)
                self.get_logger().info(
                    f"蓝方二维码2：收到命令10完成回包，前往抓取点 {next_index}；"
                    "已重新发布 /mode=1，到点后执行顶靠/预抓取，再按命令1->7回包流程继续扫码"
                )
                return True

            return True

    def navigate_to_shift_point(self, index: int):
        self.cancel_pre_grab_open_loop_timer_locked()
        self.active_nav_target = NavTarget.NONE
        self.active_nav_index = None
        self.current_state = RobotState.WAITING_SHIFT_PREPARE_RESULT
        prepare_cmd = "10" if self.use_cmd10_for_next_shift_prepare else "7"
        self.use_cmd10_for_next_shift_prepare = False
        self.set_chassis_mode(
            ChassisMode.HOLD,
            f"蓝方抓取完成，跳过平移点 {index}，等待电控命令{prepare_cmd}流程完成",
        )
        self.send_serial(prepare_cmd)
        if prepare_cmd == "10":
            self.get_logger().info(
                f"蓝方抓取点 {index} 抓取后跳过平移点，已发送命令10；"
                "等待命令10回包后发送命令7，再等待命令7回包进入 TF 对位和扫码"
            )
            return
        self.get_logger().info(
            f"蓝方抓取点 {index} 抓取后跳过平移点，已发送命令{prepare_cmd}；"
            "等待回包后进入 TF 对位和扫码"
        )

    def schedule_shift_return_after_barcode(
        self,
        reason: str,
        force_final: bool = False,
        action: str = "final",
    ):
        self.pending_shift_return_reason = reason
        self.pending_shift_return_force_final = force_final
        self.pending_shift_return_index = self.point_index
        self.pending_shift_return_action = action
        self.current_state = (
            RobotState.WAITING_SHIFT_RETURN_RESULT
            if action == "retry_next"
            else RobotState.WAITING_FINAL_RESULT
        )
        self.set_chassis_mode(
            ChassisMode.HOLD,
            "蓝方扫码完成，跳过扫码后 odom 0.3m 移动",
        )
        self.start_post_barcode_cmd8_sequence(reason, action)

    def start_post_barcode_cmd8_sequence(self, reason: str, action: str):
        with self.lock:
            if self.current_state not in (
                RobotState.WAITING_SHIFT_RETURN_RESULT,
                RobotState.WAITING_FINAL_RESULT,
            ):
                return

        self.send_serial("8")
        self.stop_serial("8")
        threading.Timer(
            self.BLUE_POST_BARCODE_CMD8_DELAY,
            self.finish_post_barcode_after_cmd8,
            args=(reason, action),
        ).start()

    def finish_post_barcode_after_cmd8(self, reason: str, action: str):
        with self.lock:
            if self.current_state not in (
                RobotState.WAITING_SHIFT_RETURN_RESULT,
                RobotState.WAITING_FINAL_RESULT,
            ):
                return

        self.send_serial("3")
        self.stop_serial("3")
        threading.Timer(
            self.BLUE_POST_BARCODE_CMD3_DELAY,
            self.finish_post_barcode_after_cmd3,
            args=(reason, action),
        ).start()

    def finish_post_barcode_after_cmd3(self, reason: str, action: str):
        with self.lock:
            if self.current_state not in (
                RobotState.WAITING_SHIFT_RETURN_RESULT,
                RobotState.WAITING_FINAL_RESULT,
            ):
                return

        if action == "retry_next":
            self.send_serial("4")
            self.get_logger().info(
                f"{reason}，蓝方已跳过扫码后 0.3m 移动；"
                f"命令8后延时 {self.BLUE_POST_BARCODE_CMD8_DELAY:.1f}s，"
                f"命令3后延时 {self.BLUE_POST_BARCODE_CMD3_DELAY:.1f}s，"
                "已发送命令4，等待命令4回包后前往下一个抓取点"
            )
            return

        threading.Timer(
            self.BLUE_FINAL_CMD_DELAY,
            self.send_final_cmd_after_delay,
            args=(reason,),
        ).start()
        self.get_logger().info(
            f"{reason}，蓝方已跳过扫码后 0.3m 移动；"
            f"命令8后延时 {self.BLUE_POST_BARCODE_CMD8_DELAY:.1f}s，"
            f"命令3后延时 {self.BLUE_POST_BARCODE_CMD3_DELAY:.1f}s，"
            f"{self.BLUE_FINAL_CMD_DELAY:.1f}s 后发送最终命令{self.get_final_result_cmd()}"
        )

    def send_final_cmd_after_delay(
        self,
        reason: str,
        publish_mode2_after_send: bool = False,
    ):
        with self.lock:
            if self.current_state != RobotState.WAITING_FINAL_RESULT:
                return

        final_cmd = self.get_final_result_cmd()
        self.send_serial(final_cmd)
        self.get_logger().info(
            f"{reason}，蓝方已跳过扫码后 0.3m 移动；"
            f"命令8后延时 {self.BLUE_POST_BARCODE_CMD8_DELAY:.1f}s，"
            f"命令3后延时 {self.BLUE_POST_BARCODE_CMD3_DELAY:.1f}s，"
            f"命令3后等待 {self.BLUE_FINAL_CMD_DELAY:.1f}s，"
            f"已发送最终命令{final_cmd}，等待最终回包；"
            f"收到回包后 {self.BLUE_FINAL_MODE2_DELAY:.1f}s 发布 /mode=2"
        )

    def handle_spin_completed(self):
        if self.current_state != RobotState.WAITING_FINAL_RESULT:
            super().handle_spin_completed()
            return

        final_cmd = self.get_final_result_cmd()
        self.stop_serial(final_cmd)
        self.current_state = RobotState.FINISHED
        threading.Timer(
            self.BLUE_FINAL_MODE2_DELAY,
            self.publish_delayed_mode2_after_final_cmd,
            args=(final_cmd,),
        ).start()
        self.get_logger().info(
            f"收到命令{final_cmd}完成回包，"
            f"已发布 /mode=2 | 状态 -> FINISHED"
        )

    def publish_delayed_mode2_after_final_cmd(self, final_cmd: str):
        with self.lock:
            if self.current_state != RobotState.FINISHED:
                return

            self.publish_mode2_and_release_hold(
                f"蓝方最终命令{final_cmd}完成后延时 {self.BLUE_FINAL_MODE2_DELAY:.1f}s，释放底盘给后续导航",
            )
        self.get_logger().info(
            f"蓝方最终命令{final_cmd}完成后延时 {self.BLUE_FINAL_MODE2_DELAY:.1f}s，已发布 /mode=2"
        )
        threading.Timer(2.0, self.final_exit).start()

    @staticmethod
    def swap_spin_mode(mode: ChassisMode) -> ChassisMode:
        if mode == ChassisMode.SPIN_FORWARD:
            return ChassisMode.SPIN_BACKWARD
        if mode == ChassisMode.SPIN_BACKWARD:
            return ChassisMode.SPIN_FORWARD
        return mode

    def set_chassis_mode(self, mode: ChassisMode, reason: str = ""):
        super().set_chassis_mode(self.swap_spin_mode(mode), reason)


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = R2_mode1_blue()
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
