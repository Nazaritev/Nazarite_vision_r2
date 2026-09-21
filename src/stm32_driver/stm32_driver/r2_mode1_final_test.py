#!/usr/bin/env python3
import os
import sys
import tempfile
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from stm32_driver.r2_mode1 import R2_mode1


class R2Mode1FinalTest(R2_mode1):
    def __init__(self):
        try:
            super().__init__(node_name="r2_mode1_final_test")
        except TypeError:
            super().__init__()

        self.serial_handed_off = False

        startup_timer = getattr(self, "startup_timer", None)
        if startup_timer is not None:
            startup_timer.cancel()

        self.start_initial_navigation()
        self.get_logger().info(
            "测试模式：启动后直接前往最终点，保留命令9回包、发布 /mode=2；"
            "命令9完成回包后释放串口给 r2_global_matrix_node，节点本身保持运行"
        )

    def get_final_result_cmd(self) -> str:
        return "9"

    def start_initial_navigation(self):
        with self.lock:
            if self.initial_navigation_started:
                return
            self.publish_mode1_and_enable_contact_logic("最终点测试启动")
            self.initial_navigation_started = True
            self.navigate_to_final_point()
        self.get_logger().info("启动即发布最终点导航目标")

    def handle_spin_completed(self):
        previous_state = getattr(self.current_state, "name", "")
        super().handle_spin_completed()
        if previous_state == "WAITING_FINAL_RESULT" and getattr(self.current_state, "name", "") == "FINISHED":
            self.release_serial_for_mode2_handoff()

    def release_serial_for_mode2_handoff(self):
        if self.serial_handed_off:
            return

        self.node_running = False
        self.stop_serial()

        if self.serial and self.serial.is_open:
            serial_port = getattr(self.serial, "port", "串口")
            self.serial.close()
            self.get_logger().info(
                f"测试模式：命令9完成回包后已停止串口接收并释放 {serial_port}，"
                "可供 r2_global_matrix_node 使用"
            )

        self.serial_handed_off = True

    def final_exit(self):
        if self.serial_handed_off:
            self.get_logger().info("测试模式：最终点流程结束，保持节点运行；串口已释放给后续节点")
            return

        self.get_logger().info("测试模式：最终点流程结束，保持节点运行，不退出节点，不关闭串口")


def configure_ros_log_dir():
    if os.environ.get("ROS_LOG_DIR"):
        return

    fallback_log_dir = Path(tempfile.gettempdir()) / "ros_logs" / "r2_mode1_final_test"
    fallback_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(fallback_log_dir)


def parse_start_mode(argv: list[str]) -> tuple[bool, list[str]]:
    start_embedded_nav_agent = False
    ros_args = [argv[0]]

    for arg in argv[1:]:
        if arg == "--with-nav-agent":
            start_embedded_nav_agent = True
            continue
        if arg == "--external-nav-agent":
            start_embedded_nav_agent = False
            continue
        ros_args.append(arg)

    return start_embedded_nav_agent, ros_args


def main():
    configure_ros_log_dir()
    start_embedded_nav_agent, ros_args = parse_start_mode(sys.argv)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=ros_args)
    node = None
    nav_agent_node = None
    executor = MultiThreadedExecutor()
    try:
        if start_embedded_nav_agent:
            from stm32_driver.nav_agent import NavAgent

            nav_agent_node = NavAgent()
            executor.add_node(nav_agent_node)

        node = R2Mode1FinalTest()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if nav_agent_node:
            nav_agent_node.destroy_node()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
