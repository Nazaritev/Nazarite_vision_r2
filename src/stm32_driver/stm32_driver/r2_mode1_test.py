#!/usr/bin/env python3
import os
import sys
import tempfile
from pathlib import Path

try:
    from stm32_driver.r2_mode1 import RobotState, Point3d, R2_mode1
except ModuleNotFoundError:
    # Support direct execution via `python3 path/to/r2_mode1_test6.py`.
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    from stm32_driver.r2_mode1 import RobotState, Point3d, R2_mode1

class R2Mode1Test6(R2_mode1):
    def __init__(self, start_target: str = "grab"):
        try:
            super().__init__(node_name="r2_mode1_test6")
        except TypeError:
            # Keep compatibility with older R2_mode1 versions that do not
            # accept a custom node name.
            super().__init__()

        # 6 个测试抓取点 / 平移点，延续当前 3 点布局向后展开。
        self.grab_points = [
            Point3d(-0.890, 0.430, 0.0, 0.0),
            Point3d(-0.890, 0.630, 0.0, 0.0),
            Point3d(-0.880, 0.780, 0.0, 0.0),
            Point3d(-0.880, 0.980, 0.0, 0.0),
            Point3d(-0.880, 0.980, 0.0, 0.0),
            Point3d(-0.880, 1.180, 0.0, 0.0),
        ]
        self.shift_points = [
            Point3d(-0.580, 0.420, 0.0, 0.0),
            Point3d(-0.525, 0.380, 0.0, 0.0),
            Point3d(-0.525, 0.580, 0.0, 0.0),
            Point3d(-0.525, 0.780, 0.0, 0.0),
            Point3d(-0.525, 0.980, 0.0, 0.0),
            Point3d(-0.525, 1.180, 0.0, 0.0),
        ]
        self.max_grab_count = len(self.grab_points)
        self.wait_for_vision_before_start = False

        if start_target == "shift" and hasattr(self, "navigate_to_shift_point"):
            startup_timer = getattr(self, "startup_timer", None)
            if startup_timer is not None:
                startup_timer.cancel()
            if hasattr(self, "initial_navigation_started"):
                self.initial_navigation_started = True
            self.publish_mode1_and_enable_contact_logic("6 点测试启动平移点模式")
            self.navigate_to_shift_point(0)
            self.get_logger().info("测试模式：启动后直接前往平移点 0，用于验证平移点容差与收敛表现")

        self.get_logger().info("已切换到 6 抓取点测试状态机：保留平移点、TF 对位、扫码后前进 0.3m 并执行回正链路")
        self.get_logger().info("测试模式：启动时不再等待视觉首帧，其余抓取检测与抓取反馈逻辑保持原样")

    def navigate_to_final_point(self):
        # 测试版没有最终导航点；最后一个流程收尾后直接停止。
        self.current_state = RobotState.FINISHED
        self.set_chassis_mode(self.current_chassis_mode, "6 点测试流程结束")
        self.get_logger().info("6 个抓取点流程已结束，不再前往最终点，直接停止节点")
        self.final_exit()


def configure_ros_log_dir():
    if os.environ.get("ROS_LOG_DIR"):
        return

    fallback_log_dir = Path(tempfile.gettempdir()) / "ros_logs" / "r2_mode1_test6"
    fallback_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(fallback_log_dir)


def parse_start_mode(argv: list[str]) -> tuple[bool, str, list[str]]:
    start_embedded_nav_agent = False
    start_target = "grab"
    ros_args = [argv[0]]

    for arg in argv[1:]:
        if arg == "--with-nav-agent":
            start_embedded_nav_agent = True
            continue
        if arg == "--external-nav-agent":
            start_embedded_nav_agent = False
            continue
        if arg == "--start-shift-point":
            start_target = "shift"
            continue
        if arg == "--start-grab-point":
            start_target = "grab"
            continue
        ros_args.append(arg)

    return start_embedded_nav_agent, start_target, ros_args


def main():
    configure_ros_log_dir()
    start_embedded_nav_agent, start_target, ros_args = parse_start_mode(sys.argv)

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

        node = R2Mode1Test6(start_target=start_target)
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
