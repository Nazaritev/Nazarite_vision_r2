#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Bool, String, Float32
import tf2_ros
from tf2_ros import TransformException
import serial
import math
import time
import threading
import sys
import struct
from enum import Enum, IntEnum, auto

CHASSIS_MODE_TOPIC = "/chassis_mode"
PRE_GRAB_REACHED_TOPIC = "/agent/pre_grab_reached"


class RobotState(Enum):
    INIT = auto()               # 初始化状态
    NAVIGATING = auto()         # 导航移动中
    STABILIZING = auto()        # 到达目标点，等待稳定
    DETECTING = auto()          # 检测物体（颜色+深度）
    WAITING_GRAB_RESULT = auto()  # 已发送1抓取，等待抓取结果判定
    WAITING_SHIFT_PREPARE_RESULT = auto()  # 首次到达平移点后已发送7，等待电控完成前置动作
    SPINNING_FORWARD = auto()   # 到达平移点后正向自旋 180 度
    ALIGNING_TF = auto()        # 正向自旋结束后等待 TF 对位
    WAITING_BARCODE = auto()    # 已发送2，等待二维码(100/200/300)
    WAITING_SHIFT_RETURN_DELAY = auto()  # 扫码后已发送3，等待延时后前往前进点
    SPINNING_BACK = auto()      # 扫码完成后反向自旋回原朝向
    WAITING_SHIFT_RETURN_RESULT = auto()  # 300恢复流程到达前进点后已发送4，等待电控回包
    WAITING_FINAL_RESULT = auto()  # 到达最终点/扫码后前进点后已发送最终流程命令，等待电控完成回包
    FINISHED = auto()           # 最终流程完成


class NavTarget(Enum):
    NONE = auto()
    GRAB_POINT = auto()
    SHIFT_POINT = auto()
    SHIFT_POINT_RETURN = auto()
    FINAL_POINT = auto()


class ChassisMode(IntEnum):
    NAVIGATION = 0
    SPIN_FORWARD = 1
    HOLD = 2
    SPIN_BACKWARD = 3


class Point3d:
    def __init__(self, x_, y_, z_, yaw_=0.0):
        self.x = x_
        self.y = y_
        self.z = z_
        self.yaw = yaw_


FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55
FRAME_PAYLOAD_LENGTH = 7
FRAME_LENGTH = 11

SERIAL_COMMANDS = {
    "1": {"region": 0x01, "mode": 0x01, "need_return": 0, "data": 0},
    "2": {"region": 0x01, "mode": 0x02, "need_return": 0, "data": 0},
    "3": {"region": 0x01, "mode": 0x03, "need_return": 0, "data": 0},
    "4": {"region": 0x01, "mode": 0x04, "need_return": 1, "data": 0},
    "5": {"region": 0x01, "mode": 0x05, "need_return": 0, "data": 0},
    "6": {"region": 0x01, "mode": 0x06, "need_return": 1, "data": 0},
    "7": {"region": 0x01, "mode": 0x07, "need_return": 1, "data": 0},
    "8": {"region": 0x01, "mode": 0x08, "need_return": 0, "data": 0},
    "9": {"region": 0x01, "mode": 0x09, "need_return": 1, "data": 0},
}


def modbus_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder="little")


def build_vision_frame(region: int, mode: int, need_return: int, data_value: int = 0) -> bytes:
    data_bytes = struct.pack("<h", data_value)
    payload = bytes([region, mode, need_return]) + data_bytes + b"\x00\x00"
    crc = modbus_crc16(payload)
    return bytes([FRAME_HEADER]) + payload + crc + bytes([FRAME_TAIL])


def parse_vision_frame(frame: bytes) -> dict | None:
    if len(frame) != FRAME_LENGTH:
        return None
    if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        return None

    payload = frame[1:1 + FRAME_PAYLOAD_LENGTH]
    crc_received = frame[1 + FRAME_PAYLOAD_LENGTH:1 + FRAME_PAYLOAD_LENGTH + 2]
    crc_calc = modbus_crc16(payload)

    return {
        "region": payload[0],
        "mode": payload[1],
        "task_complete": payload[2],
        "data": struct.unpack("<h", payload[3:5])[0],
        "reserve": payload[5:7],
        "crc_ok": crc_calc == crc_received,
        "raw": frame.hex(),
    }


class R2_mode1(Node):
    def __init__(self, node_name: str = "r2_mode1"):
        super().__init__(node_name)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        selector_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 参数 
        self.declare_parameter('target_tag_frame', 'base')
        self.declare_parameter('reference_frame', 'camera_link')
        self.declare_parameter('align_target_y', 0.0) #可改
        self.declare_parameter('align_tolerance', 0.01)
        self.declare_parameter('stabilization_time', 0.0) #抓取点到了的稳定时间
        self.declare_parameter('spin_target_angle_deg', 180.0)
        self.declare_parameter('spin_angle_tolerance_deg', 2.0)
        self.declare_parameter('spin_control_mode', 'serial4')
        self.declare_parameter('spin_angle_source', 'imu')
        self.declare_parameter('spin_imu_topic', '/livox/imu')
        self.declare_parameter('spin_imu_timeout', 0.5)
        self.declare_parameter('spin_auto_fallback_to_odom', True)
        self.declare_parameter('grab_goal_checker_id', 'grab_goal_checker')
        self.declare_parameter('shift_goal_checker_id', 'shift_goal_checker')
        self.declare_parameter('final_goal_checker_id', 'shift_goal_checker')
        self.declare_parameter('goal_checker_selector_topic', '/goal_checker_selector')
        self.declare_parameter('post_barcode_shift_delay', 3.0)
        self.declare_parameter('dynamic_shift_forward_distance', 0.3)
        self.declare_parameter('preferred_start_grab_index', 0)
        self.declare_parameter('required_success_count', 1)
        self.declare_parameter('open_loop_grab_enabled', True)
        self.declare_parameter('open_loop_grab_depart_delay', 0.2)
        self.declare_parameter('pre_grab_open_loop_delay', 0.8)
        self.declare_parameter('finish_after_first_success', False)  # 旧参数保留兼容，不再参与流程判断
        self.target_tag_frame = self.get_parameter('target_tag_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.align_target_y = self.get_parameter('align_target_y').value
        self.align_tolerance = self.get_parameter('align_tolerance').value
        self.stabilization_time = self.get_parameter('stabilization_time').value
        self.spin_target_angle = math.radians(self.get_parameter('spin_target_angle_deg').value)
        self.spin_angle_tolerance = math.radians(self.get_parameter('spin_angle_tolerance_deg').value)
        self.spin_control_mode = str(self.get_parameter('spin_control_mode').value).strip().lower()
        if self.spin_control_mode not in ("serial4", "local"):
            self.get_logger().warn(
                f"未知 spin_control_mode={self.spin_control_mode}，回退到 serial4"
            )
            self.spin_control_mode = "serial4"
        self.spin_angle_source = str(self.get_parameter('spin_angle_source').value).strip().lower()
        if self.spin_angle_source not in ("imu", "odom"):
            self.get_logger().warn(
                f"未知 spin_angle_source={self.spin_angle_source}，回退到 odom"
            )
            self.spin_angle_source = "odom"
        self.spin_imu_topic = self.get_parameter('spin_imu_topic').value
        self.spin_imu_timeout = float(self.get_parameter('spin_imu_timeout').value)
        self.spin_auto_fallback_to_odom = bool(self.get_parameter('spin_auto_fallback_to_odom').value)
        self.grab_goal_checker_id = str(self.get_parameter('grab_goal_checker_id').value).strip()
        self.shift_goal_checker_id = str(self.get_parameter('shift_goal_checker_id').value).strip()
        self.final_goal_checker_id = str(self.get_parameter('final_goal_checker_id').value).strip()
        self.goal_checker_selector_topic = self.get_parameter('goal_checker_selector_topic').value
        self.post_barcode_shift_delay = float(self.get_parameter('post_barcode_shift_delay').value)
        self.dynamic_shift_forward_distance = float(
            self.get_parameter('dynamic_shift_forward_distance').value
        )
        self.preferred_start_grab_index = int(
            self.get_parameter('preferred_start_grab_index').value
        )
        self.required_success_count = max(
            1,
            int(self.get_parameter('required_success_count').value),
        )
        self.open_loop_grab_enabled = bool(
            self.get_parameter('open_loop_grab_enabled').value
        )
        self.open_loop_grab_depart_delay = max(
            0.0,
            float(self.get_parameter('open_loop_grab_depart_delay').value),
        )
        self.pre_grab_open_loop_delay = float(
            self.get_parameter('pre_grab_open_loop_delay').value
        )
        self.finish_after_first_success = bool(
            self.get_parameter('finish_after_first_success').value
        )

        # 串口初始化
        self.declare_parameter('serial_port', '/dev/ttyserial')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        self.serial = None
        self.serial_rx_buffer = bytearray()
        self.node_running = True
        # 仅记录最近一次发给电控、尚未被状态机清理的命令；不再用于重复发送。
        self.active_serial_cmd = None
        try:
            self.serial = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=1.0,
                exclusive=True,
            )
            self.get_logger().info(f'串口连接成功: {serial_port} | 波特率: {baudrate}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {str(e)}')

        self.grab_points = [
            Point3d(-0.88, 0.41, 0.0, 0.0),
            Point3d(-0.80, 0.59, 0.0, 0.0),
            Point3d(-0.80, 0.79, 0.0, 0.0),
            Point3d(-0.80, 0.99, 0.0, 0.0),
            Point3d(-0.80, 1.19, 0.0, 0.0),
            Point3d(-0.80, 0.19, 0.0, 0.0),
        ]
        self.shift_points = [
            Point3d(-0.60, 0.40, 0.0, 0.0),
            Point3d(-0.60, 0.59, 0.0, 0.0),
            Point3d(-0.60, 0.79, 0.0, 0.0),
            Point3d(-0.60, 0.99, 0.0, 0.0),
            Point3d(-0.60, 1.19, 0.0, 0.0),
            Point3d(-0.60, 0.19, 0.0, 0.0),
        ]
        self.final_point = Point3d(1.70, 1.4, 0.0, 0.0)
        self.point_index = 0
        self.grab_count = 0
        self.success_count = 0
        self.max_grab_count = len(self.grab_points)
        self.active_nav_target = NavTarget.NONE
        self.active_nav_index = None

        # --- 核心改变：引入状态机 ---
        self.current_state = RobotState.INIT
        self.special_barcode_triggered = False
        self.current_chassis_mode = ChassisMode.NAVIGATION
        self.initial_navigation_started = False
        self.current_goal_checker_id = None

        # 抓取反馈专用变量（独立于主状态机的并行检测）
        self.checking_grab_feedback = False
        self.pre_grab_depth = 0.0
        self.current_depth = 0.0
        self.depth_sample_received = False
        self.object_present = False
        self.object_detected_rx_count = 0
        self.object_depth_rx_count = 0
        self.grab_start_time = 0.0
        self.grab_check_delay = 1.00
        self.grab_depth_tolerance = 0.05
        self.last_spin_yaw = None
        self.last_imu_rx_time = None
        self.imu_last_sample_time = None
        self.imu_integrated_yaw = 0.0
        self.imu_start_frame_locked = False
        self.spin_accumulated_yaw = 0.0
        self.spin_source_in_use = "odom"
        self.pending_post_spin_reason = None
        self.pending_post_spin_force_final = False
        self.pending_post_spin_send_cmd3 = True
        self.pending_shift_return_reason = None
        self.pending_shift_return_force_final = False
        self.pending_shift_return_index = None
        self.pending_shift_return_action = "final"
        self.active_spin_target_angle = self.spin_target_angle
        self.current_odom_pose = None
        self.pre_grab_open_loop_timer = None
        self.pre_grab_open_loop_index = None

        self.lock = threading.RLock()

        # ROS 接口
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.tf_check_loop)
        self.nav_sub = self.create_subscription(Bool, "/agent/arrival_status", self.nav_callback, qos)
        self.pre_grab_sub = self.create_subscription(
            Bool, PRE_GRAB_REACHED_TOPIC, self.pre_grab_reached_callback, qos
        )
        self.odom_sub = self.create_subscription(Odometry, "/Odometry", self.odom_callback, qos)
        self.imu_sub = None
        if self.spin_angle_source == "imu":
            self.imu_sub = self.create_subscription(Imu, self.spin_imu_topic, self.imu_callback, qos)
        self.mode_pub = self.create_publisher(Int32, CHASSIS_MODE_TOPIC, qos)
        self.external_mode_pub = self.create_publisher(Int32, "/mode", qos)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", qos)
        self.barcode_sub = self.create_subscription(String, "/barcode", self.barcode_callback, 10)
        self.goal_checker_selector_pub = self.create_publisher(
            String,
            self.goal_checker_selector_topic,
            selector_qos,
        )
        # 颜色+深度检测订阅
        self.object_sub = self.create_subscription(
            Bool, "/object_detected", self.object_status_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Float32, "/object_depth", self.object_depth_callback, qos_profile_sensor_data
        )
        self.grab_feedback_timer = self.create_timer(0.1, self.grab_feedback_timer_callback)

        # 串口线程
        self.serial_thread = threading.Thread(target=self.listen_stm32_data, daemon=True)
        self.serial_thread.start()
        self.get_logger().info(
            "R2_mode1 初始化完成（Enum 状态机重构版），启动后直接发送首个抓取点"
        )
        self.get_logger().info(f"自旋控制模式: {self.spin_control_mode}")
        self.get_logger().info(
            f"自旋角度源配置: {self.describe_spin_source(self.spin_angle_source)}"
            f" | IMU 超时回退 odom={self.spin_auto_fallback_to_odom}"
        )
        self.get_logger().info("串口命令发送模式: 单次下发（不重复发送）")
        self.get_logger().info(
            "抓取点策略: "
            f"preferred_start_grab_index={self.preferred_start_grab_index} | "
            f"required_success_count={self.required_success_count}"
        )
        self.get_logger().info(
            "开环抓取: "
            f"enabled={self.open_loop_grab_enabled} | "
            f"depart_delay={self.open_loop_grab_depart_delay:.2f}s | "
            f"pre_grab_delay={self.pre_grab_open_loop_delay:.2f}s"
        )
        self.start_initial_navigation()

    def start_initial_navigation(self):
        with self.lock:
            if self.initial_navigation_started:
                return
            self.initial_navigation_started = True
            start_index = self.resolve_start_grab_index()
            self.navigate_to_grab_point(start_index)
            self.send_serial("8")
            self.stop_serial("8")
        self.get_logger().info(
            f"启动即发布第一个导航点，目标抓取点 {start_index}，并同步发送命令8（无需回包）"
        )

    def get_final_result_cmd(self) -> str:
        return "6"

    @staticmethod
    def describe_qos_policy(policy) -> str:
        return getattr(policy, "name", str(policy))

    def log_sensor_topic_debug(self, reason: str):
        for topic_name, rx_count in (
            ("/object_detected", self.object_detected_rx_count),
            ("/object_depth", self.object_depth_rx_count),
        ):
            pub_count = self.count_publishers(topic_name)
            infos = self.get_publishers_info_by_topic(topic_name)
            self.get_logger().info(
                f"[视觉订阅自检] {reason} | topic={topic_name} | rx_count={rx_count} | publishers={pub_count}"
            )
            for info in infos:
                qos = info.qos_profile
                self.get_logger().info(
                    "[视觉订阅自检] "
                    f"topic={topic_name} | pub_node={info.node_namespace}/{info.node_name} "
                    f"| type={info.topic_type} | reliability={self.describe_qos_policy(qos.reliability)} "
                    f"| durability={self.describe_qos_policy(qos.durability)} | depth={qos.depth}"
                )

    def select_goal_checker_for_target(self, checker_id: str, target_name: str):
        checker_id = checker_id.strip()
        if not checker_id:
            self.get_logger().warn(f"目标={target_name} 未配置 goal checker，沿用当前设置")
            return
        if self.current_goal_checker_id == checker_id:
            return
        self.goal_checker_selector_pub.publish(String(data=checker_id))
        self.current_goal_checker_id = checker_id
        self.get_logger().info(f"已切换 goal checker -> {checker_id} | 目标={target_name}")

    def navigate_to_grab_point(self, index: int):
        self.cancel_pre_grab_open_loop_timer_locked()
        self.point_index = index
        self.active_nav_target = NavTarget.GRAB_POINT
        self.active_nav_index = index
        self.current_state = RobotState.NAVIGATING
        self.select_goal_checker_for_target(self.grab_goal_checker_id, f"抓取点 {index}")
        self.set_chassis_mode(ChassisMode.NAVIGATION, f"准备前往抓取点 {index}")
        self.publish_goal(self.grab_points[index], f"抓取点 {index}")

    @staticmethod
    def clone_point(point: Point3d) -> Point3d:
        return Point3d(point.x, point.y, point.z, point.yaw)

    def resolve_start_grab_index(self) -> int:
        if not self.grab_points:
            return 0

        if self.preferred_start_grab_index < 0:
            self.get_logger().warn(
                f"preferred_start_grab_index={self.preferred_start_grab_index} 非法，回退到 0"
            )
            return 0

        if self.preferred_start_grab_index >= len(self.grab_points):
            fallback_index = len(self.grab_points) - 1
            self.get_logger().warn(
                "preferred_start_grab_index 超出抓取点数量，"
                f"回退到最后一个抓取点 {fallback_index}"
            )
            return fallback_index

        return self.preferred_start_grab_index

    def get_next_grab_index(self, current_index: int) -> int | None:
        next_index = current_index + 1
        if next_index >= len(self.grab_points):
            return None
        return next_index

    def build_forward_goal_from_current_pose(self, distance: float) -> Point3d | None:
        if self.current_odom_pose is None:
            return None

        return Point3d(
            self.current_odom_pose.x + distance,
            self.current_odom_pose.y,
            self.current_odom_pose.z,
            self.current_odom_pose.yaw,
        )

    def navigate_to_shift_point(self, index: int):
        self.cancel_pre_grab_open_loop_timer_locked()
        self.active_nav_target = NavTarget.SHIFT_POINT
        self.active_nav_index = index
        self.current_state = RobotState.NAVIGATING
        self.select_goal_checker_for_target(self.shift_goal_checker_id, f"平移点 {index}")
        self.set_chassis_mode(ChassisMode.NAVIGATION, f"抓取成功，前往平移点 {index}")
        self.publish_goal(self.shift_points[index], f"平移点 {index}")

    def navigate_to_shift_point_for_return(self, index: int):
        self.cancel_pre_grab_open_loop_timer_locked()
        goal_point = self.build_forward_goal_from_current_pose(self.dynamic_shift_forward_distance)
        if goal_point is None:
            self.get_logger().warn(
                "扫码后尚未收到 /Odometry，前进点回退到对应平移点 "
                f"{index}"
            )
            goal_point = self.clone_point(self.shift_points[index])
        else:
            current_pose = self.current_odom_pose
            self.get_logger().info(
                "扫码后使用 /Odometry 当前位置生成前进点: "
                f"({current_pose.x:.3f}, {current_pose.y:.3f}) -> "
                f"({goal_point.x:.3f}, {goal_point.y:.3f})"
            )
        self.active_nav_target = NavTarget.SHIFT_POINT_RETURN
        self.active_nav_index = index
        self.current_state = RobotState.NAVIGATING
        self.select_goal_checker_for_target(self.shift_goal_checker_id, f"扫码后前进点 {index}")
        self.set_chassis_mode(
            ChassisMode.NAVIGATION,
            f"扫码完成，基于当前位置前进 {self.dynamic_shift_forward_distance:.3f}m"
        )
        self.publish_goal(goal_point, f"扫码后前进点 {index}")

    def navigate_to_final_point(self):
        self.cancel_pre_grab_open_loop_timer_locked()
        self.active_nav_target = NavTarget.FINAL_POINT
        self.active_nav_index = None
        self.current_state = RobotState.NAVIGATING
        self.select_goal_checker_for_target(self.final_goal_checker_id, "最终点")
        self.set_chassis_mode(ChassisMode.NAVIGATION, "准备前往最终点")
        self.publish_goal(self.final_point, "最终点")

    def set_chassis_mode(self, mode: ChassisMode, reason: str = ""):
        self.mode_pub.publish(Int32(data=int(mode)))
        if self.current_chassis_mode != mode:
            self.current_chassis_mode = mode
            suffix = f" | {reason}" if reason else ""
            self.get_logger().info(f"切换底盘模式 -> {mode.name}{suffix}")

    def cancel_pre_grab_open_loop_timer_locked(self):
        timer = self.pre_grab_open_loop_timer
        self.pre_grab_open_loop_timer = None
        self.pre_grab_open_loop_index = None
        if timer is not None:
            timer.cancel()

    def pre_grab_reached_callback(self, msg: Bool):
        if not msg.data:
            return

        with self.lock:
            if not self.open_loop_grab_enabled or self.pre_grab_open_loop_delay < 0.0:
                return
            if self.active_nav_target != NavTarget.GRAB_POINT or self.active_nav_index is None:
                return
            if self.current_state != RobotState.NAVIGATING:
                return

            nav_index = self.active_nav_index
            self.cancel_pre_grab_open_loop_timer_locked()
            timer = threading.Timer(
                self.pre_grab_open_loop_delay,
                self.trigger_pre_grab_open_loop_grab,
                args=(nav_index,),
            )
            timer.daemon = True
            self.pre_grab_open_loop_timer = timer
            self.pre_grab_open_loop_index = nav_index
            timer.start()
            self.get_logger().info(
                f"预抓取点 {nav_index} 已到达，"
                f"{self.pre_grab_open_loop_delay:.2f}s 后提前执行开环抓取"
            )

    def trigger_pre_grab_open_loop_grab(self, nav_index: int):
        with self.lock:
            if self.pre_grab_open_loop_index != nav_index:
                return

            self.pre_grab_open_loop_timer = None
            self.pre_grab_open_loop_index = None

            if not self.open_loop_grab_enabled or self.pre_grab_open_loop_delay < 0.0:
                return
            if self.active_nav_target != NavTarget.GRAB_POINT or self.active_nav_index != nav_index:
                return
            if self.current_state != RobotState.NAVIGATING:
                return

            self.active_nav_target = NavTarget.NONE
            self.active_nav_index = None
            self.set_chassis_mode(
                ChassisMode.HOLD,
                f"预抓取点 {nav_index} 延时 {self.pre_grab_open_loop_delay:.2f}s，提前执行开环抓取",
            )
            self.get_logger().info(
                f"预抓取点 {nav_index} 延时结束，跳过顶靠成功等待，直接发送命令1"
            )
            self.start_open_loop_grab()

    def reset_spin_tracking(self):
        self.last_spin_yaw = None
        self.spin_accumulated_yaw = 0.0

    def is_spinning_state(self) -> bool:
        return self.current_state in (
            RobotState.SPINNING_FORWARD,
            RobotState.SPINNING_BACK,
        )

    def describe_spin_source(self, source: str) -> str:
        if source == "imu":
            return f"IMU({self.spin_imu_topic})"
        return "ODOM(/Odometry)"

    def select_spin_source(self) -> str:
        if self.spin_angle_source != "imu":
            return "odom"

        if not self.spin_auto_fallback_to_odom:
            return "imu"

        if self.last_imu_rx_time is None:
            self.get_logger().warn("未收到 IMU 数据，本轮自旋回退到 ODOM")
            return "odom"

        imu_age = time.monotonic() - self.last_imu_rx_time
        if imu_age > self.spin_imu_timeout:
            self.get_logger().warn(
                f"IMU 数据超时 {imu_age:.2f}s，本轮自旋回退到 ODOM"
            )
            return "odom"

        return "imu"

    def prepare_spin_tracking(self) -> str:
        self.reset_spin_tracking()
        self.spin_source_in_use = self.select_spin_source()
        source_desc = self.describe_spin_source(self.spin_source_in_use)
        if self.spin_source_in_use == "imu" and self.imu_start_frame_locked:
            self.last_spin_yaw = self.imu_integrated_yaw
            self.get_logger().info(
                "使用启动时首帧 IMU 作为全局零点，"
                f"当前累计角度 {math.degrees(self.imu_integrated_yaw):.1f} deg，作为本轮自旋起点"
            )
        self.get_logger().info(f"自旋角度累计源 -> {source_desc}")
        return source_desc

    def maybe_fallback_spin_source_to_odom(self, current_odom_yaw: float) -> bool:
        if self.spin_source_in_use != "imu" or not self.spin_auto_fallback_to_odom:
            return False

        if self.last_imu_rx_time is None:
            imu_age = float("inf")
        else:
            imu_age = time.monotonic() - self.last_imu_rx_time

        if imu_age <= self.spin_imu_timeout:
            return False

        self.spin_source_in_use = "odom"
        self.last_spin_yaw = current_odom_yaw
        self.get_logger().warn(
            f"自旋过程中 IMU 数据超时 {imu_age:.2f}s，回退到 ODOM 继续累计角度"
        )
        return True

    def update_spin_with_yaw(self, yaw: float, source_desc: str):
        if self.last_spin_yaw is None:
            self.last_spin_yaw = yaw
            if self.spin_accumulated_yaw > 0.0:
                self.get_logger().info(
                    f"已切换到 {source_desc}，保留已累计角度 {math.degrees(self.spin_accumulated_yaw):.1f} deg"
                )
            else:
                self.get_logger().info(f"已锁定 {source_desc} 自旋起始朝向，开始累计旋转角度")
            return

        delta_yaw = abs(self.normalize_angle(yaw - self.last_spin_yaw))
        self.last_spin_yaw = yaw
        if delta_yaw < 1e-4:
            return

        self.spin_accumulated_yaw += delta_yaw
        if self.spin_accumulated_yaw >= max(0.0, self.active_spin_target_angle - self.spin_angle_tolerance):
            self.finish_spin()

    def update_spin_with_imu(self):
        if self.last_spin_yaw is None:
            self.last_spin_yaw = self.imu_integrated_yaw
            self.get_logger().info(
                "IMU 自旋起点尚未就绪，已使用当前累计 IMU 角度补锁起点"
            )
            return

        delta_yaw = abs(self.imu_integrated_yaw - self.last_spin_yaw)
        self.last_spin_yaw = self.imu_integrated_yaw
        if delta_yaw < 1e-4:
            return

        self.spin_accumulated_yaw += delta_yaw
        if self.spin_accumulated_yaw >= max(0.0, self.active_spin_target_angle - self.spin_angle_tolerance):
            self.finish_spin()

    def start_spin_mode(self, shift_index: int | None):
        if self.spin_control_mode == "local":
            self.current_state = RobotState.SPINNING_FORWARD
            self.active_spin_target_angle = self.spin_target_angle
            source_desc = self.prepare_spin_tracking()
            self.set_chassis_mode(ChassisMode.SPIN_FORWARD, f"到达平移点 {shift_index}，开始自旋 180 度")
            self.get_logger().info(f"平移点 {shift_index} 已到达，等待 {source_desc} 累计旋转角度")
            return

        self.current_state = RobotState.WAITING_SHIFT_PREPARE_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, f"到达平移点 {shift_index}，等待电控命令7流程完成")
        self.send_serial("7")
        self.get_logger().info(
            f"平移点 {shift_index} 已到达，已发送命令7；"
            "底盘保持 HOLD，等待电控回包7 后再进入 TF 对位与扫码流程"
        )

    def start_spin_back(self, reason: str, force_final: bool = False, send_cmd3_after_spin: bool = True):
        self.pending_post_spin_reason = reason
        self.pending_post_spin_force_final = force_final
        self.pending_post_spin_send_cmd3 = send_cmd3_after_spin
        if self.spin_control_mode == "local":
            self.current_state = RobotState.SPINNING_BACK
            self.active_spin_target_angle = self.spin_target_angle
            source_desc = self.prepare_spin_tracking()
            self.set_chassis_mode(ChassisMode.SPIN_BACKWARD, "扫码完成，开始反向自旋回原朝向")
            self.get_logger().info(
                f"扫码阶段结束，开始反向自旋 180 度，使用 {source_desc} 回正后再发布下一个导航点"
            )
            return

        self.current_state = RobotState.SPINNING_BACK
        self.set_chassis_mode(ChassisMode.HOLD, "扫码完成，串口模式下等待电控反向自旋回原朝向")
        self.get_logger().info(
            "扫码阶段结束，已等待电控在命令3后自旋回原朝向；"
            "ROS 不再发布反向自旋底盘模式，收到命令3完成回包后再发布下一个导航点"
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
        self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
        self.set_chassis_mode(
            ChassisMode.HOLD,
            f"扫码完成，等待 {self.post_barcode_shift_delay:.1f} 秒后前往扫码后前进点",
        )
        self.send_serial("3")
        self.stop_serial("3")
        self.get_logger().info(
            f"{reason}，已发送命令3；电控本次不自旋且不回包，"
            f"等待 {self.post_barcode_shift_delay:.1f} 秒后前往扫码后前进点"
        )
        threading.Timer(self.post_barcode_shift_delay, self.resume_shift_return_navigation).start()

    def resume_shift_return_navigation(self):
        with self.lock:
            if self.current_state != RobotState.WAITING_SHIFT_RETURN_DELAY:
                return

            shift_index = self.pending_shift_return_index
            if shift_index is None:
                self.get_logger().warn("扫码后前进点流程缺少索引，无法恢复导航")
                return

            self.navigate_to_shift_point_for_return(shift_index)
            if self.pending_shift_return_action == "retry_next":
                arrival_action = "到达后发送命令4等待电控回包，再前往下一个抓取点"
            else:
                arrival_action = f"到达后发送命令{self.get_final_result_cmd()}等待电控最终回包"
            self.get_logger().info(
                f"扫码后等待结束，基于当前位置前进 {self.dynamic_shift_forward_distance:.3f}m，"
                f"{arrival_action}"
            )

    def start_shift_return_final_mode(self, shift_index: int | None):
        final_cmd = self.get_final_result_cmd()
        self.current_state = RobotState.WAITING_FINAL_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, f"到达扫码后前进点 {shift_index}，等待电控命令{final_cmd}完成")
        self.send_serial(final_cmd)
        self.get_logger().info(
            f"已到达扫码后前进点 {shift_index}，已发送命令{final_cmd}；"
            "等待电控有效回包后发布 /mode=2 并退出节点"
        )

    def start_shift_return_retry_mode(self, shift_index: int | None):
        self.current_state = RobotState.WAITING_SHIFT_RETURN_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, f"到达扫码后继续前进点 {shift_index}，等待电控命令4完成")
        self.send_serial("4")
        self.get_logger().info(
            f"已到达扫码后继续前进点 {shift_index}，已发送命令4；"
            "等待电控有效回包后前往下一个抓取点"
        )

    def start_final_spin(self):
        final_cmd = self.get_final_result_cmd()
        self.reset_spin_tracking()
        self.current_state = RobotState.WAITING_FINAL_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, "到达最终点，等待电控最终流程完成")
        self.send_serial(final_cmd)
        self.get_logger().info(
            f"最终点已到达，已发送命令{final_cmd}，等待电控完成回包后发布 /mode=2"
        )

    def finish_spin(self):
        spin_deg = math.degrees(self.spin_accumulated_yaw)
        if self.current_state == RobotState.SPINNING_FORWARD:
            self.reset_spin_tracking()
            self.set_chassis_mode(ChassisMode.HOLD, "正向自旋完成，驻停等待 TF 对位")
            self.current_state = RobotState.ALIGNING_TF
            self.get_logger().info(f"正向自旋完成，累计角度 {spin_deg:.1f} deg | 状态 -> {self.current_state.name}")
            return

        if self.current_state == RobotState.SPINNING_BACK:
            reason = self.pending_post_spin_reason or "反向自旋完成"
            force_final = self.pending_post_spin_force_final
            send_cmd3_after_spin = self.pending_post_spin_send_cmd3
            self.pending_post_spin_reason = None
            self.pending_post_spin_force_final = False
            self.pending_post_spin_send_cmd3 = True
            self.reset_spin_tracking()
            self.set_chassis_mode(ChassisMode.HOLD, "反向自旋完成，准备恢复导航")
            if send_cmd3_after_spin:
                self.send_serial("3")
                self.get_logger().info(f"反向自旋完成，累计角度 {spin_deg:.1f} deg，发送命令3后恢复导航")
            else:
                self.get_logger().info(f"反向自旋完成，累计角度 {spin_deg:.1f} deg，直接恢复导航")
            self.advance_to_next_target(
                reason,
                force_final=force_final,
                success_completed=True,
            )
            return

    def handle_spin_completed(self):
        if self.current_state == RobotState.SPINNING_FORWARD:
            self.set_chassis_mode(ChassisMode.HOLD, "电控反馈正向自旋完成，驻停等待 TF 对位")
            self.current_state = RobotState.ALIGNING_TF
            self.get_logger().info(f"收到命令3完成回包，正向自旋完成 | 状态 -> {self.current_state.name}")
            return

        if self.current_state == RobotState.SPINNING_BACK:
            reason = self.pending_post_spin_reason or "反向自旋完成"
            force_final = self.pending_post_spin_force_final
            send_cmd3_after_spin = self.pending_post_spin_send_cmd3
            self.pending_post_spin_reason = None
            self.pending_post_spin_force_final = False
            self.pending_post_spin_send_cmd3 = True
            self.set_chassis_mode(ChassisMode.HOLD, "电控反馈反向自旋完成，准备恢复导航")
            if self.spin_control_mode == "local" and send_cmd3_after_spin:
                self.send_serial("3")
                self.get_logger().info("收到命令3完成回包，反向自旋完成，发送命令3后恢复导航")
            elif self.spin_control_mode == "local":
                self.get_logger().info("收到命令3完成回包，反向自旋完成，直接恢复导航")
            else:
                self.stop_serial("3")
                self.get_logger().info("收到命令3完成回包，电控反向自旋完成，准备恢复导航")
            self.advance_to_next_target(
                reason,
                force_final=force_final,
                success_completed=True,
            )
            return

        if self.current_state == RobotState.WAITING_FINAL_RESULT:
            final_cmd = self.get_final_result_cmd()
            self.stop_serial(final_cmd)
            self.set_chassis_mode(ChassisMode.NAVIGATION, "电控反馈最终流程完成，释放底盘导航模式")
            self.current_state = RobotState.FINISHED
            self.external_mode_pub.publish(Int32(data=2))
            self.get_logger().info(
                f"收到命令{final_cmd}完成回包，底盘已切回 NAVIGATION，已发布 /mode=2 | 状态 -> FINISHED"
            )
            threading.Timer(2.0, self.final_exit).start()

    def grab_feedback_timer_callback(self):
        with self.lock:
            self.check_grab_feedback_task()

    def odom_callback(self, msg: Odometry):
        with self.lock:
            position = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.current_odom_pose = Point3d(position.x, position.y, position.z, yaw)

            if self.spin_control_mode != "local":
                return

            if not self.is_spinning_state():
                return

            if self.maybe_fallback_spin_source_to_odom(yaw):
                return

            if self.spin_source_in_use != "odom":
                return

            self.update_spin_with_yaw(yaw, "ODOM")

    def imu_callback(self, msg: Imu):
        with self.lock:
            now = time.monotonic()
            self.last_imu_rx_time = now

            if self.imu_last_sample_time is None:
                self.imu_last_sample_time = now
                self.imu_integrated_yaw = 0.0
                self.imu_start_frame_locked = True
                self.get_logger().info("已锁定启动时首帧 IMU，作为全局角度零点")
            else:
                dt = now - self.imu_last_sample_time
                self.imu_last_sample_time = now
                if 0.0 < dt <= max(0.5, self.spin_imu_timeout * 2.0):
                    self.imu_integrated_yaw += msg.angular_velocity.z * dt

            if self.spin_control_mode != "local":
                return

            if not self.is_spinning_state():
                return

            if self.spin_source_in_use != "imu":
                return

            self.update_spin_with_imu()

    # ==================== TF 对位 ====================
    def tf_check_loop(self):
        with self.lock:
            # 只有在 ALIGNING_TF 状态下才处理 TF
            if self.current_state != RobotState.ALIGNING_TF:
                return
            try:
                now = self.get_clock().now()
                t = self.tf_buffer.lookup_transform(
                    self.reference_frame, self.target_tag_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
                )
                transform_age = (now - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e9
                if transform_age > 0.6:
                    return
                tag_y = t.transform.translation.y
                align_error = tag_y - self.align_target_y
                if abs(align_error) < self.align_tolerance:
                    self.send_serial("2")
                    self.current_state = RobotState.WAITING_BARCODE
                    self.get_logger().info(
                        f"TF 对位成功! tag_y={tag_y:.3f}m target_y={self.align_target_y:.3f}m "
                        f"error={align_error:.3f}m | 状态 -> {self.current_state.name}"
                    )
            except TransformException:
                pass

    def try_handle_detecting_state(self, trigger_source: str, allow_skip_on_false: bool = False):
        if self.current_state != RobotState.DETECTING:
            return

        if self.point_index >= len(self.grab_points):
            return

        if self.object_present:
            if self.grab_count >= self.max_grab_count:
                return
            if not self.depth_sample_received:
                self.get_logger().debug(
                    f"{trigger_source}: 已识别到物体，但尚未收到深度，继续等待 /object_depth"
                )
                return
            self.pre_grab_depth = self.current_depth
            self.send_serial("1")
            self.grab_count += 1
            self.current_state = RobotState.WAITING_GRAB_RESULT
            self.checking_grab_feedback = True
            self.grab_start_time = time.time()
            self.get_logger().info(
                f"检测到物体（触发源: {trigger_source}），发送命令1开始抓取 | "
                f"物体深度={self.pre_grab_depth:.3f}m | 状态 -> {self.current_state.name}"
            )
            return

        if not allow_skip_on_false:
            return

        current_index = self.point_index
        next_index = self.get_next_grab_index(current_index)
        if next_index is not None:
            self.navigate_to_grab_point(next_index)
            self.get_logger().info(
                f"⚠️ 抓取点 {current_index} 未检测到物体，直接前往抓取点 {next_index} | 状态 -> {self.current_state.name}"
            )
        else:
            self.navigate_to_final_point()
            self.get_logger().info(
                f"⚠️ 抓取点 {current_index} 未检测到物体，直接前往最终点 | 状态 -> {self.current_state.name}"
            )

    def object_depth_callback(self, msg: Float32):
        with self.lock:
            self.current_depth = msg.data
            self.depth_sample_received = True
            self.object_depth_rx_count += 1
            if self.object_depth_rx_count == 1:
                self.get_logger().info(f"已收到首帧 /object_depth: {self.current_depth:.3f}m")
            self.try_handle_detecting_state("/object_depth")

    def object_status_callback(self, msg: Bool):
        with self.lock:
            self.object_present = msg.data
            self.object_detected_rx_count += 1
            if self.object_detected_rx_count == 1:
                self.get_logger().info(f"已收到首帧 /object_detected: {self.object_present}")
            self.try_handle_detecting_state("/object_detected", allow_skip_on_false=True)

    def check_grab_feedback_task(self):
        """独立于主状态机的抓取反馈检测逻辑"""
        if not self.checking_grab_feedback:
            return
            
        elapsed = time.time() - self.grab_start_time
        if elapsed < self.grab_check_delay:
            self.get_logger().debug(f"[抓取反馈] 等待中... {elapsed:.1f}s/{self.grab_check_delay}s")
            return

        if self.object_present and abs(self.current_depth - self.pre_grab_depth) < self.grab_depth_tolerance:
            self.get_logger().info(f" 物体还在 | 原深度:{self.pre_grab_depth:.3f} | 现深度:{self.current_depth:.3f} | 差值:{abs(self.current_depth - self.pre_grab_depth):.3f}")
            self.get_logger().warn("⚠️ 判定抓取失败，结束当前抓取流程并跳过当前抓取点")
            if self.active_serial_cmd == "1":
                self.stop_serial("1")
            self.send_serial("5")
            self.grab_count = max(0, self.grab_count - 1)
            self.checking_grab_feedback = False
            self.advance_to_next_target("抓取失败，跳过当前抓取点", success_completed=False)
        else:
            self.checking_grab_feedback = False
            self.navigate_to_shift_point(self.point_index)
            self.get_logger().info(
                f"物体已被拿走（颜色或深度已变化），先前往平移点 {self.point_index}，"
                "到达后执行平移点前置动作流程"
            )

    def barcode_callback(self, msg: String):
        with self.lock:
            barcode_data = msg.data.strip()

            # TF 还在对齐时，如果提前扫到码，也直接接收处理
            if self.current_state not in (RobotState.ALIGNING_TF, RobotState.WAITING_BARCODE):
                return

            if barcode_data == "200":
                if self.special_barcode_triggered:
                    return
                self.special_barcode_triggered = True
                if self.active_serial_cmd == "2":
                    self.stop_serial("2")
                self.schedule_shift_return_after_barcode(
                    "特殊二维码 200，扫码流程完成",
                    force_final=True,
                )
                return

            if barcode_data == "100":
                if self.active_serial_cmd == "2":
                    self.stop_serial("2")
                self.success_count += 1
                if self.success_count >= self.required_success_count:
                    action = "final"
                    reason = (
                        f"普通二维码 100，已完成 {self.success_count}/"
                        f"{self.required_success_count} 个目标"
                    )
                else:
                    action = "retry_next"
                    reason = (
                        f"普通二维码 100，已完成 {self.success_count}/"
                        f"{self.required_success_count} 个目标，继续下一个抓取点"
                    )
                self.schedule_shift_return_after_barcode(
                    reason,
                    action=action,
                )
                return

            if barcode_data == "300":
                if self.active_serial_cmd == "2":
                    self.stop_serial("2")
                self.schedule_shift_return_after_barcode(
                    "二维码 300 判定本次抓取失败",
                    action="retry_next",
                )
                return

    def advance_to_next_target(
        self,
        reason: str,
        force_final: bool = False,
        success_completed: bool = False,
    ):
        self.checking_grab_feedback = False
        self.pending_post_spin_reason = None
        self.pending_post_spin_force_final = False
        self.pending_post_spin_send_cmd3 = True
        self.pending_shift_return_reason = None
        self.pending_shift_return_force_final = False
        self.pending_shift_return_index = None
        self.pending_shift_return_action = "final"
        next_index = self.get_next_grab_index(self.point_index)
        if (
            force_final
            or next_index is None
            or self.grab_count >= self.max_grab_count
        ):
            self.navigate_to_final_point()
            self.get_logger().info(f"{reason}，准备前往最终点 | 状态 -> {self.current_state.name}")
            return

        self.navigate_to_grab_point(next_index)
        self.get_logger().info(f"{reason}，准备前往抓取点 {next_index} | 状态 -> {self.current_state.name}")

    def publish_goal(self, goal_point: Point3d, goal_name: str):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = goal_point.x
        msg.pose.position.y = goal_point.y
        msg.pose.position.z = goal_point.z
        cy = math.cos(goal_point.yaw * 0.5)
        sy = math.sin(goal_point.yaw * 0.5)
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy
        self.goal_pub.publish(msg)
        self.get_logger().info(f"已发布{goal_name}: ({goal_point.x}, {goal_point.y}, {goal_point.z}, {goal_point.yaw:.2f})")

    def nav_callback(self, msg: Bool):
        with self.lock:
            if not msg.data:
                return

            nav_target = self.active_nav_target
            nav_index = self.active_nav_index
            if nav_target == NavTarget.NONE:
                return

            self.active_nav_target = NavTarget.NONE
            self.active_nav_index = None

            if nav_target == NavTarget.SHIFT_POINT:
                self.start_spin_mode(nav_index)
                return

            if nav_target == NavTarget.SHIFT_POINT_RETURN:
                if self.active_serial_cmd == "3":
                    self.stop_serial("3")
                if self.pending_shift_return_action == "retry_next":
                    self.start_shift_return_retry_mode(nav_index)
                else:
                    self.start_shift_return_final_mode(nav_index)
                return

            if nav_target == NavTarget.GRAB_POINT and self.active_serial_cmd == "3":
                self.stop_serial("3")

            if nav_target == NavTarget.FINAL_POINT:
                if self.active_serial_cmd == "3":
                    self.stop_serial("3")
                self.start_final_spin()
                return

            if nav_target != NavTarget.GRAB_POINT:
                return

            self.current_state = RobotState.STABILIZING
            if self.stabilization_time <= 0.0:
                self.get_logger().info(
                    f"到达抓取点 {nav_index}，跳过稳定等待，立即进入抓取处理 | 状态 -> {self.current_state.name}"
                )
                self.finish_stabilization()
                return
            threading.Timer(self.stabilization_time, self.finish_stabilization).start()
            self.get_logger().info(f"到达抓取点 {nav_index}，开始稳定等待 {self.stabilization_time} 秒... | 状态 -> {self.current_state.name}")

    def finish_stabilization(self):
        with self.lock:
            if self.current_state == RobotState.STABILIZING:
                if self.open_loop_grab_enabled:
                    self.start_open_loop_grab()
                    return

                self.current_state = RobotState.DETECTING
                self.get_logger().info(f"抓取点 {self.point_index} 已稳定，现在开始正常检测颜色+深度 | 状态 -> {self.current_state.name}")
                if self.object_detected_rx_count == 0 or self.object_depth_rx_count == 0:
                    self.log_sensor_topic_debug("进入 DETECTING 但视觉消息未齐")
                self.try_handle_detecting_state("进入 DETECTING")

    def start_open_loop_grab(self):
        if self.point_index >= len(self.grab_points):
            return

        if self.grab_count >= self.max_grab_count:
            self.advance_to_next_target("开环抓取次数已满", success_completed=False)
            return

        self.send_serial("1")
        self.grab_count += 1
        self.current_state = RobotState.WAITING_GRAB_RESULT
        self.checking_grab_feedback = False
        self.get_logger().info(
            f"开环抓取: 抓取点 {self.point_index} 已到达，直接发送命令1；"
            f"{self.open_loop_grab_depart_delay:.2f}s 后前往平移点 | 状态 -> {self.current_state.name}"
        )
        threading.Timer(self.open_loop_grab_depart_delay, self.finish_open_loop_grab).start()

    def finish_open_loop_grab(self):
        with self.lock:
            if self.current_state != RobotState.WAITING_GRAB_RESULT:
                return

            self.checking_grab_feedback = False
            self.navigate_to_shift_point(self.point_index)
            self.get_logger().info(
                f"开环抓取: 命令1后延时结束，直接前往平移点 {self.point_index}"
            )

    def final_exit(self):
        self.get_logger().info("最终点流程结束，准备退出节点")
        time.sleep(0.5)
        self.stop_node()

    def send_serial(self, cmd: str):
        if cmd not in SERIAL_COMMANDS:
            self.get_logger().warn(f"未知串口命令: {cmd}")
            return
        with self.lock:
            self.active_serial_cmd = cmd
        self._write_serial_frame(cmd)

    def stop_serial(self, cmd: str | None = None):
        with self.lock:
            if self.active_serial_cmd is None:
                return
            if cmd is not None and self.active_serial_cmd != cmd:
                return
            self.get_logger().info(f"清除活动串口命令记录 {self.active_serial_cmd}")
            self.active_serial_cmd = None

    def _write_serial_frame(self, cmd: str):
        if not (self.serial and self.serial.is_open):
            self.get_logger().warn(f"串口不可用，跳过指令: {cmd}")
            return
        command_cfg = SERIAL_COMMANDS.get(cmd)
        if command_cfg is None:
            self.get_logger().warn(f"未知串口命令: {cmd}")
            return

        frame = build_vision_frame(
            command_cfg["region"],
            command_cfg["mode"],
            command_cfg["need_return"],
            command_cfg["data"],
        )
        try:
            self.serial.reset_output_buffer()
            self.serial.write(frame)
            self.serial.flush()
            self.get_logger().info(f"➡ 串口命令 {cmd} 单次发送成功")
        except Exception as e:
            self.get_logger().error(f"发送失败: {e}")

    def listen_stm32_data(self):
        while self.node_running:
            if self.serial and self.serial.is_open:
                try:
                    recv_data = self.serial.read(self.serial.in_waiting or 1)
                    if recv_data:
                        self.serial_rx_buffer.extend(recv_data)
                        frame_hex = " ".join(f"{byte:02X}" for byte in recv_data)
                        self.get_logger().info(f"⬅ 收到串口原始数据: {frame_hex}")
                        self.process_serial_frames()
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    self.get_logger().error(f"串口读取失败: {e}")
                    time.sleep(0.01)
            else:
                time.sleep(1.0)

    def process_serial_frames(self):
        while True:
            while self.serial_rx_buffer and self.serial_rx_buffer[0] != FRAME_HEADER:
                dropped = self.serial_rx_buffer.pop(0)
                self.get_logger().warn(f"丢弃无效帧头字节: 0x{dropped:02X}")

            if len(self.serial_rx_buffer) < FRAME_LENGTH:
                return

            frame = bytes(self.serial_rx_buffer[:FRAME_LENGTH])
            if frame[-1] != FRAME_TAIL:
                dropped = self.serial_rx_buffer.pop(0)
                self.get_logger().warn(f"帧尾不匹配，丢弃当前帧头字节: 0x{dropped:02X}")
                continue

            del self.serial_rx_buffer[:FRAME_LENGTH]
            parsed = parse_vision_frame(frame)
            if parsed is None:
                self.get_logger().warn(f"收到无法解析的帧: {frame.hex()}")
                continue

            if not parsed["crc_ok"]:
                self.get_logger().warn(f"收到 CRC 校验失败的帧: {parsed['raw']}")
                continue

            self.handle_serial_frame(parsed)

    def handle_serial_frame(self, frame: dict):
        self.get_logger().info(
            "⬅ 收到有效回包: "
            f"region={frame['region']} mode={frame['mode']} "
            f"task_complete={frame['task_complete']} data={frame['data']}"
        )

        with self.lock:
            final_cmd = self.get_final_result_cmd()
            if (
                frame["mode"] == SERIAL_COMMANDS["7"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_SHIFT_PREPARE_RESULT
            ):
                self.stop_serial("7")
                self.set_chassis_mode(ChassisMode.HOLD, "电控反馈命令7完成，准备进入 TF 对位")
                self.current_state = RobotState.ALIGNING_TF
                self.get_logger().info("收到命令7完成回包，开始 TF 对位与扫码 | 状态 -> ALIGNING_TF")
                return

            if (
                frame["mode"] == SERIAL_COMMANDS[final_cmd]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_FINAL_RESULT
            ):
                self.handle_spin_completed()
                return

            if (
                frame["mode"] == SERIAL_COMMANDS["4"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_SHIFT_RETURN_RESULT
            ):
                reason = self.pending_shift_return_reason or "扫码后前进点恢复流程完成"
                is_retry_failure = "300" in reason
                self.stop_serial("4")
                if is_retry_failure:
                    self.grab_count = max(0, self.grab_count - 1)
                self.set_chassis_mode(ChassisMode.HOLD, "电控反馈命令4完成，准备前往下一个抓取点")
                if is_retry_failure:
                    self.get_logger().info(
                        "收到命令4完成回包，二维码300判定本次抓取失败，准备跳过当前抓取点"
                    )
                else:
                    self.get_logger().info(
                        "收到命令4完成回包，本次目标完成但未达到总数，准备前往下一个抓取点"
                    )
                self.advance_to_next_target(
                    reason,
                    success_completed=False,
                )
                return

            active_cmd = self.active_serial_cmd
            if active_cmd is not None:
                active_mode = SERIAL_COMMANDS[active_cmd]["mode"]
                if frame["mode"] == active_mode and frame["task_complete"] == 1:
                    self.stop_serial(active_cmd)

    def stop_node(self):
        self.node_running = False
        self.stop_serial()
        time.sleep(1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
        sys.exit(0)

    def destroy_node(self):
        self.node_running = False
        self.stop_serial()
        super().destroy_node()

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = R2_mode1()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
