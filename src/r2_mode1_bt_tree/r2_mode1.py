#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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


class RobotState(Enum):
    INIT = auto()               # 初始化状态
    NAVIGATING = auto()         # 导航移动中
    STABILIZING = auto()        # 到达目标点，等待稳定
    DETECTING = auto()          # 检测物体（颜色+深度）
    WAITING_GRAB_RESULT = auto()  # 已发送1抓取，等待抓取结果判定
    WAITING_SHIFT_PREPARE_RESULT = auto()  # 首次到达平移点后已发送7，等待电控完成前置动作
    ALIGNING_TF = auto()        # 命令7完成后等待 TF 对位
    WAITING_BARCODE = auto()    # 已发送2，等待二维码(100/200)
    WAITING_POST_BARCODE_CMD8_RESULT = auto()  # 扫码后已发送8，等待回包后发送3
    WAITING_POST_BARCODE_CMD3_RESULT = auto()  # 已发送3，等待回包后开始15s延时
    WAITING_SHIFT_RETURN_DELAY = auto()  # 命令3回包后等待延时，再发送9
    WAITING_POST_BARCODE_CMD9_RESULT = auto()  # 延时后已发送9，等待回包后前往前进点
    WAITING_SHIFT_RETURN_RESULT = auto()  # 到达扫码后前进点后已发送4，等待电控回正完成回包
    WAITING_FINAL_RESULT = auto()  # 到达最终点后已发送6，等待电控完成回包
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
    "3": {"region": 0x01, "mode": 0x03, "need_return": 1, "data": 0},
    "4": {"region": 0x01, "mode": 0x04, "need_return": 1, "data": 0},
    "5": {"region": 0x01, "mode": 0x05, "need_return": 0, "data": 0},
    "6": {"region": 0x01, "mode": 0x06, "need_return": 1, "data": 0},
    "7": {"region": 0x01, "mode": 0x07, "need_return": 1, "data": 0},
    "8": {"region": 0x01, "mode": 0x08, "need_return": 1, "data": 0},
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
        self.declare_parameter('stabilization_time', 0.5)
        self.declare_parameter('grab_goal_checker_id', 'grab_goal_checker')
        self.declare_parameter('shift_goal_checker_id', 'shift_goal_checker')
        self.declare_parameter('final_goal_checker_id', 'shift_goal_checker')
        self.declare_parameter('goal_checker_selector_topic', '/goal_checker_selector')
        self.declare_parameter('post_barcode_shift_delay', 15.0)
        self.declare_parameter('dynamic_shift_forward_distance', 0.3)
        self.declare_parameter('preferred_start_grab_index', 0)
        self.declare_parameter('finish_after_first_success', True)
        self.target_tag_frame = self.get_parameter('target_tag_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.align_target_y = self.get_parameter('align_target_y').value
        self.align_tolerance = self.get_parameter('align_tolerance').value
        self.stabilization_time = self.get_parameter('stabilization_time').value
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
        self.finish_after_first_success = bool(
            self.get_parameter('finish_after_first_success').value
        )

        # 串口初始化
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
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
            )
            self.get_logger().info(f'串口连接成功: {serial_port} | 波特率: {baudrate}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {str(e)}')

        self.grab_points = [
            Point3d(-0.85, 0.75, 0.0, 0.0),
            Point3d(-0.85, 0.95, 0.0, 0.0),
            Point3d(-0.85, 1.15, 0.0, 0.0),
            Point3d(-0.85, 1.35, 0.0, 0.0),
            Point3d(-0.85, 1.55, 0.0, 0.0),
            Point3d(-0.85, 1.75, 0.0, 0.0),
        ]
        self.shift_points = [
            Point3d(-0.60, 0.75, 0.0, 0.0),
            Point3d(-0.60, 0.95, 0.0, 0.0),
            Point3d(-0.60, 1.15, 0.0, 0.0),
            Point3d(-0.60, 1.35, 0.0, 0.0),
            Point3d(-0.60, 1.55, 0.0, 0.0),
            Point3d(-0.60, 1.75, 0.0, 0.0),
        ]
        self.final_point = Point3d(1.85, 1.4, 0.0, 0.0)
        self.point_index = 0
        self.grab_count = 0
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
        self.grab_check_delay = 1.25
        self.grab_depth_tolerance = 0.05
        self.pending_shift_return_reason = None
        self.pending_shift_return_force_final = False
        self.pending_shift_return_index = None
        self.current_odom_pose = None

        self.lock = threading.RLock()

        # ROS 接口
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.tf_check_loop)
        self.nav_sub = self.create_subscription(Bool, "/agent/arrival_status", self.nav_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, "/Odometry", self.odom_callback, qos)
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
        self.get_logger().info("串口命令发送模式: 单次下发（不重复发送）")
        self.get_logger().info(
            "抓取点策略: "
            f"preferred_start_grab_index={self.preferred_start_grab_index} | "
            f"finish_after_first_success={self.finish_after_first_success}"
        )
        self.start_initial_navigation()

    def start_initial_navigation(self):
        with self.lock:
            if self.initial_navigation_started:
                return
            self.initial_navigation_started = True
            start_index = self.resolve_start_grab_index()
            self.navigate_to_grab_point(start_index)
        self.get_logger().info(f"启动即发布第一个导航点，目标抓取点 {start_index}")

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
        self.active_nav_target = NavTarget.SHIFT_POINT
        self.active_nav_index = index
        self.current_state = RobotState.NAVIGATING
        self.select_goal_checker_for_target(self.shift_goal_checker_id, f"平移点 {index}")
        self.set_chassis_mode(ChassisMode.NAVIGATION, f"抓取成功，前往平移点 {index}")
        self.publish_goal(self.shift_points[index], f"平移点 {index}")

    def navigate_to_shift_point_for_return(self, index: int):
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

    def start_shift_prepare_mode(self, shift_index: int | None):
        self.current_state = RobotState.WAITING_SHIFT_PREPARE_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, f"到达平移点 {shift_index}，等待电控命令7流程完成")
        self.send_serial("7")
        self.get_logger().info(
            f"平移点 {shift_index} 已到达，已发送命令7；"
            "底盘保持 HOLD，等待电控回包7 后再进入 TF 对位与扫码流程"
        )

    def schedule_shift_return_after_barcode(self, reason: str, force_final: bool = False):
        self.pending_shift_return_reason = reason
        self.pending_shift_return_force_final = force_final
        self.pending_shift_return_index = self.point_index
        self.current_state = RobotState.WAITING_POST_BARCODE_CMD8_RESULT
        self.set_chassis_mode(
            ChassisMode.HOLD,
            "扫码完成，开始执行电控命令8->3->延时->9流程",
        )
        self.send_serial("8")
        self.get_logger().info(
            f"{reason}，已发送命令8，等待回包后继续发送命令3"
        )

    def resume_shift_return_navigation(self):
        with self.lock:
            if self.current_state != RobotState.WAITING_SHIFT_RETURN_DELAY:
                return

            self.current_state = RobotState.WAITING_POST_BARCODE_CMD9_RESULT
            self.send_serial("9")
            self.get_logger().info(
                f"命令3回包后的 {self.post_barcode_shift_delay:.1f}s 延时结束，"
                "已发送命令9，等待回包后前往扫码后前进点"
            )

    def start_shift_return_mode(self, shift_index: int | None):
        self.current_state = RobotState.WAITING_SHIFT_RETURN_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, f"到达扫码后前进点 {shift_index}，等待电控命令4回正")
        self.send_serial("4")
        self.get_logger().info(
            f"已到达扫码后前进点 {shift_index}，已发送命令4；"
            "等待电控有效回包后恢复导航到下一个目标"
        )

    def start_final_mode(self):
        self.current_state = RobotState.WAITING_FINAL_RESULT
        self.set_chassis_mode(ChassisMode.HOLD, "到达最终点，等待电控最终流程完成")
        self.send_serial("6")
        self.get_logger().info("最终点已到达，已发送命令6，等待电控完成回包后发布 /mode=2")

    def grab_feedback_timer_callback(self):
        with self.lock:
            self.check_grab_feedback_task()

    def odom_callback(self, msg: Odometry):
        with self.lock:
            position = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.current_odom_pose = Point3d(position.x, position.y, position.z, yaw)

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
                self.schedule_shift_return_after_barcode("普通二维码 100，扫码流程完成")

    def advance_to_next_target(
        self,
        reason: str,
        force_final: bool = False,
        success_completed: bool = False,
    ):
        self.checking_grab_feedback = False
        self.pending_shift_return_reason = None
        self.pending_shift_return_force_final = False
        self.pending_shift_return_index = None
        should_finish_after_success = (
            success_completed and self.finish_after_first_success
        )
        next_index = self.get_next_grab_index(self.point_index)
        if (
            force_final
            or should_finish_after_success
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
                self.start_shift_prepare_mode(nav_index)
                return

            if nav_target == NavTarget.SHIFT_POINT_RETURN:
                if self.active_serial_cmd == "3":
                    self.stop_serial("3")
                self.start_shift_return_mode(nav_index)
                return

            if nav_target == NavTarget.GRAB_POINT and self.active_serial_cmd == "3":
                self.stop_serial("3")

            if nav_target == NavTarget.FINAL_POINT:
                if self.active_serial_cmd == "3":
                    self.stop_serial("3")
                self.start_final_mode()
                return

            if nav_target != NavTarget.GRAB_POINT:
                return

            self.current_state = RobotState.STABILIZING
            threading.Timer(self.stabilization_time, self.finish_stabilization).start()
            self.get_logger().info(f"到达抓取点 {nav_index}，开始稳定等待 {self.stabilization_time} 秒... | 状态 -> {self.current_state.name}")

    def finish_stabilization(self):
        with self.lock:
            if self.current_state == RobotState.STABILIZING:
                self.current_state = RobotState.DETECTING
                self.get_logger().info(f"抓取点 {self.point_index} 已稳定，现在开始正常检测颜色+深度 | 状态 -> {self.current_state.name}")
                self.try_handle_detecting_state("进入 DETECTING")

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
                frame["mode"] == SERIAL_COMMANDS["8"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_POST_BARCODE_CMD8_RESULT
            ):
                self.stop_serial("8")
                self.current_state = RobotState.WAITING_POST_BARCODE_CMD3_RESULT
                self.send_serial("3")
                self.get_logger().info("收到命令8完成回包，已发送命令3，继续等待回包")
                return

            if (
                frame["mode"] == SERIAL_COMMANDS["3"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_POST_BARCODE_CMD3_RESULT
            ):
                self.stop_serial("3")
                self.current_state = RobotState.WAITING_SHIFT_RETURN_DELAY
                self.set_chassis_mode(
                    ChassisMode.HOLD,
                    f"命令3完成，等待 {self.post_barcode_shift_delay:.1f} 秒后发送命令9",
                )
                self.get_logger().info(
                    f"收到命令3完成回包，开始 {self.post_barcode_shift_delay:.1f}s 延时"
                )
                threading.Timer(self.post_barcode_shift_delay, self.resume_shift_return_navigation).start()
                return

            if (
                frame["mode"] == SERIAL_COMMANDS["9"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_POST_BARCODE_CMD9_RESULT
            ):
                self.stop_serial("9")
                shift_index = self.pending_shift_return_index
                if shift_index is None:
                    self.get_logger().warn("扫码后前进点流程缺少索引，无法恢复导航")
                    return
                self.get_logger().info(
                    "收到命令9完成回包，开始前往扫码后前进点"
                )
                self.navigate_to_shift_point_for_return(shift_index)
                return

            if (
                frame["mode"] == SERIAL_COMMANDS["4"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_SHIFT_RETURN_RESULT
            ):
                reason = self.pending_shift_return_reason or "电控命令4流程完成"
                force_final = self.pending_shift_return_force_final
                self.stop_serial("4")
                self.set_chassis_mode(ChassisMode.HOLD, "电控反馈命令4完成，准备恢复导航")
                self.get_logger().info("收到命令4完成回包，电控已回正，准备前往下一个目标")
                self.advance_to_next_target(
                    reason,
                    force_final=force_final,
                    success_completed=True,
                )
                return

            if (
                frame["mode"] == SERIAL_COMMANDS["6"]["mode"]
                and frame["task_complete"] == 1
                and self.current_state == RobotState.WAITING_FINAL_RESULT
            ):
                self.stop_serial("6")
                self.set_chassis_mode(ChassisMode.HOLD, "电控反馈最终流程完成，底盘驻停")
                self.current_state = RobotState.FINISHED
                self.external_mode_pub.publish(Int32(data=2))
                self.get_logger().info("收到命令6完成回包，已发布 /mode=2 | 状态 -> FINISHED")
                threading.Timer(2.0, self.final_exit).start()
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
