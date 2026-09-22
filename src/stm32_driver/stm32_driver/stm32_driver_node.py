#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
import math
import time
from enum import IntEnum

CHASSIS_MODE_TOPIC = "/chassis_mode"
SLOPE_SPEED_BOOST_CHANGE_TS = "2026-06-18T00:19:00+08:00"
LATERAL_LIMIT_CHANGE_TS = "2026-06-18T07:35:00+08:00"


class ChassisMode(IntEnum):
    NAVIGATION = 0
    SPIN_FORWARD = 1
    HOLD = 2
    SPIN_BACKWARD = 3
    CONTACT_APPROACH = 4


class STM32Driver(Node):
    def __init__(self):
        super().__init__('stm32_driver')
        #!/usr/bin/env python3

        self.PITCH_TARGET_1 = math.radians(17)
        self.PITCH_TARGET_2 = math.radians(-17)
        self.PITCH_TOLERANCE = math.radians(1)
        self.current_compensation = 0.0
        self.K_COMPENSATION = 7.0
        self.MAX_COMPENSATION = 2.0

        self.MIN_UPHILL_SPEED = 1.4

        # 加速响应要快 (Attack)
        self.STEP_UP = 0.8
        # 减速响应要适中 (Release)
        self.STEP_DOWN = 0.15
        self.slope_curve_speed_abs = None
        self.slope_curve_last_update_time = time.monotonic()
        self.slope_curve_last_log_time = 0.0

        # 1. 配置串口参数（与STM32一致）
        self.declare_parameter('serial_port', '/dev/ttyUSB1') 
        self.declare_parameter('baudrate', 961200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('spin_angular_speed', 1.5)
        self.declare_parameter('override_frequency', 20.0)
        self.declare_parameter('contact_cmd_vel_topic', '/contact_cmd_vel')
        # 修改记录: 2026-06-18T07:35:00+08:00
        # 修改原因: 二三区过渡 slope_pre 导航时 Nav2 会给出较大 linear.y，实车沿 y 方向贴边。
        # 修改目的: 保留可选的全局横移限幅，但默认关闭，避免影响普通导航。
        # 实现效果: max_linear_y_abs > 0.0 时限制 |vy|；设置为 0.0 时关闭。
        self.declare_parameter('max_linear_y_abs', 0.0)
        # 修改记录: 2026-06-30T00:00:00+08:00
        # 修改原因: 打开上坡补速外部接口，同时支持启动后默认允许补速。
        # 实现效果: 默认订阅 slope_speed_boost_topic；topic=false 时可关闭补速。
        self.declare_parameter('enable_slope_speed_boost', True)
        self.declare_parameter('slope_speed_boost_initial_active', False)
        self.declare_parameter('slope_speed_boost_topic', '/r2/area2_to_area3/slope_speed_boost')
        self.declare_parameter('slope_required_mode_topic', '/mode')
        self.declare_parameter('slope_required_mode_value', 2)
        self.declare_parameter('slope_pitch_abs_threshold_rad', math.radians(1.0))
        self.declare_parameter('slope_linear_x_boost', 0.55)
        self.declare_parameter('slope_curve_accel_mps2', 0.4)
        self.declare_parameter('slope_min_linear_x', 0.20)
        self.declare_parameter('slope_curve_decel_pitch_deg', 16.5)
        self.declare_parameter('slope_curve_decel_mps2', 1.0)
        self.declare_parameter('slope_curve_min_cmd_linear_x', 0.03)
        self.declare_parameter('slope_curve_log_interval_sec', 0.5)
        self.declare_parameter('slope_max_linear_y_abs', 0.16)
        self.declare_parameter('nav_linear_x_decel_limit_enabled', True)
        self.declare_parameter('nav_linear_x_decel_mps2', 0.3)
        self.declare_parameter('nav_linear_x_decel_target_abs_threshold', 0.08)
        self.angle=[0,0,0]
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        self.spin_angular_speed = self.get_parameter('spin_angular_speed').value
        self.contact_cmd_vel_topic = self.get_parameter('contact_cmd_vel_topic').value
        override_frequency = self.get_parameter('override_frequency').value
        self.max_linear_y_abs = abs(float(self.get_parameter('max_linear_y_abs').value))
        self.enable_slope_speed_boost = bool(self.get_parameter('enable_slope_speed_boost').value)
        self.slope_speed_boost_initial_active = bool(self.get_parameter('slope_speed_boost_initial_active').value)
        self.slope_speed_boost_topic = str(self.get_parameter('slope_speed_boost_topic').value)
        self.slope_required_mode_topic = str(self.get_parameter('slope_required_mode_topic').value)
        self.slope_required_mode_value = int(self.get_parameter('slope_required_mode_value').value)
        self.slope_pitch_abs_threshold_rad = abs(float(self.get_parameter('slope_pitch_abs_threshold_rad').value))
        self.slope_linear_x_boost = max(0.0, float(self.get_parameter('slope_linear_x_boost').value))
        self.slope_curve_accel_mps2 = max(0.0, float(self.get_parameter('slope_curve_accel_mps2').value))
        self.slope_min_linear_x = abs(float(self.get_parameter('slope_min_linear_x').value))
        self.slope_curve_decel_pitch_rad = abs(math.radians(float(self.get_parameter('slope_curve_decel_pitch_deg').value)))
        self.slope_curve_decel_mps2 = max(0.0, float(self.get_parameter('slope_curve_decel_mps2').value))
        self.slope_curve_min_cmd_linear_x = abs(float(self.get_parameter('slope_curve_min_cmd_linear_x').value))
        self.slope_curve_log_interval_sec = max(
            0.0,
            float(self.get_parameter('slope_curve_log_interval_sec').value),
        )
        self.slope_max_linear_y_abs = abs(float(self.get_parameter('slope_max_linear_y_abs').value))
        self.nav_linear_x_decel_limit_enabled = bool(
            self.get_parameter('nav_linear_x_decel_limit_enabled').value
        )
        self.nav_linear_x_decel_mps2 = max(
            0.0,
            float(self.get_parameter('nav_linear_x_decel_mps2').value),
        )
        self.nav_linear_x_decel_target_abs_threshold = abs(float(
            self.get_parameter('nav_linear_x_decel_target_abs_threshold').value
        ))
        self.slope_speed_boost_active = self.enable_slope_speed_boost and self.slope_speed_boost_initial_active
        self.slope_required_mode_current = None
        self.nav_limited_linear_x = None
        self.nav_linear_x_limit_last_update_time = time.monotonic()
        override_period = 1.0 / max(1.0, override_frequency)
        self.count=0
        self.current_mode = ChassisMode.NAVIGATION
        self.serial = None
        # 2. 初始化串口
        try:
            self.serial = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
            self.get_logger().info(f'已连接STM32: {serial_port}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            return
        self.odom_sub =self.create_subscription(Odometry,"/Odometry",self.odom_callback,10)
        self.mode_sub = self.create_subscription(Int32, CHASSIS_MODE_TOPIC, self.mode_callback, 10)
        self.slope_required_mode_sub = self.create_subscription(
            Int32,
            self.slope_required_mode_topic,
            self.slope_required_mode_callback,
            10,
        )
        self.slope_speed_boost_sub = None
        if self.enable_slope_speed_boost:
            self.slope_speed_boost_sub = self.create_subscription(
                Bool,
                self.slope_speed_boost_topic,
                self.slope_speed_boost_callback,
                10,
            )
            self.get_logger().info(
                f'[上坡速度补偿] 接口已打开，订阅: {self.slope_speed_boost_topic}, '
                f'初始状态={"开启" if self.slope_speed_boost_active else "关闭"}'
            )
        else:
            self.get_logger().info(
                f'[上坡速度补偿] 接口已关闭，不订阅: {self.slope_speed_boost_topic}'
            )
        self.get_logger().info(
            f'[上坡曲线] start={math.degrees(self.slope_pitch_abs_threshold_rad):.1f}°, '
            f'decel={math.degrees(self.slope_curve_decel_pitch_rad):.1f}°, '
            f'vx_boost={self.slope_linear_x_boost:.2f}m/s, '
            f'accel_rate={self.slope_curve_accel_mps2:.2f}m/s^2, '
            f'decel_rate={self.slope_curve_decel_mps2:.2f}m/s^2, '
            f'slope_vy_limit={self.slope_max_linear_y_abs:.2f}m/s, '
            f'nav_vx_decel_limit={self.nav_linear_x_decel_mps2:.2f}m/s^2, '
            f'nav_vx_decel_target<={self.nav_linear_x_decel_target_abs_threshold:.2f}m/s, '
            f'required_mode={self.slope_required_mode_topic}:{self.slope_required_mode_value}'
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10 # 队列大小10
        )
        self.contact_cmd_vel_sub = self.create_subscription(
            Twist, self.contact_cmd_vel_topic, self.contact_cmd_vel_callback, 10
        )
        self.override_timer = self.create_timer(override_period, self.override_control_callback)
    def odom_callback(self,msg:Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        self.angle=[roll,pitch,yaw]
        
    def slope_speed_boost_callback(self, msg: Bool):
        enabled = bool(msg.data)
        if enabled == self.slope_speed_boost_active:
            return
        self.slope_speed_boost_active = enabled
        self.reset_slope_curve()
        state_text = '开启' if enabled else '关闭'
        self.get_logger().info(
            f'[上坡速度补偿][{SLOPE_SPEED_BOOST_CHANGE_TS}] {state_text}: '
            f'topic={self.slope_speed_boost_topic}'
        )

    def slope_required_mode_callback(self, msg: Int32):
        mode_value = int(msg.data)
        if mode_value == 2 and self.current_mode != ChassisMode.NAVIGATION:
            self.current_mode = ChassisMode.NAVIGATION
            self.send_binary_command(0.0, 0.0, 0.0, 0, 0, log_command=False)
            self.get_logger().info('[外部模式] 收到 /mode=2，强制解除 HOLD/CONTACT_APPROACH，切回 NAVIGATION')
        if mode_value == self.slope_required_mode_current:
            return
        self.slope_required_mode_current = mode_value
        if not self.is_slope_required_mode_active():
            self.reset_slope_curve()
        state_text = '允许补速' if self.is_slope_required_mode_active() else '禁止补速'
        self.get_logger().info(
            f'[上坡速度补偿] /mode={mode_value}, {state_text}; '
            f'要求={self.slope_required_mode_value}'
        )
        
    def quaternion_to_euler(self,x, y, z, w):
   
    # 计算滚转角（roll）
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

    # 计算俯仰角（pitch）
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

    # 计算偏航角（yaw）
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def reset_slope_curve(self):
        self.slope_curve_speed_abs = None
        self.slope_curve_last_update_time = time.monotonic()

    def reset_nav_linear_x_limit(self, linear_x: float = 0.0):
        self.nav_limited_linear_x = linear_x
        self.nav_linear_x_limit_last_update_time = time.monotonic()

    def limit_nav_linear_x_decel(self, target_linear_x: float) -> float:
        if (
            not self.nav_linear_x_decel_limit_enabled
            or self.nav_linear_x_decel_mps2 <= 0.0
        ):
            self.reset_nav_linear_x_limit(target_linear_x)
            return target_linear_x

        now = time.monotonic()
        dt = now - self.nav_linear_x_limit_last_update_time
        self.nav_linear_x_limit_last_update_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        if self.nav_limited_linear_x is None:
            self.nav_limited_linear_x = target_linear_x
            return target_linear_x

        current_linear_x = self.nav_limited_linear_x
        max_delta = self.nav_linear_x_decel_mps2 * dt
        if max_delta <= 0.0:
            return current_linear_x

        if abs(target_linear_x) > self.nav_linear_x_decel_target_abs_threshold:
            self.nav_limited_linear_x = target_linear_x
            return target_linear_x

        same_direction = (
            current_linear_x == 0.0
            or target_linear_x == 0.0
            or math.copysign(1.0, current_linear_x) == math.copysign(1.0, target_linear_x)
        )
        is_decelerating = same_direction and abs(target_linear_x) < abs(current_linear_x)
        is_reversing = (
            current_linear_x != 0.0
            and target_linear_x != 0.0
            and math.copysign(1.0, current_linear_x) != math.copysign(1.0, target_linear_x)
        )

        if is_decelerating or is_reversing:
            next_speed_abs = max(0.0, abs(current_linear_x) - max_delta)
            if is_decelerating:
                next_speed_abs = max(abs(target_linear_x), next_speed_abs)
            limited_linear_x = math.copysign(next_speed_abs, current_linear_x)
        else:
            limited_linear_x = target_linear_x

        self.nav_limited_linear_x = limited_linear_x
        return limited_linear_x

    def update_slope_curve_speed(self, raw_speed_abs: float, target_speed_abs: float, rate_mps2: float) -> float:
        now = time.monotonic()
        dt = now - self.slope_curve_last_update_time
        self.slope_curve_last_update_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        if self.slope_curve_speed_abs is None:
            self.slope_curve_speed_abs = raw_speed_abs

        if rate_mps2 <= 0.0:
            self.slope_curve_speed_abs = target_speed_abs
        else:
            diff = target_speed_abs - self.slope_curve_speed_abs
            max_step = rate_mps2 * dt
            if abs(diff) <= max_step:
                self.slope_curve_speed_abs = target_speed_abs
            else:
                self.slope_curve_speed_abs += math.copysign(max_step, diff)

        # 补速曲线只抬高速度或从抬高速度回落，不压低 Nav2 原始速度。
        self.slope_curve_speed_abs = max(raw_speed_abs, self.slope_curve_speed_abs)
        return self.slope_curve_speed_abs

    def apply_slope_accel_curve(self, raw_linear_x: float, slope_angle_rad: float):
        raw_speed_abs = abs(raw_linear_x)
        if (
            not self.enable_slope_speed_boost
            or not self.slope_speed_boost_active
            or not self.is_slope_required_mode_active()
            or raw_speed_abs < self.slope_curve_min_cmd_linear_x
        ):
            self.reset_slope_curve()
            return raw_linear_x, raw_speed_abs, "off"

        slope_abs_angle_rad = abs(slope_angle_rad)
        if slope_abs_angle_rad < self.slope_pitch_abs_threshold_rad:
            self.reset_slope_curve()
            return raw_linear_x, raw_speed_abs, "below_pitch_threshold"

        if slope_abs_angle_rad < self.slope_curve_decel_pitch_rad:
            phase = "pre_slope_accel"
            target_speed_abs = raw_speed_abs + self.slope_linear_x_boost
            rate_mps2 = self.slope_curve_accel_mps2
        else:
            phase = "on_slope_decel"
            target_speed_abs = raw_speed_abs
            rate_mps2 = self.slope_curve_decel_mps2

        curved_speed_abs = self.update_slope_curve_speed(
            raw_speed_abs,
            target_speed_abs,
            rate_mps2,
        )
        direction = 1.0 if raw_linear_x >= 0.0 else -1.0
        return direction * curved_speed_abs, target_speed_abs, phase

    def should_log_slope_curve(self) -> bool:
        now = time.monotonic()
        if self.slope_curve_log_interval_sec <= 0.0:
            self.slope_curve_last_log_time = now
            return True
        if now - self.slope_curve_last_log_time < self.slope_curve_log_interval_sec:
            return False
        self.slope_curve_last_log_time = now
        return True

    def is_slope_required_mode_active(self) -> bool:
        return self.slope_required_mode_current == self.slope_required_mode_value

    def is_slope_y_boost_active(self, slope_angle_rad: float) -> bool:
        return (
            self.enable_slope_speed_boost
            and self.slope_speed_boost_active
            and self.is_slope_required_mode_active()
            and abs(slope_angle_rad) >= self.slope_pitch_abs_threshold_rad
        )

    def mode_callback(self, msg: Int32):
        try:
            new_mode = ChassisMode(msg.data)
        except ValueError:
            self.get_logger().warn(f'收到未知底盘模式: {msg.data}')
            return

        if new_mode == self.current_mode:
            return

        self.current_mode = new_mode
        self.get_logger().info(f'底盘模式切换为: {new_mode.name}')
        if new_mode != ChassisMode.NAVIGATION:
            self.reset_slope_curve()
            self.reset_nav_linear_x_limit(0.0)

        if new_mode == ChassisMode.SPIN_FORWARD:
            self.send_binary_command(0.0, 0.0, 0, 0, 0, log_command=False)
        elif new_mode == ChassisMode.SPIN_BACKWARD:
            self.send_binary_command(0.0, 0.0, 0, 0, 0, log_command=False)
        elif new_mode == ChassisMode.HOLD:
            self.send_binary_command(0.0, 0.0, 0.0, 0, 0, log_command=False)
        elif new_mode == ChassisMode.CONTACT_APPROACH:
            self.send_binary_command(0.0, 0.0, 0.0, 0, 0, log_command=False)
        else:
            self.send_binary_command(0.0, 0.0, 0.0, 0, 0, log_command=False)

    def override_control_callback(self):
        if self.current_mode == ChassisMode.SPIN_FORWARD:
            self.send_binary_command(0.0, 0.0, 0, 0, 0, log_command=False)
        elif self.current_mode == ChassisMode.SPIN_BACKWARD:
            self.send_binary_command(0.0, 0.0, 0, 0, 0, log_command=False)
        elif self.current_mode == ChassisMode.HOLD:
            self.send_binary_command(0.0, 0.0, 0.0, 0, 0, log_command=False)
        
    def contact_cmd_vel_callback(self, msg: Twist):
        if self.current_mode != ChassisMode.CONTACT_APPROACH:
            return

        linear_x = msg.linear.x
        self.send_binary_command(linear_x, 0.0, 0.0, 0, 0)
        self.get_logger().info(f'contact_vx:{linear_x:.2f},vy:0.00,vz:0.0')

    def cmd_vel_callback(self, msg:Twist):
        if self.current_mode != ChassisMode.NAVIGATION:
            return
        # 限制速度范围（避免STM32电机过载）
        raw_linear_x = msg.linear.x
        raw_linear_y = msg.linear.y
        linear_x = raw_linear_x
        target_compensation = 0.0
        # MID360 当前为 pitch 方向倒装，/Odometry 的 pitch 与车体上坡方向相反。
        # 这里取负后再取绝对值判定坡度，是否处于上坡段由外部补速开关限制。
        slope_angle_rad = -self.angle[1]
        linear_x, slope_curve_target_x, slope_curve_phase = self.apply_slope_accel_curve(
            raw_linear_x,
            slope_angle_rad,
        )
        linear_x = self.limit_nav_linear_x_decel(linear_x)
        raw_angular_z = max(min(msg.angular.z, 3.14), -3.14) # 角速度范围：-π~π rad/s
        angular_z = 0.0
        # 上坡段保留 vy，但单独限幅，避免横移过大干扰短坡上的前向冲坡。
        linear_y = raw_linear_y
        if self.is_slope_y_boost_active(slope_angle_rad):
            linear_y = max(min(linear_y, self.slope_max_linear_y_abs), -self.slope_max_linear_y_abs)
        if self.max_linear_y_abs > 0.0:
            linear_y = max(min(linear_y, self.max_linear_y_abs), -self.max_linear_y_abs)
        if (
            self.enable_slope_speed_boost
            and self.slope_speed_boost_active
            and self.is_slope_required_mode_active()
            and slope_curve_phase in ("pre_slope_accel", "on_slope_decel")
            and self.should_log_slope_curve()
        ):
            phase_text = "坡前加速" if slope_curve_phase == "pre_slope_accel" else "17度后减速"
            self.get_logger().info(
                f'上坡曲线({phase_text}): 角度={math.degrees(slope_angle_rad):.1f}°, '
                f'目标={slope_curve_target_x:.2f}m/s, '
                f'vx:{raw_linear_x:.2f}->{linear_x:.2f}, '
                f'vy:{raw_linear_y:.2f}->{linear_y:.2f}'
            )
 
       # --- 合成指令 ---
        mode=0
        self.send_binary_command(linear_x, linear_y, 0, 0, mode)
        if linear_y != raw_linear_y:
            self.get_logger().info(
                f'vx:{linear_x:.2f},vy:{linear_y:.2f}(raw:{raw_linear_y:.2f},limit:{self.max_linear_y_abs:.2f}),vz:{angular_z:.2f}(raw:{raw_angular_z:.2f})'
            )
        else:
            self.get_logger().info(f'vx:{linear_x:.2f},vy:{linear_y:.2f},vz:{angular_z:.2f}(raw:{raw_angular_z:.2f})')

    def send_binary_command(self, vx, vy, vw, flag, mode, log_command=True):
        """发送二进制控制协议到STM32"""
        if self.serial is None or not self.serial.is_open:
            return
            
        try:
            # 下层暂不接收角速度，所有发往 STM32 的 Vw 统一清零。
            # vw = 0.0
            # 1. 浮点数转换为整型 (m/s -> mm/s, rad/s -> mrad/s)
            vx_int = int(vx * 1000)
            vy_int = int(vy * 1000)
            vw_int = int(vw * 10)
            flag = int(flag)
            mode = int(mode)
            reserved = 0  # 保留字，当前固定填 0

            if not 0 <= flag <= 0xFF:
                raise ValueError(f'flag 超出 1 字节范围: {flag}')
            if not 0 <= mode <= 0xFF:
                raise ValueError(f'mode 超出 1 字节范围: {mode}')
            
            # 2. 打包前 10 个字节：SOF + Vx + Vy + Vw + Flag + Reserved + Mode
            # '<' 表示小端模式(STM32常用)
            frame_head = struct.pack('<B h h h B B B', 
                                     0xA5,       # SOF (帧头，1字节)
                                     vx_int,     # Vx (2字节)
                                     vy_int,     # Vy (2字节)
                                     vw_int,     # Vw (2字节)
                                     flag,       # Flag (1字节)
                                     reserved,   # Reserved (1字节)
                                     mode)       # Mode (1字节)
            
            # 3. 提取纯数据部分进行 CRC 校验
            # 使用 [1:] 跳过第0个字节（即 0xA5 帧头），只对后面的 9 个字节的数据进行校验
            data_to_crc = frame_head[1:]
            crc = self.calc_crc8(data_to_crc)
            
            # 4. 拼接完整数据帧 (追加 CRC 和 0xFF 帧尾)
            full_frame = frame_head + struct.pack('<B B', crc, 0xFF)
            
            # 5. 通过串口发送字节流
            if log_command:
                self.get_logger().info(f'命令发送成功: flag={flag}, mode={mode}')
            
            self.serial.write(full_frame)
            
        except Exception as e:
            self.get_logger().error(f'二进制命令发送失败: {e}')
      # ================== CRC-8 计算函数 ==================
    def calc_crc8(self, data: bytes, poly: int = 0x07, init_value: int = 0x00) -> int:
        """
        计算标准 CRC-8
        :param data: 需要计算的字节数据
        :param poly: 多项式 (标准通常为 0x07)
        :param init_value: 初始值
        :return: 计算出的 CRC-8 校验值 (0-255)
        """
        crc = init_value
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFF  # 确保始终是8位
        return crc
def main():
    rclpy.init() 
    node = STM32Driver() # 创建节点
    try:
        rclpy.spin(node) # 自旋节点（持续运行）
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            node.serial.close() # 关闭串口
        node.destroy_node() # 销毁节点
        rclpy.shutdown() # 关闭ROS2上下文

if __name__ == '__main__':
    main()
        
