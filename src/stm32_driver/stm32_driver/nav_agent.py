#!/usr/bin/env python3
from collections import deque
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

CHASSIS_MODE_TOPIC = "/chassis_mode"
PRE_GRAB_REACHED_TOPIC = "/agent/pre_grab_reached"
CHASSIS_MODE_NAVIGATION = 0
CHASSIS_MODE_HOLD = 2
CHASSIS_MODE_CONTACT_APPROACH = 4
CONTACT_LOGIC_MODE_VALUE = 1
AREA3_GOAL_CHECKER_MODE_VALUE = 3

class NavAgent(Node):
    def __init__(self):
        super().__init__('nav_agent_node')
        self.declare_parameter('controller_selector_topic', '/controller_selector')
        self.declare_parameter('goal_checker_selector_topic', '/goal_checker_selector')
        self.declare_parameter('odom_topic', '/nav2_odom')
        self.declare_parameter('short_distance_controller', 'TEB_Short')
        self.declare_parameter('long_distance_controller', 'MPPI_Long')
        self.declare_parameter('controller_switch_distance', 0.90)
        self.declare_parameter('area1_to_area2_controller_switch_distance', 0.20)
        self.declare_parameter('retry_controller_switch_distance', 0.10)
        self.declare_parameter('controller_switch_hysteresis', 0.15)
        self.declare_parameter('duplicate_goal_distance_epsilon', 0.005)
        self.declare_parameter('duplicate_goal_time_window', 4.0)
        self.declare_parameter('grab_goal_checker_name', 'grab_goal_checker')
        self.declare_parameter('area3_goal_checker_name', 'area3_goal_checker')
        self.declare_parameter('grab_one_sided_arrival_enabled', False)
        self.declare_parameter('grab_one_sided_axis', 'x')
        self.declare_parameter('grab_one_sided_side_mode', 'closer_to_zero')
        self.declare_parameter('grab_one_sided_side_epsilon', 0.001)
        self.declare_parameter('grab_one_sided_correction_margin', 0.005)
        self.declare_parameter('grab_one_sided_max_corrections', 2)
        self.declare_parameter('grab_contact_arrival_enabled', False)
        self.declare_parameter('grab_contact_window_sec', 0.7)
        self.declare_parameter('grab_contact_position_epsilon', 0.01)
        self.declare_parameter('grab_contact_max_remaining', 0.04)
        self.declare_parameter('grab_contact_min_samples', 3)
        self.declare_parameter('grab_contact_require_preferred_side', True)
        self.declare_parameter('grab_contact_window_slack_sec', 0.05)
        self.declare_parameter('grab_contact_debug_enabled', True)
        self.declare_parameter('grab_contact_debug_interval_sec', 0.5)
        self.declare_parameter('grab_contact_debug_distance_margin', 0.10)
        self.declare_parameter('low_speed_arrival_enabled', False)
        self.declare_parameter('low_speed_arrival_vx_threshold', 0.05)
        self.declare_parameter('low_speed_arrival_vy_threshold', 0.05)
        self.declare_parameter('low_speed_arrival_max_remaining', 0.08)
        self.declare_parameter('contact_approach_enabled', True)
        self.declare_parameter('contact_approach_cmd_vel_topic', '/contact_cmd_vel')
        self.declare_parameter('contact_approach_speed_x', -0.50)
        self.declare_parameter('contact_approach_timeout_sec', 4.0)
        self.declare_parameter('contact_approach_max_distance', 0.70)
        self.declare_parameter('contact_approach_min_distance', 0.03)
        self.declare_parameter('contact_approach_command_period_sec', 0.05)
        self.declare_parameter('contact_approach_require_preferred_side', False)
        self.declare_parameter('contact_approach_result_delay_sec', 0.3)
        self.declare_parameter('retry_abort_hold_topic', '/agent/retry_abort_hold')
        self.declare_parameter('speed_up_topic', '/speed_up')
        self.declare_parameter(
            'slope_speed_boost_topic',
            '/r2/area2_to_area3/slope_speed_boost',
        )

        self.controller_selector_topic = self.get_parameter(
            'controller_selector_topic'
        ).value
        self.goal_checker_selector_topic = self.get_parameter(
            'goal_checker_selector_topic'
        ).value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.short_distance_controller = self.get_parameter(
            'short_distance_controller'
        ).value
        self.long_distance_controller = self.get_parameter(
            'long_distance_controller'
        ).value
        self.controller_switch_distance = float(
            self.get_parameter('controller_switch_distance').value
        )
        self.area1_to_area2_controller_switch_distance = float(
            self.get_parameter('area1_to_area2_controller_switch_distance').value
        )
        self.retry_controller_switch_distance = float(
            self.get_parameter('retry_controller_switch_distance').value
        )
        self.controller_switch_hysteresis = float(
            self.get_parameter('controller_switch_hysteresis').value
        )
        self.duplicate_goal_distance_epsilon = float(
            self.get_parameter('duplicate_goal_distance_epsilon').value
        )
        self.duplicate_goal_time_window = float(
            self.get_parameter('duplicate_goal_time_window').value
        )
        self.grab_goal_checker_name = str(
            self.get_parameter('grab_goal_checker_name').value
        ).strip()
        self.area3_goal_checker_name = str(
            self.get_parameter('area3_goal_checker_name').value
        ).strip()
        self.grab_one_sided_arrival_enabled = bool(
            self.get_parameter('grab_one_sided_arrival_enabled').value
        )
        self.grab_one_sided_axis = str(
            self.get_parameter('grab_one_sided_axis').value
        ).strip().lower()
        if self.grab_one_sided_axis not in ('x', 'y'):
            self.get_logger().warn(
                f'未知 grab_one_sided_axis={self.grab_one_sided_axis}，回退到 x'
            )
            self.grab_one_sided_axis = 'x'
        self.grab_one_sided_side_mode = str(
            self.get_parameter('grab_one_sided_side_mode').value
        ).strip().lower()
        if self.grab_one_sided_side_mode not in (
            'closer_to_zero',
            'further_from_zero',
            'less_than_target',
            'greater_than_target',
        ):
            self.get_logger().warn(
                f'未知 grab_one_sided_side_mode={self.grab_one_sided_side_mode}，回退到 closer_to_zero'
            )
            self.grab_one_sided_side_mode = 'closer_to_zero'
        self.grab_one_sided_side_epsilon = float(
            self.get_parameter('grab_one_sided_side_epsilon').value
        )
        self.grab_one_sided_correction_margin = max(
            0.0, float(self.get_parameter('grab_one_sided_correction_margin').value)
        )
        self.grab_one_sided_max_corrections = max(
            0, int(self.get_parameter('grab_one_sided_max_corrections').value)
        )
        self.grab_contact_arrival_enabled = bool(
            self.get_parameter('grab_contact_arrival_enabled').value
        )
        self.grab_contact_window_sec = max(
            0.2, float(self.get_parameter('grab_contact_window_sec').value)
        )
        self.grab_contact_position_epsilon = max(
            0.0, float(self.get_parameter('grab_contact_position_epsilon').value)
        )
        self.grab_contact_max_remaining = max(
            0.0, float(self.get_parameter('grab_contact_max_remaining').value)
        )
        self.grab_contact_min_samples = max(
            2, int(self.get_parameter('grab_contact_min_samples').value)
        )
        self.grab_contact_require_preferred_side = bool(
            self.get_parameter('grab_contact_require_preferred_side').value
        )
        self.grab_contact_window_slack_sec = max(
            0.0, float(self.get_parameter('grab_contact_window_slack_sec').value)
        )
        self.grab_contact_debug_enabled = bool(
            self.get_parameter('grab_contact_debug_enabled').value
        )
        self.grab_contact_debug_interval_sec = max(
            0.1, float(self.get_parameter('grab_contact_debug_interval_sec').value)
        )
        self.grab_contact_debug_distance_margin = max(
            0.0, float(self.get_parameter('grab_contact_debug_distance_margin').value)
        )
        self.low_speed_arrival_enabled = bool(
            self.get_parameter('low_speed_arrival_enabled').value
        )
        self.low_speed_arrival_vx_threshold = abs(float(
            self.get_parameter('low_speed_arrival_vx_threshold').value
        ))
        self.low_speed_arrival_vy_threshold = abs(float(
            self.get_parameter('low_speed_arrival_vy_threshold').value
        ))
        self.low_speed_arrival_max_remaining = max(
            0.0, float(self.get_parameter('low_speed_arrival_max_remaining').value)
        )
        self.contact_approach_enabled = bool(
            self.get_parameter('contact_approach_enabled').value
        )
        self.contact_approach_cmd_vel_topic = str(
            self.get_parameter('contact_approach_cmd_vel_topic').value
        ).strip()
        if not self.contact_approach_cmd_vel_topic:
            self.contact_approach_cmd_vel_topic = '/cmd_vel'
        self.contact_approach_speed_x = float(
            self.get_parameter('contact_approach_speed_x').value
        )
        if self.contact_approach_speed_x > 0.0:
            self.get_logger().warn(
                f'contact_approach_speed_x={self.contact_approach_speed_x:.3f} 为正，'
                '固定顶靠只允许后退，自动改为负值'
            )
            self.contact_approach_speed_x = -self.contact_approach_speed_x
        self.contact_approach_timeout_sec = max(
            0.1, float(self.get_parameter('contact_approach_timeout_sec').value)
        )
        self.contact_approach_max_distance = max(
            0.0, float(self.get_parameter('contact_approach_max_distance').value)
        )
        self.contact_approach_min_distance = max(
            0.0, float(self.get_parameter('contact_approach_min_distance').value)
        )
        self.contact_approach_command_period_sec = max(
            0.02, float(self.get_parameter('contact_approach_command_period_sec').value)
        )
        self.contact_approach_require_preferred_side = bool(
            self.get_parameter('contact_approach_require_preferred_side').value
        )
        self.contact_approach_result_delay_sec = max(
            0.0, float(self.get_parameter('contact_approach_result_delay_sec').value)
        )
        self.retry_abort_hold_topic = str(
            self.get_parameter('retry_abort_hold_topic').value
        ).strip()
        if not self.retry_abort_hold_topic:
            self.retry_abort_hold_topic = '/agent/retry_abort_hold'
        self.speed_up_topic = str(self.get_parameter('speed_up_topic').value).strip() or '/speed_up'
        self.slope_speed_boost_topic = str(
            self.get_parameter('slope_speed_boost_topic').value
        ).strip() or '/r2/area2_to_area3/slope_speed_boost'
        self.slope_speed_boost_active = False

        self.current_x = None
        self.current_y = None
        self.current_cmd_vx = 0.0
        self.current_cmd_vy = 0.0
        self.active_goal = None
        self.reference_goal = None
        self.active_controller = None
        self.selected_goal_checker = self.grab_goal_checker_name
        self.active_goal_checker = None
        self.grab_correction_attempts = 0
        self.last_goal_signature = None
        self.last_goal_received_time = None
        self.pending_retry_goal_signature = None
        self.retry_controller_switch_distance_override = None
        self.area1_to_area2_goal_pending = False
        self.area1_to_area2_goal_active = False
        self.grab_contact_samples = deque()
        self.contact_arrival_pending_success = False
        self.contact_arrival_pending_reason = ''
        self.last_grab_contact_debug_time = None
        self.contact_approach_active = False
        self.contact_approach_start_time = None
        self.contact_approach_start_x = None
        self.contact_approach_start_y = None
        self.last_contact_approach_cmd_time = None
        self.contact_approach_result_timer = None
        self.external_mode_value = None
        self.retry_hold_active = False

        #初始化
        self.navigator = BasicNavigator()

        # Nav2 启动可能需要数秒。提前建立目标订阅，确保等待期间发布的
        # /goal_pose 会进入订阅队列，并在节点初始化完成后得到处理。
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.retry_abort_hold_sub = self.create_subscription(
            Bool,
            self.retry_abort_hold_topic,
            self.retry_abort_hold_callback,
            10,
        )
        
        self.get_logger().info('正在等待 Nav2 启动 ...')
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        selector_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        self.goal_checker_sub = self.create_subscription(
            String,
            self.goal_checker_selector_topic,
            self.goal_checker_callback,
            selector_qos,
        )
        self.external_mode_sub = self.create_subscription(
            Int32,
            '/mode',
            self.external_mode_callback,
            10,
        )
        self.speed_up_sub = self.create_subscription(
            Bool,
            self.speed_up_topic,
            self.speed_up_callback,
            selector_qos,
        )
        self.feedback_pub = self.create_publisher(String, '/agent/feedback_log', 10)
        self.controller_selector_pub = self.create_publisher(
            String, self.controller_selector_topic, selector_qos
        )
        self.goal_checker_selector_pub = self.create_publisher(
            String, self.goal_checker_selector_topic, selector_qos
        )
        # 发布：到达状态 (True=成功, False=失败)
        self.result_pub = self.create_publisher(Bool, '/agent/arrival_status', 10)
        self.pre_grab_reached_pub = self.create_publisher(Bool, PRE_GRAB_REACHED_TOPIC, 10)
        
        # 发布：实时状态描述 (可选，用于调试)
        self.contact_approach_cmd_pub = self.create_publisher(
            Twist, self.contact_approach_cmd_vel_topic, 10
        )
        self.chassis_mode_pub = self.create_publisher(Int32, CHASSIS_MODE_TOPIC, 10)
        self.slope_speed_boost_pub = self.create_publisher(
            Bool,
            self.slope_speed_boost_topic,
            selector_qos,
        )
        self.slope_speed_boost_pub.publish(Bool(data=False))

      
        self.nav_timer = self.create_timer(0.25, self.nav_status_callback)
        self.nav_timer.cancel() # 默认先关闭定时器

        self.get_logger().info('✅ 导航代理节点已就绪 (异步模式)，等待指令...')
        self.get_logger().info(
            f'控制器自动切换已启用: > {self.controller_switch_distance:.2f}m 用 '
            f'{self.long_distance_controller}, <= {self.controller_switch_distance:.2f}m 用 '
            f'{self.short_distance_controller}'
        )
        self.get_logger().info(
            '一区转二区控制器切换: '
            f'剩余距离 <= {self.area1_to_area2_controller_switch_distance:.2f}m '
            f'切 {self.short_distance_controller}'
        )
        self.get_logger().info(
            f'失败重试控制器策略: 先 {self.long_distance_controller}，'
            f'剩余距离 <= {self.retry_controller_switch_distance:.2f}m 切 {self.short_distance_controller}'
        )
        self.get_logger().info(
            '抓取点单侧验收: '
            f'enabled={self.grab_one_sided_arrival_enabled} '
            f'axis={self.grab_one_sided_axis} '
            f'side_mode={self.grab_one_sided_side_mode} '
            f'correction_margin={self.grab_one_sided_correction_margin:.3f}m '
            f'max_corrections={self.grab_one_sided_max_corrections}'
        )
        self.get_logger().info(
            '抓取点顶靠验收: '
            f'enabled={self.grab_contact_arrival_enabled} '
            f'window={self.grab_contact_window_sec:.2f}s '
            f'slack={self.grab_contact_window_slack_sec:.2f}s '
            f'position_epsilon={self.grab_contact_position_epsilon:.3f}m '
            f'max_remaining={self.grab_contact_max_remaining:.3f}m '
            f'min_samples={self.grab_contact_min_samples} '
            f'require_preferred_side={self.grab_contact_require_preferred_side}'
        )
        self.get_logger().info(
            '抓取点顶靠调试: '
            f'enabled={self.grab_contact_debug_enabled} '
            f'interval={self.grab_contact_debug_interval_sec:.2f}s '
            f'distance_margin={self.grab_contact_debug_distance_margin:.3f}m'
        )
        self.get_logger().info(
            '低速到点判定: '
            f'enabled={self.low_speed_arrival_enabled} '
            f'|vx|<{self.low_speed_arrival_vx_threshold:.2f}m/s '
            f'|vy|<{self.low_speed_arrival_vy_threshold:.2f}m/s '
            f'remaining<={self.low_speed_arrival_max_remaining:.2f}m'
        )
        self.get_logger().info(
            f'重试中止保持: topic={self.retry_abort_hold_topic} '
            '收到 true 后取消当前 Nav2 目标并切 HOLD'
        )
        self.get_logger().info(
            '抓取点固定后退顶靠: '
            f'enabled={self.contact_approach_enabled} '
            f'cmd_vel_topic={self.contact_approach_cmd_vel_topic} '
            f'speed_x={self.contact_approach_speed_x:.3f}m/s '
            f'timeout={self.contact_approach_timeout_sec:.2f}s '
            f'max_distance={self.contact_approach_max_distance:.3f}m '
            f'min_distance={self.contact_approach_min_distance:.3f}m '
            f'period={self.contact_approach_command_period_sec:.2f}s '
            f'require_preferred_side={self.contact_approach_require_preferred_side} '
            f'result_delay={self.contact_approach_result_delay_sec:.2f}s'
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_cmd_vx = float(msg.twist.twist.linear.x)
        self.current_cmd_vy = float(msg.twist.twist.linear.y)
        self.record_grab_contact_sample()
        if self.contact_approach_active:
            self.evaluate_contact_approach(log_debug=False)

    def retry_abort_hold_callback(self, msg: Bool):
        if not bool(msg.data):
            return

        if self.retry_hold_active:
            self.publish_stop_velocity()
            self.publish_chassis_mode(CHASSIS_MODE_HOLD)
            self.get_logger().warn('收到重复 retry abort/hold 指令：已在 HOLD，继续保持')
            return

        self.get_logger().warn('收到 retry abort/hold 指令：取消当前导航并切 HOLD')
        self.retry_hold_active = True
        if self.contact_approach_result_timer is not None:
            self.contact_approach_result_timer.cancel()
            self.contact_approach_result_timer = None
        self.cancel_contact_approach(publish_result=False)

        try:
            if not self.navigator.isTaskComplete():
                self.navigator.cancelTask()
        except Exception as exc:
            self.get_logger().warn(f'retry abort/hold 取消 Nav2 任务失败: {exc}')

        self.nav_timer.cancel()
        self.publish_stop_velocity()
        self.publish_chassis_mode(CHASSIS_MODE_HOLD)
        self.active_goal = None
        self.reference_goal = None
        self.active_controller = None
        self.active_goal_checker = None
        self.pending_retry_goal_signature = None
        self.retry_controller_switch_distance_override = None
        self.area1_to_area2_goal_active = False
        self.grab_correction_attempts = 0
        self.reset_grab_contact_tracking()
        self.get_logger().warn(
            'retry abort/hold 已完成：底盘保持 HOLD，等待下一个 /goal_pose 或 /mode 恢复导航'
        )

    def release_retry_hold(self, reason: str):
        if not self.retry_hold_active:
            return
        self.retry_hold_active = False
        self.publish_chassis_mode(CHASSIS_MODE_NAVIGATION)
        self.get_logger().warn(f'retry HOLD 已解除，恢复 NAVIGATION | reason={reason}')

    def goal_checker_callback(self, msg: String):
        checker_id = msg.data.strip()
        if not checker_id:
            return
        if self.is_area3_goal_checker_enabled() and checker_id != self.area3_goal_checker_name:
            self.publish_goal_checker_selection(
                self.area3_goal_checker_name,
                f'/mode={AREA3_GOAL_CHECKER_MODE_VALUE} 区域三导航覆盖外部选择 {checker_id}',
            )
            return
        if checker_id == self.selected_goal_checker:
            return
        self.selected_goal_checker = checker_id
        self.get_logger().info(f'收到 goal checker 选择: {checker_id}')

    def external_mode_callback(self, msg: Int32):
        mode_value = int(msg.data)
        if mode_value == AREA3_GOAL_CHECKER_MODE_VALUE:
            self.set_slope_speed_boost(False, '/mode=3，关闭斜坡速度补偿')
        if (
            mode_value == self.external_mode_value
            and not self.contact_approach_active
            and not self.retry_hold_active
        ):
            return
        previous_mode_value = self.external_mode_value
        self.external_mode_value = mode_value
        self.get_logger().info(f'收到外部 /mode={mode_value}')
        if previous_mode_value == 1 and mode_value == 2:
            self.area1_to_area2_goal_pending = True
            self.get_logger().info(
                '检测到一区转二区：下一导航目标的 TEB 切换阈值设为 '
                f'{self.area1_to_area2_controller_switch_distance:.2f}m'
            )
        elif mode_value != 2:
            self.area1_to_area2_goal_pending = False
        self.release_retry_hold(f'/mode={mode_value}')

        if self.is_area3_goal_checker_enabled():
            self.publish_goal_checker_selection(
                self.area3_goal_checker_name,
                f'/mode={AREA3_GOAL_CHECKER_MODE_VALUE}：区域三导航启用独立 goal checker',
            )
            self.publish_chassis_mode(CHASSIS_MODE_NAVIGATION)
            return

        if self.is_contact_logic_enabled():
            self.get_logger().info('/mode=1：顶靠逻辑已开启')
            return

        if self.contact_approach_active:
            self.get_logger().warn(
                f'/mode={mode_value} 关闭顶靠逻辑，当前固定后退顶靠已中止并切回正常导航'
            )
            self.cancel_contact_approach(publish_result=False)
        else:
            self.reset_grab_contact_tracking()
        self.publish_chassis_mode(CHASSIS_MODE_NAVIGATION)
        self.get_logger().info(f'/mode={mode_value}：顶靠逻辑关闭，底盘保持 NAVIGATION')

    def speed_up_callback(self, msg: Bool):
        self.set_slope_speed_boost(bool(msg.data), f'{self.speed_up_topic}={bool(msg.data)}')

    def set_slope_speed_boost(self, enabled: bool, reason: str):
        enabled = bool(enabled)
        if enabled == self.slope_speed_boost_active:
            return
        self.slope_speed_boost_active = enabled
        self.slope_speed_boost_pub.publish(Bool(data=enabled))
        state_text = '开启' if enabled else '关闭'
        self.get_logger().info(
            f'斜坡速度补偿{state_text}: publish {self.slope_speed_boost_topic}={enabled} | reason={reason}'
        )

    def is_contact_logic_enabled(self) -> bool:
        return self.external_mode_value == CONTACT_LOGIC_MODE_VALUE

    def is_area3_goal_checker_enabled(self) -> bool:
        return (
            self.external_mode_value == AREA3_GOAL_CHECKER_MODE_VALUE
            and bool(self.area3_goal_checker_name)
        )

    def publish_goal_checker_selection(self, checker_id: str, reason: str):
        if not checker_id:
            return
        self.selected_goal_checker = checker_id
        self.goal_checker_selector_pub.publish(String(data=checker_id))
        self.get_logger().info(f'已切换 goal checker -> {checker_id} | {reason}')

    def estimate_distance_to_goal(self, goal: PoseStamped):
        if self.current_x is None or self.current_y is None:
            return None

        dx = goal.pose.position.x - self.current_x
        dy = goal.pose.position.y - self.current_y
        return math.hypot(dx, dy)

    @staticmethod
    def build_goal_signature(goal: PoseStamped | None):
        if goal is None:
            return None
        return (
            round(goal.pose.position.x, 3),
            round(goal.pose.position.y, 3),
            round(goal.pose.position.z, 3),
            round(goal.pose.orientation.z, 3),
            round(goal.pose.orientation.w, 3),
        )

    def get_active_controller_switch_distance(self) -> float:
        if self.retry_controller_switch_distance_override is not None:
            return self.retry_controller_switch_distance_override
        if self.area1_to_area2_goal_active:
            return self.area1_to_area2_controller_switch_distance
        return self.controller_switch_distance

    def select_controller_by_distance(self, distance_remaining, source: str):
        if distance_remaining is None:
            return

        desired_controller = self.long_distance_controller
        switch_distance = self.get_active_controller_switch_distance()
        hysteresis = max(0.0, self.controller_switch_hysteresis)

        if self.active_controller == self.short_distance_controller:
            if distance_remaining <= switch_distance + hysteresis:
                desired_controller = self.short_distance_controller
        elif distance_remaining <= switch_distance:
            desired_controller = self.short_distance_controller

        if desired_controller == self.active_controller:
            return

        self.controller_selector_pub.publish(String(data=desired_controller))
        self.active_controller = desired_controller
        self.get_logger().info(
            f'切换控制器 -> {desired_controller} | 剩余距离={distance_remaining:.2f}m | 来源={source}'
        )

    def goal_callback(self, msg):
        """
        收到新目标时的回调函数
        """
        now = self.get_clock().now()
        goal_signature = self.build_goal_signature(msg)
        active_goal_signature = self.build_goal_signature(self.active_goal)
        hold_was_active = self.retry_hold_active
        area1_to_area2_goal_was_active = self.area1_to_area2_goal_active
        is_pending_retry_goal = (
            self.pending_retry_goal_signature == goal_signature
        )
        is_active_goal_reissue = (
            active_goal_signature == goal_signature
        )
        if (
            not hold_was_active and
            not is_active_goal_reissue and
            not is_pending_retry_goal and
            self.last_goal_signature == goal_signature and
            self.last_goal_received_time is not None and
            (now - self.last_goal_received_time).nanoseconds / 1e9 <= self.duplicate_goal_time_window
        ):
            self.get_logger().info(
                f'忽略重复目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) '
                f'| 窗口={self.duplicate_goal_time_window:.2f}s'
            )
            return

        self.last_goal_signature = goal_signature
        self.last_goal_received_time = now
        self.pending_retry_goal_signature = None
        force_controller = None
        if is_pending_retry_goal:
            self.retry_controller_switch_distance_override = self.retry_controller_switch_distance
            force_controller = self.long_distance_controller
        elif is_active_goal_reissue:
            self.retry_controller_switch_distance_override = None
            self.get_logger().warn(
                f'收到同目标重发，立即重新导航: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )
        else:
            self.retry_controller_switch_distance_override = None
        self.active_goal_checker = self.selected_goal_checker or self.grab_goal_checker_name
        self.area1_to_area2_goal_active = (
            not is_pending_retry_goal
            and (
                (self.area1_to_area2_goal_pending and not is_active_goal_reissue)
                or (is_active_goal_reissue and area1_to_area2_goal_was_active)
            )
        )
        if self.area1_to_area2_goal_active:
            self.area1_to_area2_goal_pending = False
        if hold_was_active:
            self.release_retry_hold('/goal_pose')
        self.get_logger().info(
            f'📍 收到新指令: 前往 ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) '
            f'| goal_checker={self.active_goal_checker}'
        )
        self.reference_goal = self.clone_goal(msg)
        self.grab_correction_attempts = 0
        self.reset_grab_contact_tracking()
        dispatch_source = (
            'goal_dispatch_retry_mppi_then_teb'
            if is_pending_retry_goal else
            'goal_dispatch'
        )
        self.dispatch_navigation_goal(
            msg,
            source=dispatch_source,
            allow_cancel=True,
            force_controller=force_controller,
        )

    def dispatch_navigation_goal(
        self,
        goal: PoseStamped,
        source: str,
        allow_cancel: bool,
        force_controller: str | None = None,
    ):
        self.cancel_contact_approach(publish_result=False)
        self.active_goal = goal
        self.reset_grab_contact_tracking()
        self.publish_chassis_mode(CHASSIS_MODE_NAVIGATION)

        if allow_cancel and not self.navigator.isTaskComplete():
            self.get_logger().warn('⚠️ 检测到新目标，正在中止当前任务...')
            self.navigator.cancelTask()

        initial_distance = self.estimate_distance_to_goal(goal)
        active_switch_distance = self.get_active_controller_switch_distance()
        if initial_distance is not None:
            self.get_logger().info(
                f'目标初始直线距离={initial_distance:.2f}m | '
                f'TEB切换阈值={active_switch_distance:.2f}m'
            )
        if force_controller:
            self.controller_selector_pub.publish(String(data=force_controller))
            self.active_controller = force_controller
            self.get_logger().info(
                f'强制切换控制器 -> {force_controller} | 来源={source}'
            )
        else:
            self.select_controller_by_distance(initial_distance, source)
        self.navigator.goToPose(goal)
        self.nav_timer.reset()
        self.get_logger().info(f'🚀 导航开始，监控线程已启动 | source={source}')

    @staticmethod
    def clone_goal(goal: PoseStamped) -> PoseStamped:
        cloned = PoseStamped()
        cloned.header.stamp = goal.header.stamp
        cloned.header.frame_id = goal.header.frame_id
        cloned.pose.position.x = goal.pose.position.x
        cloned.pose.position.y = goal.pose.position.y
        cloned.pose.position.z = goal.pose.position.z
        cloned.pose.orientation.x = goal.pose.orientation.x
        cloned.pose.orientation.y = goal.pose.orientation.y
        cloned.pose.orientation.z = goal.pose.orientation.z
        cloned.pose.orientation.w = goal.pose.orientation.w
        return cloned

    def get_current_axis_value(self):
        return self.current_x if self.grab_one_sided_axis == 'x' else self.current_y

    def get_goal_axis_value(self, goal: PoseStamped):
        if self.grab_one_sided_axis == 'x':
            return goal.pose.position.x
        return goal.pose.position.y

    def should_apply_grab_one_sided_logic(self) -> bool:
        return (
            self.grab_one_sided_arrival_enabled
            and self.active_goal_checker == self.grab_goal_checker_name
            and self.reference_goal is not None
        )

    def should_monitor_grab_contact(self) -> bool:
        return (
            self.grab_contact_arrival_enabled
            and self.is_contact_logic_enabled()
            and not self.contact_arrival_pending_success
            and self.active_goal_checker == self.grab_goal_checker_name
            and self.active_goal is not None
            and self.reference_goal is not None
        )

    def should_use_contact_approach(self) -> bool:
        return (
            self.contact_approach_enabled
            and self.is_contact_logic_enabled()
            and self.active_goal_checker == self.grab_goal_checker_name
            and self.active_goal is not None
            and self.reference_goal is not None
            and not self.contact_approach_active
        )

    def should_allow_low_speed_arrival(self) -> bool:
        return (
            self.low_speed_arrival_enabled
            and self.active_goal is not None
            and self.active_goal_checker != self.grab_goal_checker_name
            and not self.contact_approach_active
        )

    def try_trigger_low_speed_arrival(self, remaining: float | None) -> bool:
        if not self.should_allow_low_speed_arrival():
            return False
        if self.contact_arrival_pending_success:
            return False
        if remaining is None or remaining > self.low_speed_arrival_max_remaining:
            return False

        if abs(self.current_cmd_vx) >= self.low_speed_arrival_vx_threshold:
            return False
        if abs(self.current_cmd_vy) >= self.low_speed_arrival_vy_threshold:
            return False

        self.contact_arrival_pending_success = True
        self.contact_arrival_pending_reason = (
            '低速到点判定成功: '
            f'remaining={remaining:.3f}m '
            f'vx={self.current_cmd_vx:.3f}m/s '
            f'vy={self.current_cmd_vy:.3f}m/s'
        )
        self.get_logger().warn(
            f'{self.contact_arrival_pending_reason}，主动结束当前 Nav2 任务并转发到达成功'
        )
        self.navigator.cancelTask()
        return True

    def should_collect_grab_contact_samples(self) -> bool:
        return self.should_monitor_grab_contact() or self.contact_approach_active

    def reset_grab_contact_tracking(self):
        self.grab_contact_samples.clear()
        self.contact_arrival_pending_success = False
        self.contact_arrival_pending_reason = ''
        self.last_grab_contact_debug_time = None

    def record_grab_contact_sample(self):
        if not self.should_collect_grab_contact_samples():
            if self.grab_contact_samples:
                self.grab_contact_samples.clear()
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.grab_contact_samples.append((now_sec, self.current_x, self.current_y))
        cutoff_sec = now_sec - self.grab_contact_window_sec
        while self.grab_contact_samples and self.grab_contact_samples[0][0] < cutoff_sec:
            self.grab_contact_samples.popleft()

    def calculate_grab_contact_metrics(self):
        sample_count = len(self.grab_contact_samples)
        if sample_count <= 0:
            return 0, 0.0, None

        start_time = self.grab_contact_samples[0][0]
        end_time = self.grab_contact_samples[-1][0]
        duration = max(0.0, end_time - start_time)

        x_values = [sample[1] for sample in self.grab_contact_samples]
        y_values = [sample[2] for sample in self.grab_contact_samples]
        span = math.hypot(
            max(x_values) - min(x_values),
            max(y_values) - min(y_values),
        )
        return sample_count, duration, span

    def get_grab_contact_span(self):
        sample_count, duration, span = self.calculate_grab_contact_metrics()
        if sample_count < self.grab_contact_min_samples:
            return None

        if duration + self.grab_contact_window_slack_sec < self.grab_contact_window_sec:
            return None

        return span, duration

    def is_on_contact_preferred_side(self) -> bool:
        if not self.grab_contact_require_preferred_side:
            return True

        current_value = self.get_current_axis_value()
        if current_value is None or self.reference_goal is None:
            return False

        target_value = self.get_goal_axis_value(self.reference_goal)
        return self.is_on_preferred_side(current_value, target_value)

    def should_log_grab_contact_debug(self, remaining: float | None) -> bool:
        if not self.grab_contact_debug_enabled or remaining is None:
            return False

        debug_distance_threshold = (
            self.grab_contact_max_remaining + self.grab_contact_debug_distance_margin
        )
        if remaining > debug_distance_threshold:
            return False

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.last_grab_contact_debug_time is None:
            self.last_grab_contact_debug_time = now_sec
            return True

        if now_sec - self.last_grab_contact_debug_time < self.grab_contact_debug_interval_sec:
            return False

        self.last_grab_contact_debug_time = now_sec
        return True

    def log_grab_contact_debug(self, remaining: float | None):
        if not self.should_log_grab_contact_debug(remaining):
            return

        sample_count, duration, span = self.calculate_grab_contact_metrics()
        current_value = self.get_current_axis_value()
        target_value = None
        if self.reference_goal is not None:
            target_value = self.get_goal_axis_value(self.reference_goal)

        preferred_side_ok = self.is_on_contact_preferred_side()
        reasons = []
        if remaining is None:
            reasons.append('remaining_none')
        elif remaining > self.grab_contact_max_remaining:
            reasons.append('remaining_gt')
        if sample_count < self.grab_contact_min_samples:
            reasons.append('samples_short')
        if duration + self.grab_contact_window_slack_sec < self.grab_contact_window_sec:
            reasons.append('window_short')
        if span is None:
            reasons.append('span_none')
        elif span > self.grab_contact_position_epsilon:
            reasons.append('span_gt')
        if not preferred_side_ok:
            reasons.append('wrong_side')
        if not reasons:
            reasons.append('ready_to_trigger')

        current_str = 'None' if current_value is None else f'{current_value:.4f}'
        target_str = 'None' if target_value is None else f'{target_value:.4f}'
        remaining_str = 'None' if remaining is None else f'{remaining:.3f}'
        span_str = 'None' if span is None else f'{span:.4f}'
        self.get_logger().info(
            '[grab_contact_debug] '
            f'axis={self.grab_one_sided_axis} '
            f'current={current_str} target={target_str} '
            f'remaining={remaining_str}/{self.grab_contact_max_remaining:.3f} '
            f'span={span_str}/{self.grab_contact_position_epsilon:.4f} '
            f'window={duration:.3f}/{self.grab_contact_window_sec:.3f}s '
            f'slack={self.grab_contact_window_slack_sec:.3f}s '
            f'samples={sample_count}/{self.grab_contact_min_samples} '
            f'preferred_side={preferred_side_ok} '
            f'reasons={",".join(reasons)}'
        )

    def try_trigger_grab_contact_arrival(self, remaining: float | None) -> bool:
        if not self.should_monitor_grab_contact():
            return False

        self.log_grab_contact_debug(remaining)

        if remaining is None or remaining > self.grab_contact_max_remaining:
            return False

        span_state = self.get_grab_contact_span()
        if span_state is None:
            return False

        span, duration = span_state
        if span > self.grab_contact_position_epsilon:
            return False

        if not self.is_on_contact_preferred_side():
            return False

        self.contact_arrival_pending_success = True
        self.contact_arrival_pending_reason = (
            '抓取点触发顶靠验收成功: '
            f'remaining={remaining:.3f}m '
            f'span={span:.4f}m '
            f'window={duration:.2f}s'
        )
        self.get_logger().warn(
            f'{self.contact_arrival_pending_reason}，主动结束当前 Nav2 任务并转发到达成功'
        )
        self.navigator.cancelTask()
        return True

    def publish_contact_approach_velocity(self):
        msg = Twist()
        msg.linear.x = self.contact_approach_speed_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.contact_approach_cmd_pub.publish(msg)
        self.last_contact_approach_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def publish_stop_velocity(self):
        msg = Twist()
        for _ in range(20):
            self.contact_approach_cmd_pub.publish(msg)

    def publish_chassis_mode(self, mode: int):
        self.chassis_mode_pub.publish(Int32(data=mode))

    def publish_pre_grab_reached(self):
        self.pre_grab_reached_pub.publish(Bool(data=True))

    def start_contact_approach(self) -> bool:
        if self.current_x is None or self.current_y is None:
            self.get_logger().error('固定后退顶靠缺少 /Odometry，无法安全执行')
            return False

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.contact_approach_active = True
        self.contact_approach_start_time = now_sec
        self.contact_approach_start_x = self.current_x
        self.contact_approach_start_y = self.current_y
        self.last_contact_approach_cmd_time = None
        self.reset_grab_contact_tracking()
        self.record_grab_contact_sample()
        self.publish_chassis_mode(CHASSIS_MODE_CONTACT_APPROACH)
        self.publish_contact_approach_velocity()
        self.publish_pre_grab_reached()
        self.nav_timer.reset()
        self.get_logger().warn(
            'Nav2 已到预接触点，进入固定后退顶靠阶段: '
            f'vx={self.contact_approach_speed_x:.3f}m/s '
            f'timeout={self.contact_approach_timeout_sec:.2f}s '
            f'min_distance={self.contact_approach_min_distance:.3f}m '
            f'max_distance={self.contact_approach_max_distance:.3f}m'
        )
        return True

    def cancel_contact_approach(self, publish_result: bool):
        if not self.contact_approach_active:
            return

        self.publish_stop_velocity()
        self.publish_chassis_mode(CHASSIS_MODE_HOLD)
        self.contact_approach_active = False
        self.contact_approach_start_time = None
        self.contact_approach_start_x = None
        self.contact_approach_start_y = None
        self.last_contact_approach_cmd_time = None
        self.reset_grab_contact_tracking()
        if publish_result:
            self.publish_final_result(False)

    def get_contact_approach_distance(self):
        if (
            self.current_x is None
            or self.current_y is None
            or self.contact_approach_start_x is None
            or self.contact_approach_start_y is None
        ):
            return None

        return math.hypot(
            self.current_x - self.contact_approach_start_x,
            self.current_y - self.contact_approach_start_y,
        )

    def try_trigger_contact_approach_arrival(self, traveled: float | None) -> bool:
        if not self.contact_approach_active:
            return False

        if traveled is None or traveled < self.contact_approach_min_distance:
            return False

        span_state = self.get_grab_contact_span()
        if span_state is None:
            return False

        span, _ = span_state
        if span > self.grab_contact_position_epsilon:
            return False

        if self.contact_approach_require_preferred_side and not self.is_on_contact_preferred_side():
            return False

        return True

    def log_contact_approach_debug(self, elapsed: float, traveled: float | None):
        if not self.grab_contact_debug_enabled:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            self.last_grab_contact_debug_time is not None
            and now_sec - self.last_grab_contact_debug_time < self.grab_contact_debug_interval_sec
        ):
            return
        self.last_grab_contact_debug_time = now_sec

        sample_count, duration, span = self.calculate_grab_contact_metrics()
        traveled_str = 'None' if traveled is None else f'{traveled:.3f}'
        span_str = 'None' if span is None else f'{span:.4f}'
        self.get_logger().info(
            '[contact_approach_debug] '
            f'elapsed={elapsed:.2f}/{self.contact_approach_timeout_sec:.2f}s '
            f'traveled={traveled_str}/{self.contact_approach_max_distance:.3f}m '
            f'min_traveled={self.contact_approach_min_distance:.3f}m '
            f'span={span_str}/{self.grab_contact_position_epsilon:.4f} '
            f'window={duration:.3f}/{self.grab_contact_window_sec:.3f}s '
            f'samples={sample_count}/{self.grab_contact_min_samples}'
        )

    def evaluate_contact_approach(self, log_debug: bool = True) -> bool:
        if not self.contact_approach_active or self.contact_approach_start_time is None:
            return False

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.contact_approach_start_time
        traveled = self.get_contact_approach_distance()
        if log_debug:
            self.log_contact_approach_debug(elapsed, traveled)

        if elapsed > self.contact_approach_timeout_sec:
            self.get_logger().error(
                '固定后退顶靠超时，停车并按失败处理: '
                f'elapsed={elapsed:.2f}s timeout={self.contact_approach_timeout_sec:.2f}s'
            )
            self.finish_contact_approach(False)
            return True

        if self.try_trigger_contact_approach_arrival(traveled):
            sample_count, duration, span = self.calculate_grab_contact_metrics()
            span_str = 'None' if span is None else f'{span:.4f}'
            traveled_str = 'None' if traveled is None else f'{traveled:.3f}'
            self.contact_arrival_pending_reason = (
                '固定后退顶靠验收成功: '
                f'traveled={traveled_str}m '
                f'span={span_str}m '
                f'window={duration:.2f}s '
                f'samples={sample_count}'
            )
            self.get_logger().warn(f'{self.contact_arrival_pending_reason}，发送 0 速度并上报到达成功')
            self.finish_contact_approach(True)
            return True

        return False

    def handle_contact_approach(self):
        if self.evaluate_contact_approach(log_debug=True):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.last_contact_approach_cmd_time is None:
            self.publish_contact_approach_velocity()
        elif now_sec - self.last_contact_approach_cmd_time >= self.contact_approach_command_period_sec:
            self.publish_contact_approach_velocity()

    def finish_contact_approach(self, success: bool):
        self.publish_stop_velocity()
        self.publish_chassis_mode(CHASSIS_MODE_HOLD)
        self.contact_approach_active = False
        self.contact_approach_start_time = None
        self.contact_approach_start_x = None
        self.contact_approach_start_y = None
        self.last_contact_approach_cmd_time = None
        self.nav_timer.cancel()
        if self.contact_approach_result_delay_sec > 0.0:
            if self.contact_approach_result_timer is not None:
                self.contact_approach_result_timer.cancel()
            self.get_logger().info(
                f'固定后退顶靠结束，延时 {self.contact_approach_result_delay_sec:.1f}s 后发布到达结果'
            )
            self.contact_approach_result_timer = self.create_timer(
                self.contact_approach_result_delay_sec,
                lambda: self.publish_delayed_contact_approach_result(success),
            )
            return
        self.publish_final_result(success)

    def publish_delayed_contact_approach_result(self, success: bool):
        if self.contact_approach_result_timer is not None:
            self.contact_approach_result_timer.cancel()
            self.contact_approach_result_timer = None
        self.publish_final_result(success)

    def publish_final_result(self, success: bool):
        msg_result = Bool()
        msg_result.data = success
        if not success and self.last_goal_signature is not None:
            self.pending_retry_goal_signature = self.last_goal_signature
        else:
            self.pending_retry_goal_signature = None
        self.retry_controller_switch_distance_override = None
        self.area1_to_area2_goal_active = False
        self.active_goal = None
        self.reference_goal = None
        self.active_controller = None
        self.active_goal_checker = None
        self.grab_correction_attempts = 0
        self.reset_grab_contact_tracking()
        self.result_pub.publish(msg_result)

    def is_on_preferred_side(self, current_value: float, target_value: float) -> bool:
        eps = self.grab_one_sided_side_epsilon
        mode = self.grab_one_sided_side_mode
        if mode == 'closer_to_zero':
            return abs(current_value) <= abs(target_value) + eps
        if mode == 'further_from_zero':
            return abs(current_value) >= abs(target_value) - eps
        if mode == 'less_than_target':
            return current_value <= target_value + eps
        return current_value >= target_value - eps

    def build_grab_correction_goal(self) -> PoseStamped | None:
        if self.reference_goal is None:
            return None

        margin = self.grab_one_sided_correction_margin * max(1, self.grab_correction_attempts + 1)
        target_value = self.get_goal_axis_value(self.reference_goal)
        corrected_value = target_value

        if self.grab_one_sided_side_mode == 'closer_to_zero':
            direction = -math.copysign(1.0, target_value) if abs(target_value) > 1e-6 else 0.0
            corrected_value = target_value + direction * margin
        elif self.grab_one_sided_side_mode == 'further_from_zero':
            direction = math.copysign(1.0, target_value) if abs(target_value) > 1e-6 else 0.0
            corrected_value = target_value + direction * margin
        elif self.grab_one_sided_side_mode == 'less_than_target':
            corrected_value = target_value - margin
        elif self.grab_one_sided_side_mode == 'greater_than_target':
            corrected_value = target_value + margin

        correction_goal = self.clone_goal(self.reference_goal)
        if self.grab_one_sided_axis == 'x':
            correction_goal.pose.position.x = corrected_value
        else:
            correction_goal.pose.position.y = corrected_value
        return correction_goal

    def try_handle_grab_one_sided_success(self) -> bool:
        if not self.should_apply_grab_one_sided_logic():
            return False

        current_value = self.get_current_axis_value()
        if current_value is None:
            self.get_logger().warn('抓取点单侧验收缺少当前里程计，沿用 Nav2 成功结果')
            return False

        target_value = self.get_goal_axis_value(self.reference_goal)
        if self.is_on_preferred_side(current_value, target_value):
            self.get_logger().info(
                f'抓取点单侧验收通过: axis={self.grab_one_sided_axis} '
                f'current={current_value:.4f} target={target_value:.4f} '
                f'mode={self.grab_one_sided_side_mode}'
            )
            return False

        axis_error = current_value - target_value
        self.get_logger().warn(
            f'抓取点落在非期望侧，准备回拉修正: axis={self.grab_one_sided_axis} '
            f'current={current_value:.4f} target={target_value:.4f} '
            f'error={axis_error:+.4f} mode={self.grab_one_sided_side_mode} '
            f'attempt={self.grab_correction_attempts + 1}/{self.grab_one_sided_max_corrections}'
        )

        if self.grab_correction_attempts >= self.grab_one_sided_max_corrections:
            self.get_logger().warn('抓取点单侧修正次数已耗尽，回退为接受当前 Nav2 成功结果')
            return False

        correction_goal = self.build_grab_correction_goal()
        if correction_goal is None:
            self.get_logger().warn('构造抓取点回拉目标失败，沿用当前 Nav2 成功结果')
            return False

        self.grab_correction_attempts += 1
        self.dispatch_navigation_goal(
            correction_goal,
            source=f'grab_one_sided_correction_{self.grab_correction_attempts}',
            allow_cancel=False,
        )
        return True

    def nav_status_callback(self):
        """
        定时器回调：每 0.5 秒运行一次，检查导航进度
        """
        if self.contact_approach_active:
            self.handle_contact_approach()
            return

        # 1. 如果任务还没完成 -> 打印反馈
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            remaining = None
            if feedback:
                # 打印剩余距离
                remaining = feedback.distance_remaining
            elif self.active_goal is not None:
                remaining = self.estimate_distance_to_goal(self.active_goal)

            self.select_controller_by_distance(remaining, 'nav_feedback')
            if self.try_trigger_low_speed_arrival(remaining):
                return
            if not self.should_use_contact_approach() and self.try_trigger_grab_contact_arrival(remaining):
                return

            if feedback:
                time_taken = feedback.navigation_time.sec
                
                log_msg = f'🚗 正在行驶... 剩余距离: {remaining:.2f}m | 耗时: {time_taken}s'
                self.feedback_pub.publish(String(data=log_msg))
            return 

        self.nav_timer.cancel()
        
        result = self.navigator.getResult()
        msg_result = Bool()
        
        if result == TaskResult.SUCCEEDED:
            if self.try_handle_grab_one_sided_success():
                return
            if self.should_use_contact_approach():
                if self.start_contact_approach():
                    return
                msg_result.data = False
                self.publish_stop_velocity()
                self.publish_final_result(False)
                return
            self.get_logger().info('🎉 导航成功！已到达目的地。')
            msg_result.data = True
        elif result == TaskResult.CANCELED:
            if self.contact_arrival_pending_success:
                self.get_logger().warn(
                    f'{self.contact_arrival_pending_reason}，按抓取点到达成功处理'
                )
                msg_result.data = True
            else:
                self.get_logger().warn('🛑 导航任务被取消。')
                msg_result.data = False
        elif result == TaskResult.FAILED:
            self.get_logger().error('💀 导航失败 (路径受阻或规划失败)。')
            msg_result.data = False
        else:
            self.get_logger().error('❓ 未知状态。')
            msg_result.data = False
            
        # 发布最终结果给主控节点
        if msg_result.data:
            self.publish_stop_velocity()
        self.publish_chassis_mode(CHASSIS_MODE_HOLD)
        self.publish_final_result(msg_result.data)

    def destroy_node(self):
        """
        当节点被关闭时 (Ctrl+C)，确保车能停下来
        """
        self.get_logger().info('正在关闭节点，尝试中止导航...')
        try:
            self.publish_stop_velocity()
            self.navigator.cancelTask()
        except:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = NavAgent()
    try:
        # 使用 spin 保持节点活跃，处理回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
