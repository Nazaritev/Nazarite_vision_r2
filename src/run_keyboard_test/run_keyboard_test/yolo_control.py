"""收到 /slope=3 后左右对中、前后调整，可选择稳定到位后释放串口。"""

import math
import time

import rclpy
from rclpy.node import Node
from tur3_run_interfaces.msg import Slope
from yolo_detect_box.msg import BoxDetection
from yolo_detect_box.srv import CameraUse

from run_keyboard_test.serial_velocity import SerialVelocity, speed_limit


class LateralAlignment:
    """保存视觉追踪状态并计算横移速度，串口发送由控制节点负责。"""

    def __init__(self, max_speed=0.3, gain=1.0,
                 left_ratio=0.4, right_ratio=0.6, direction=-1.0,
                 detection_timeout=0.5):
        """初始化控制参数和待机状态。

        max_speed：最大横移速度，单位 m/s，最终限制不超过 0.3。
        gain：把水平归一化偏差转换为速度的比例系数。
        left_ratio、right_ratio：停车范围占画面宽度的比例，默认 0.4 到 0.6。
        direction：横移方向符号，取 -1 或 1，用于适配相机安装方向。
        detection_timeout：检测结果允许使用的最长时间，单位秒。
        """
        self.max_speed = speed_limit(max_speed)
        values = (gain, left_ratio, right_ratio, direction, detection_timeout)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('控制参数必须为有限数值')
        if gain <= 0.0 or detection_timeout <= 0.0:
            raise ValueError('增益和检测超时必须大于零')
        if not 0.0 <= left_ratio < right_ratio <= 1.0:
            raise ValueError('中心范围必须满足 0 <= left_ratio < right_ratio <= 1')
        if direction not in (-1.0, 1.0):
            raise ValueError('lateral_direction 只能为 -1 或 1')
        self.gain = gain
        self.left_ratio = left_ratio
        self.right_ratio = right_ratio
        self.direction = direction
        self.detection_timeout = detection_timeout
        self.active = False
        self.in_center = False
        self.session_id = None
        self.last_received = None
        self.error = None

    def start(self, session_id):
        """开始一轮追踪，保存本轮编号，并清除上一轮的坐标和控制状态。

        在首次收到有效检测结果前，计算出的速度为零。
        """
        self.active = True
        self.in_center = False
        self.session_id = session_id
        self.last_received = None
        self.error = None

    def update(self, session_id, detected, fx, fy, width, height, now):
        """用最新一帧检测结果更新中心范围判断和水平偏差。

        session_id 用于确认消息属于本轮，detected 表示是否识别到方块。
        fx、fy 是框中心的像素坐标，width、height 是原图宽高。
        now 是收到消息时的单调时钟时间，单位秒，用于判断结果是否过期。
        无目标或坐标无效时清除偏差，使速度计算返回零；其他轮次的消息忽略。
        本函数只更新状态，不发送串口数据。
        """
        if not self.active or session_id != self.session_id:
            return
        self.last_received = now
        self.error = None
        self.in_center = False
        if (not detected or width <= 0 or height <= 0
                or not math.isfinite(fx) or not math.isfinite(fy)
                or not 0.0 <= fx < width or not 0.0 <= fy < height):
            return
        # 每帧重新判断范围；在范围内停车，移出后重新计算横移速度。
        self.in_center = self.left_ratio * width <= fx <= self.right_ratio * width
        target = width * (self.left_ratio + self.right_ratio) / 2.0
        self.error = (fx - target) / width

    def velocity(self, now):
        """根据最新水平偏差返回横移速度 Vy，单位 m/s，正负号表示方向。

        待机、框中心在停车范围内、无有效目标或检测结果过期时返回 0。
        范围外按比例计算速度，并限制在最大速度以内。
        每次使用最新状态判断，移出停车范围后即可继续追踪。
        """
        if (not self.active or self.in_center or self.error is None
                or self.last_received is None
                or now - self.last_received > self.detection_timeout):
            return 0.0
        # 只有一个速度上限；偏差决定速度大小，较大偏差可达到这个上限。
        speed = min(self.max_speed, self.gain * abs(self.error))
        # 默认图像右侧目标对应车体向右横移（车体 Vy 为负）；镜像相机可反转。
        return self.direction * math.copysign(speed, self.error)

    def finish(self):
        """结束本轮追踪并清除控制状态，让后续速度计算返回零。"""
        self.active = False
        self.in_center = False
        self.error = None


class BoxAlignment(LateralAlignment):
    """先横移对中，再按框中心 cy 与两条水平边界调节前后距离。"""

    def __init__(self, *, enable_forward=True, max_forward_speed=0.3,
                 forward_gain=1.0, forward_direction=1.0,
                 top_ratio=0.5, bottom_ratio=0.7, **kwargs):
        super().__init__(**kwargs)
        self.max_forward_speed = speed_limit(max_forward_speed)
        values = (forward_gain, forward_direction, top_ratio, bottom_ratio)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('前后控制参数必须为有限数值')
        if forward_gain <= 0.0:
            raise ValueError('forward_gain 必须大于零')
        if forward_direction not in (-1.0, 1.0):
            raise ValueError('forward_direction 只能为 -1 或 1')
        if not 0.0 <= top_ratio < bottom_ratio <= 1.0:
            raise ValueError('纵向中心范围必须满足 0 <= top_ratio < bottom_ratio <= 1')
        self.enable_forward = enable_forward
        self.forward_gain = forward_gain
        self.forward_direction = forward_direction
        self.top_ratio = top_ratio
        self.bottom_ratio = bottom_ratio
        self.in_vertical = False
        self.forward_error = None

    @property
    def in_position(self):
        """框中心同时处于左右和上下边界内才算到位。"""
        return self.in_center and (not self.enable_forward or self.in_vertical)

    def start(self, session_id):
        super().start(session_id)
        self.in_vertical = False
        self.forward_error = None

    def update(self, session_id, detected, fx, fy, width, height, now):
        if not self.active or session_id != self.session_id:
            return
        super().update(session_id, detected, fx, fy, width, height, now)
        self.in_vertical = False
        self.forward_error = None
        if not self.enable_forward or self.error is None:
            return
        fy_ratio = fy / height
        self.in_vertical = self.top_ratio <= fy_ratio <= self.bottom_ratio
        target = (self.top_ratio + self.bottom_ratio) / 2.0
        # 图像 y 向下增大；默认中心偏上前进、偏下后退，可用方向参数反转。
        self.forward_error = target - fy_ratio

    def forward_velocity(self, now):
        """cy 偏上前进、偏下后退；已在上下范围内、水平未对中或检测失效时为零。"""
        if (not self.enable_forward or not self.active or not self.in_center
                or self.forward_error is None or self.in_vertical
                or self.last_received is None
                or now - self.last_received > self.detection_timeout):
            return 0.0
        speed = min(self.max_forward_speed, self.forward_gain * abs(self.forward_error))
        return self.forward_direction * math.copysign(speed, self.forward_error)

    def finish(self):
        super().finish()
        self.in_vertical = False
        self.forward_error = None


class YoloControlNode(Node):
    def __init__(self, **kwargs):
        """读取 ROS 参数，创建检测订阅、服务客户端、串口对象和控制定时器。

        此时保持待机，收到 /slope 的触发值后才打开串口并启动检测。
        """
        super().__init__('yolo_control', **kwargs)
        defaults = {
            'slope_topic': '/slope',
            'trigger_value': 3,
            'camera_service': '/camara_use',
            'detection_topic': '/yolo/box_detection',
            'serial_port': '/dev/ttyUSB0',
            # 沿用现有 nav_to_pose/get_and_pub 的波特率。
            'baudrate': 961200,
            'max_lateral_speed': 0.3,
            'lateral_gain': 1.0,
            'center_left_ratio': 0.4,
            'center_right_ratio': 0.6,
            'enable_forward_control': True,
            'max_forward_speed': 0.3,
            'forward_gain': 1.0,
            'forward_direction': 1.0,
            'center_top_ratio': 0.5,
            'center_bottom_ratio': 0.7,
            'lock_on_center': False,
            'lock_stable_duration': 1.0,
            'lock_stop_duration': 1.0,
            'lateral_direction': -1.0,
            'control_rate': 20.0,
            'log_velocity': True,
            'detection_timeout': 0.5,
            'service_wait_timeout': 5.0,
            'stop_duration': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def value(name):
            """按参数名读取已声明的 ROS 参数值，供初始化控制配置使用。"""
            return self.get_parameter(name).value

        self.alignment = BoxAlignment(
            max_speed=float(value('max_lateral_speed')),
            gain=float(value('lateral_gain')),
            left_ratio=float(value('center_left_ratio')),
            right_ratio=float(value('center_right_ratio')),
            direction=float(value('lateral_direction')),
            enable_forward=value('enable_forward_control'),
            max_forward_speed=float(value('max_forward_speed')),
            forward_gain=float(value('forward_gain')),
            forward_direction=float(value('forward_direction')),
            top_ratio=float(value('center_top_ratio')),
            bottom_ratio=float(value('center_bottom_ratio')),
            detection_timeout=float(value('detection_timeout')))
        self.control_rate = float(value('control_rate'))
        self.log_velocity = value('log_velocity')
        self.last_velocity_log_time = None
        self.service_wait_timeout = float(value('service_wait_timeout'))
        self.stop_duration = float(value('stop_duration'))
        self.lock_on_center = value('lock_on_center')
        self.lock_stable_duration = float(value('lock_stable_duration'))
        self.lock_stop_duration = float(value('lock_stop_duration'))
        for name, number in (
                ('control_rate', self.control_rate),
                ('service_wait_timeout', self.service_wait_timeout),
                ('stop_duration', self.stop_duration),
                ('lock_stable_duration', self.lock_stable_duration),
                ('lock_stop_duration', self.lock_stop_duration)):
            if not math.isfinite(number) or number <= 0.0:
                raise ValueError(f'{name} 必须为有限正数')
        self.trigger_value = int(value('trigger_value'))
        self.writer = SerialVelocity(
            value('serial_port'), int(value('baudrate')), self.alignment.max_speed,
            max_forward_speed=self.alignment.max_forward_speed)
        self.client = self.create_client(CameraUse, value('camera_service'))
        self.slope_sub = self.create_subscription(
            Slope, value('slope_topic'), self.slope_callback, 10)
        self.detection_sub = self.create_subscription(
            BoxDetection, value('detection_topic'), self.detection_callback, 1)
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_tick)
        self.request_future = None
        self.service_deadline = None
        self.stop_until = None
        self.center_locked = False
        self.position_stable_since = None
        self.get_logger().info(
            f'等待 {value("slope_topic")}={self.trigger_value}；'
            f'中心范围 [{self.alignment.left_ratio:.0%}, '
            f'{self.alignment.right_ratio:.0%}]；'
            f'横移速度上限 {self.alignment.max_speed:.3f} m/s，待机不打开串口')
        self.get_logger().info(
            f'速度发送频率 {self.control_rate:.1f} Hz '
            f'（{1000.0 / self.control_rate:.1f} ms/帧）；'
            f'速度 INFO {"已开启" if self.log_velocity else "已关闭"}')
        if self.alignment.enable_forward:
            self.get_logger().info(
                f'前后控制已启用：先左右对中，再调节框中心 cy 到画面高度的 '
                f'{self.alignment.top_ratio:.0%}～'
                f'{self.alignment.bottom_ratio:.0%}；'
                f'前后速度上限 {self.alignment.max_forward_speed:.3f} m/s')
        if self.lock_on_center:
            self.get_logger().info(
                f'已开启到位锁车：连续达标 {self.lock_stable_duration:.2f} 秒后锁定，'
                f'再连续发送 {self.lock_stop_duration:.2f} 秒零速度后释放串口')

    def slope_callback(self, message):
        """处理 /slope 消息，默认收到 3 时开始一轮 YOLO 平移追踪。

        先打开串口并发零速度，再准备异步调用检测服务；忽略运行中的重复触发。
        """
        if message.slope != self.trigger_value:
            return
        if (self.alignment.active or self.stop_until is not None
                or (self.request_future is not None and not self.request_future.done())):
            self.get_logger().warn('上一轮尚未结束，忽略重复触发')
            return
        try:
            self.writer.open()
            self.last_velocity_log_time = None
            self.send_velocity(0.0, state='触发停车，等待 YOLO 服务')
        except Exception as exc:
            self.serial_failure(exc)
            return
        # 非零编号贯穿服务和检测话题，避免其他客户端的检测驱动车辆。
        self.alignment.start(time.time_ns())
        self.center_locked = False
        self.position_stable_since = None
        self.request_future = None
        self.service_deadline = time.monotonic() + self.service_wait_timeout
        self.get_logger().info('收到触发，已停车，等待 YOLO 服务就绪')

    def send_velocity(self, vy, *, vx=0.0, state=None):
        """串口完整写入后记录指令；INFO 最多与控制频率相同，不代表实测车速。"""
        if vx != 0.0:
            self.writer.send(vy, vx=vx)
        else:
            self.writer.send(vy)
        if not self.log_velocity:
            return
        now = time.monotonic()
        if (self.last_velocity_log_time is not None
                and now - self.last_velocity_log_time < 1.0 / self.control_rate - 1e-9):
            return
        if state is None:
            if self.alignment.last_received is None:
                state = '等待检测，停车'
            elif now - self.alignment.last_received > self.alignment.detection_timeout:
                state = '检测超时，停车'
            elif self.alignment.error is None:
                state = '无有效检测，停车'
            elif vy != 0.0:
                state = '横移对中'
            elif vx != 0.0:
                state = '前后调整'
            elif self.position_stable_since is not None:
                elapsed = max(0.0, now - self.position_stable_since)
                state = f'稳定确认 {elapsed:.2f}/{self.lock_stable_duration:.2f} s'
            else:
                state = '到位停车'
        # 与速度帧的 mm/s 向零取整保持一致，显示实际写入的指令值。
        wire_vx, wire_vy = int(vx * 1000) / 1000.0, int(vy * 1000) / 1000.0
        self.get_logger().info(
            f'串口速度指令已写入 vx={wire_vx:+.3f} m/s，vy={wire_vy:+.3f} m/s，'
            f'vw=0.000 rad/s；状态={state}')
        self.last_velocity_log_time = now

    def detection_callback(self, message):
        """接收实时检测结果，过滤过期坐标并更新左右和前后控制状态。

        达标时立即停车，锁车需连续达标指定时长；跳出、丢失或过期重新计时。
        """
        if not self.alignment.active or message.session_id != self.alignment.session_id:
            return
        now = time.monotonic()
        # 即使中间没有执行控制定时器，检测间隔过长也不能累计为稳定时间。
        if (self.alignment.last_received is not None
                and now - self.alignment.last_received > self.alignment.detection_timeout):
            self.position_stable_since = None
        # 已经过期的推理结果不能用于启动横移。
        stamp = message.stamp.sec + message.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        detected = message.detected and 0.0 <= age <= self.alignment.detection_timeout
        was_in_position = self.alignment.in_position
        self.alignment.update(
            message.session_id, detected, message.fx, message.fy,
            message.image_width, message.image_height, now)
        if not self.alignment.in_position or not self.lock_on_center:
            self.position_stable_since = None
        elif self.position_stable_since is None:
            self.position_stable_since = now
            self.get_logger().info(
                f'方块已到位，停车并开始 {self.lock_stable_duration:.2f} 秒稳定确认')
        elif now - self.position_stable_since >= self.lock_stable_duration:
            # 只由新的有效到位帧完成确认，定时器不能用静止的旧状态锁车。
            self.lock_at_center()
            return
        if not was_in_position and self.alignment.in_position:
            if not self.lock_on_center:
                self.get_logger().info('方块中心已进入左右和上下范围，发送零速度；移出后继续追踪')
            # 到位当次回调立即发零，不必等下一个控制定时器。
            try:
                self.send_velocity(0.0)
            except Exception as exc:
                self.serial_failure(exc)

    def lock_at_center(self):
        """稳定到位后停止追踪速度，连续发指定时长零速度，再由定时器关闭串口。

        检测服务继续等待方块被拿走，但其结果不再触发串口发送或延长停车时间。
        """
        self.center_locked = True
        self.alignment.finish()
        self.position_stable_since = None
        self.stop_until = time.monotonic() + self.lock_stop_duration
        self.get_logger().info(
            f'首次到位并稳定 {self.lock_stable_duration:.2f} 秒，'
            f'连续发送 {self.lock_stop_duration:.2f} 秒零速度后结束本轮串口控制')
        try:
            self.send_velocity(0.0, state='已锁定，停车')
        except Exception as exc:
            self.serial_failure(exc)

    def control_tick(self):
        """按固定频率执行控制，默认每 50 ms 调用一次。

        每轮仅发送一次 YOLO 启动请求，之后持续发送横移或前后速度。
        本轮结束或锁车模式稳定到位后，连续发送对应时长的零速度，再关闭串口。
        串口释放后不再发送速度；检测服务可以继续等待方块被拿走。
        """
        now = time.monotonic()
        if self.alignment.active:
            if (not self.alignment.in_position or self.alignment.last_received is None
                    or now - self.alignment.last_received > self.alignment.detection_timeout):
                self.position_stable_since = None
            if self.request_future is None:
                if self.client.service_is_ready():
                    request = CameraUse.Request()
                    request.use = 1
                    request.session_id = self.alignment.session_id
                    try:
                        self.request_future = self.client.call_async(request)
                        self.request_future.add_done_callback(self.service_done)
                    except Exception as exc:
                        self.finish_round(False, f'调用 YOLO 服务失败: {exc}')
                elif now >= self.service_deadline:
                    self.finish_round(False, '等待 YOLO 服务超时')
            if self.alignment.active:
                try:
                    vy = self.alignment.velocity(now)
                    vx = self.alignment.forward_velocity(now)
                    self.send_velocity(vy, vx=vx)
                except Exception as exc:
                    self.serial_failure(exc)
        elif self.stop_until is not None:
            try:
                self.send_velocity(
                    0.0, state='已锁定，停车' if self.center_locked else '本轮结束，停车')
                if now >= self.stop_until:
                    self.writer.close()
                    self.stop_until = None
                    self.get_logger().info('停车帧发送完毕，已释放串口，等待下一次触发')
            except Exception as exc:
                self.serial_failure(exc)

    def service_done(self, future):
        """处理 YOLO 异步服务的最终响应，收到 success=1 时确认本轮识别完成。

        未锁车时，返回 0、其他状态或异常也会进入停车流程。
        已首次到位锁车时只记录检测结果，不重发速度，也不重置停车计时。
        """
        if future is not self.request_future:
            return
        if not self.alignment.active and not self.center_locked:
            return
        try:
            response = future.result()
            success = response is not None and response.success == 1
        except Exception as exc:
            if self.center_locked:
                self.get_logger().warn(f'YOLO 服务异常: {exc}；已首次到位锁车，不重新接管串口')
                return
            self.finish_round(False, f'YOLO 服务异常: {exc}')
            return
        if self.center_locked:
            (self.get_logger().info if success else self.get_logger().warn)(
                f'YOLO 检测结束，完成状态为 {int(success)}；已首次到位锁车，不重新接管串口')
            return
        self.finish_round(
            success, 'YOLO 返回 1，方块已消失，本轮完成' if success
            else 'YOLO 未返回完成状态 1，本轮未成功，停车')

    def finish_round(self, success, message):
        """结束当前追踪、记录结果，并立即发零速度、启动连续停车计时。"""
        self.alignment.finish()
        self.position_stable_since = None
        self.stop_until = time.monotonic() + self.stop_duration
        (self.get_logger().info if success else self.get_logger().warn)(message)
        try:
            self.send_velocity(0.0, state='本轮结束，停车')
        except Exception as exc:
            self.serial_failure(exc)

    def serial_failure(self, exc):
        """串口发生异常时终止本轮控制，并尝试发送停车帧和关闭串口。"""
        self.alignment.finish()
        self.position_stable_since = None
        self.stop_until = None
        self.get_logger().error(f'串口通信失败，停止本轮控制: {exc}')
        try:
            self.writer.close()
        except Exception as close_error:
            self.get_logger().error(f'发送停车帧或关闭串口失败: {close_error}')

    def destroy_node(self):
        """节点退出时尝试发送停车帧、释放串口，再销毁 ROS 节点资源。"""
        self.alignment.finish()
        self.position_stable_since = None
        try:
            self.writer.close()
        except Exception as exc:
            self.get_logger().error(f'退出时发送停车帧失败: {exc}')
        return super().destroy_node()


def main(args=None):
    """初始化 ROS 并运行控制节点，程序退出时执行停车和资源清理。"""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
