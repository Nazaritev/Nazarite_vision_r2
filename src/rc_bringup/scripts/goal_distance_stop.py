#!/usr/bin/env python3
"""Stop Nav2 after it stays within a distance threshold for a fixed time."""

import math
from typing import Optional

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from rclpy.node import Node


class GoalDistanceStop(Node):
    """Cancel an active NavigateToPose goal after a stable near-goal period."""

    def __init__(self):
        super().__init__('goal_distance_stop')

        self.declare_parameter('distance_threshold', 0.18)
        self.declare_parameter('hold_duration_sec', 1.0)
        self.declare_parameter('stop_publish_frequency', 20.0)
        self.declare_parameter('feedback_topic', '/navigate_to_pose/_action/feedback')
        self.declare_parameter('cancel_service', '/navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')

        self.distance_threshold = max(
            0.0, float(self.get_parameter('distance_threshold').value)
        )
        self.hold_duration_sec = max(
            0.0, float(self.get_parameter('hold_duration_sec').value)
        )
        publish_frequency = max(
            1.0, float(self.get_parameter('stop_publish_frequency').value)
        )
        feedback_topic = str(self.get_parameter('feedback_topic').value)
        cancel_service = str(self.get_parameter('cancel_service').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        cmd_vel_nav_topic = str(self.get_parameter('cmd_vel_nav_topic').value)

        self.stop_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.nav_stop_pub = self.create_publisher(Twist, cmd_vel_nav_topic, 10)
        self.cancel_client = self.create_client(CancelGoal, cancel_service)
        self.feedback_sub = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            feedback_topic,
            self.feedback_callback,
            10,
        )
        self.stop_timer = self.create_timer(
            1.0 / publish_frequency,
            self.stop_timer_callback,
        )

        self.active_goal_id: Optional[bytes] = None
        self.within_threshold_since: Optional[float] = None
        self.stop_latched = False
        self.cancel_in_flight = False
        self.cancel_completed = False
        self.last_cancel_attempt = 0.0

        self.get_logger().info(
            '到点停车已启用: '
            f'距离 < {self.distance_threshold:.3f}m 持续 {self.hold_duration_sec:.2f}s 后停止'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def goal_id_bytes(message: NavigateToPose_FeedbackMessage) -> bytes:
        return bytes(message.goal_id.uuid)

    def feedback_callback(self, message: NavigateToPose_FeedbackMessage) -> None:
        goal_id = self.goal_id_bytes(message)
        if goal_id != self.active_goal_id:
            self.active_goal_id = goal_id
            self.within_threshold_since = None
            self.stop_latched = False
            self.cancel_in_flight = False
            self.cancel_completed = False
            self.last_cancel_attempt = 0.0
            self.get_logger().info('检测到新的 Nav2 导航目标，重新开始到点计时')

        distance = float(message.feedback.distance_remaining)
        if not math.isfinite(distance) or distance < 0.0:
            self.within_threshold_since = None
            return

        if self.stop_latched:
            return

        now = self.now_sec()
        if distance > self.distance_threshold:
            self.within_threshold_since = None
            return

        if self.within_threshold_since is None:
            self.within_threshold_since = now
            self.get_logger().info(
                f'进入到点停车范围: distance={distance:.3f}m，开始计时'
            )
            return

        elapsed = now - self.within_threshold_since
        if elapsed < 0.0:
            self.within_threshold_since = now
            return
        if elapsed >= self.hold_duration_sec:
            self.stop_latched = True
            self.within_threshold_since = None
            self.get_logger().warn(
                f'距离 {distance:.3f}m 已持续 {elapsed:.2f}s，取消导航并停车'
            )
            self.publish_stop()
            self.try_cancel_goal()

    def try_cancel_goal(self) -> None:
        if (
            self.active_goal_id is None
            or self.cancel_in_flight
            or self.cancel_completed
        ):
            return

        now = self.now_sec()
        if now - self.last_cancel_attempt < 0.5:
            return
        self.last_cancel_attempt = now

        if not self.cancel_client.service_is_ready():
            self.get_logger().warn(
                'Nav2 取消服务尚未就绪，继续发送零速度并等待重试',
                throttle_duration_sec=2.0,
            )
            return

        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = list(self.active_goal_id)
        self.cancel_in_flight = True
        future = self.cancel_client.call_async(request)
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future) -> None:
        self.cancel_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS service state
            self.get_logger().error(f'取消 Nav2 导航失败，将继续重试: {exc}')
            return

        self.cancel_completed = True
        self.get_logger().info(
            f'Nav2 导航取消请求已处理，匹配目标数={len(response.goals_canceling)}'
        )

    def publish_stop(self) -> None:
        stop = Twist()
        self.stop_pub.publish(stop)
        self.nav_stop_pub.publish(stop)

    def stop_timer_callback(self) -> None:
        if not self.stop_latched:
            return
        self.publish_stop()
        self.try_cancel_goal()

    def destroy_node(self):
        self.publish_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalDistanceStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
