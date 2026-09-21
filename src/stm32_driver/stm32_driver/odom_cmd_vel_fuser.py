#!/usr/bin/env python3
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class OdomCmdVelFuser(Node):
    def __init__(self):
        super().__init__("odom_cmd_vel_fuser")

        self.declare_parameter("odom_in_topic", "/Odometry")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_out_topic", "/nav2_odom")

        self.odom_in_topic = self.get_parameter("odom_in_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_out_topic = self.get_parameter("odom_out_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.last_cmd_vel = Twist()

        self.odom_pub = self.create_publisher(Odometry, self.odom_out_topic, qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_in_topic,
            self.odom_callback,
            qos,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos,
        )

        self.get_logger().info(
            "Publishing fused odom: "
            f"{self.odom_in_topic} + {self.cmd_vel_topic} -> {self.odom_out_topic}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = deepcopy(msg)

    def odom_callback(self, msg: Odometry):
        fused_odom = deepcopy(msg)
        fused_odom.twist.twist = deepcopy(self.last_cmd_vel)
        self.odom_pub.publish(fused_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCmdVelFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
