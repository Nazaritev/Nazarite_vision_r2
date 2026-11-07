#!/usr/bin/env python3
"""Read-only ROS graph preflight; exit before launching duplicate publishers."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def conflict_reasons(node_names, publishers, transforms, start_lidar, start_lio, start_rsp):
    """Pure policy, also tested offline without constructing a ROS node."""
    reasons = []
    for reserved in ('/cloud_localizer', '/cloud_localization_rviz'):
        if reserved in node_names:
            reasons.append(reserved + ' already exists')
    if sum(name == '/cloud_localization_preflight' for name in node_names) > 1:
        reasons.append('another localization preflight is running')
    if any(child in ('map', 'odom') for _, child in transforms):
        reasons.append('another publisher owns a parent transform of map/odom')
    checks = []
    if start_lidar:
        checks.extend(('/livox/lidar', '/livox/imu'))
        if '/cloud_test_livox' in node_names:
            reasons.append('/cloud_test_livox already exists')
    if start_lio:
        checks.extend(('/cloud_registered', '/Odometry'))
        if '/cloud_test_fastlio' in node_names or any(
                child == 'base_link' for _, child in transforms):
            reasons.append('a FAST-LIO/base_link TF publisher already exists')
    if start_rsp:
        checks.append('/robot_description')
        if '/cloud_test_robot_state_publisher' in node_names or any(
                child in ('imu_link', 'livox_frame', 'footprint_link')
                for _, child in transforms):
            reasons.append('real robot mount TFs already exist')
    for topic in checks:
        if publishers.get(topic, 0):
            reasons.append(f'{topic} already has a publisher; use its start flag=false')
    for topic in ('/livox/lidar', '/livox/imu', '/cloud_registered', '/Odometry', '/robot_description'):
        if publishers.get(topic, 0) > 1:
            reasons.append(f'{topic} has multiple publishers')
    return reasons


def main(args=None):
    rclpy.init(args=args)
    node = Node('cloud_localization_preflight')
    starts = [node.declare_parameter(name, False).value for name in
              ('start_lidar', 'start_lio', 'start_robot_state_publisher')]
    transforms = set()

    def record(message):
        transforms.update((tf.header.frame_id, tf.child_frame_id) for tf in message.transforms)

    dynamic_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(TFMessage, '/tf', record, dynamic_qos),
                     node.create_subscription(TFMessage, '/tf_static', record, static_qos)]
    node.get_logger().info('Checking existing publishers/TF for 2 seconds; no inputs started yet')
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    names = ['/' + '/'.join((namespace.strip('/'), name)).strip('/')
             for name, namespace in node.get_node_names_and_namespaces()]
    topics = ('/livox/lidar', '/livox/imu', '/cloud_registered', '/Odometry', '/robot_description')
    publishers = {topic: node.count_publishers(topic) for topic in topics}
    reasons = conflict_reasons(names, publishers, transforms, *starts)
    for reason in reasons:
        node.get_logger().error('STARTUP_CONFLICT: ' + reason)
    if reasons:
        node.get_logger().error('No test chain started; existing processes are untouched')
    else:
        node.get_logger().info('Preflight passed; continuing with requested test nodes')
    del subscriptions
    node.destroy_node()
    rclpy.shutdown()
    return 1 if reasons else 0


if __name__ == '__main__':
    raise SystemExit(main())
