"""ROS integration tests run in an isolated domain, without robot hardware."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
import yaml


ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location(
    'gripper_filter_ros_under_test', ROOT / 'scripts' / 'gripper_pointcloud_filter.py')
FILTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = FILTER
SPEC.loader.exec_module(FILTER)


@pytest.fixture
def ros_graph(monkeypatch):
    # Robot DDS profiles can pin discovery to a remote network interface.
    # Keep these hardware-free tests on loopback in their own ROS domain.
    monkeypatch.setenv('CYCLONEDDS_URI',
                       '<CycloneDDS><Domain id="any"><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces>'
                       '<AllowMulticast>false</AllowMulticast></General>'
                       '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
                       '<Peers><Peer address="127.0.0.1"/></Peers></Discovery>'
                       '</Domain></CycloneDDS>')
    domain = 100 + os.getpid() % 80
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    helper = Node('gripper_filter_test', context=context)
    executor.add_node(helper)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        yield context, executor, helper, domain
    finally:
        executor.shutdown(timeout_sec=5)
        worker.join(timeout=5)
        helper.destroy_node()
        context.shutdown()


def wait_until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(0.025)
    raise AssertionError('Timed out waiting for isolated ROS test graph')


def test_tf_arriving_during_cloud_callback_is_processed(ros_graph, monkeypatch):
    context, executor, helper, _domain = ros_graph
    node = FILTER.GripperPointCloudFilter(context=context, parameter_overrides=[
        Parameter('input_topic', value='/test/raw_cloud'),
        Parameter('output_topic', value='/test/filtered_cloud'),
        Parameter('box_count', value=1),
        Parameter('box_frames', value=['test_gripper']),
        Parameter('box_min', value=[0.0, 0.0, 0.0]),
        Parameter('box_max', value=[1.0, 1.0, 1.0]),
        Parameter('tf_timeout', value=1.5),
    ])
    executor.add_node(node)
    assert node.state == FILTER.EMPTY
    assert not node.block_filter_enabled and not node.state_input_enabled
    lookup_started = threading.Event()
    original_lookup = FILTER.lookup_transform_at_stamp

    def lookup(*args, **kwargs):
        lookup_started.set()
        return original_lookup(*args, **kwargs)

    monkeypatch.setattr(FILTER, 'lookup_transform_at_stamp', lookup)
    received = []
    helper.create_subscription(
        PointCloud2, '/test/filtered_cloud', received.append, qos_profile_sensor_data)
    publisher = helper.create_publisher(PointCloud2, '/test/raw_cloud', 10)
    broadcaster = StaticTransformBroadcaster(helper)
    transform = TransformStamped()
    transform.header.frame_id = 'test_odom'
    transform.child_frame_id = 'test_gripper'
    transform.transform.rotation.w = 1.0
    sent = threading.Event()

    def send_delayed_tf():
        if lookup_started.is_set() and not sent.is_set():
            broadcaster.sendTransform(transform)
            sent.set()

    timer = helper.create_timer(0.05, send_delayed_tf)
    try:
        wait_until(lambda: publisher.get_subscription_count() > 0)
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=helper.get_clock().now().to_msg(), frame_id='test_odom'),
            [(0.5, 0.5, 0.5), (1.01, 0.5, 0.5)])
        publisher.publish(cloud)
        wait_until(lambda: bool(received))
        assert sent.is_set()
        assert received[0].width == 1
        assert received[0].header == cloud.header
        xyz, _ = FILTER.cloud_records(received[0])
        assert xyz[0, 0] == pytest.approx(1.01)
    finally:
        helper.destroy_timer(timer)
        executor.remove_node(node)
        node.destroy_node()


def test_real_nav2_keeps_obstacle_behind_self_occlusion(ros_graph, tmp_path):
    _context, _executor, helper, domain = ros_graph
    source = yaml.safe_load((ROOT / 'config/reality/nav2_params_forest_small_teb_mppi.yaml').read_text())
    layer = source['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
    # Use the production source policy and the installed Nav2 implementation.
    layer['observation_sources'] = 'gripper_marking gripper_clearing'
    layer['footprint_clearing_enabled'] = False
    for name in ('gripper_marking', 'gripper_clearing'):
        layer[name]['topic'] = '/test/' + name
        layer[name]['observation_persistence'] = 0.0
    config = {'costmap': {'costmap': {'ros__parameters': {
        'global_frame': 'test_base', 'robot_base_frame': 'test_base',
        'width': 10, 'height': 10, 'origin_x': -1.0, 'origin_y': -5.0,
        'resolution': 0.1, 'rolling_window': False, 'robot_radius': 0.1,
        'update_frequency': 20.0, 'publish_frequency': 20.0,
        'always_send_full_costmap': True, 'plugins': ['obstacle_layer'],
        'obstacle_layer': layer,
    }}}}
    params = tmp_path / 'costmap.yaml'
    params.write_text(yaml.safe_dump(config))
    executable = Path(get_package_prefix('nav2_costmap_2d')) / 'lib/nav2_costmap_2d/nav2_costmap_2d'
    maps = []
    helper.create_subscription(OccupancyGrid, '/costmap/costmap', maps.append,
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    marking = helper.create_publisher(LaserScan, '/test/gripper_marking', 10)
    clearing = helper.create_publisher(LaserScan, '/test/gripper_clearing', 10)
    log_path = tmp_path / 'costmap.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            [str(executable), '--ros-args', '--params-file', str(params)],
            env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log, stderr=log)
        try:
            client = helper.create_client(ChangeState, '/costmap/costmap/change_state')
            assert client.wait_for_service(timeout_sec=8), log_path.read_text()
            for transition in (1, 3):  # configure, activate
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                wait_until(future.done)
                assert future.result().success, log_path.read_text()
            wait_until(lambda: marking.get_subscription_count() and clearing.get_subscription_count())

            def scan(distance):
                message = LaserScan()
                message.header.frame_id = 'test_base'
                message.header.stamp = helper.get_clock().now().to_msg()
                message.angle_min = -0.01
                message.angle_max = 0.01
                message.angle_increment = 0.01
                message.range_min = 0.1
                message.range_max = 8.0
                message.ranges = [float('inf'), distance, float('inf')]
                return message

            def obstacle_cost():
                if not maps:
                    return None
                grid = maps[-1]
                x = int((3.0 - grid.info.origin.position.x) / grid.info.resolution)
                y = int((0.0 - grid.info.origin.position.y) / grid.info.resolution)
                # OccupancyGrid resolution is float32; inspect the small
                # neighborhood around the exact cell boundary at x=3, y=0.
                return max(grid.data[j * grid.info.width + i]
                           for i in range(x - 1, x + 2)
                           for j in range(y - 1, y + 2))

            marking.publish(scan(3.0))
            wait_until(lambda: obstacle_cost() == 100)
            for raw_distance in (1.0, float('inf')):
                # Deleted self points leave +Inf in the marking-only scan;
                # raw clearing sees either the gripper or no valid return.
                previous_count = len(maps)
                marking.publish(scan(float('inf')))
                clearing.publish(scan(raw_distance))
                wait_until(lambda: len(maps) >= previous_count + 5)
                assert obstacle_cost() == 100
            # Control: a real finite return beyond the obstacle clears it.
            clearing.publish(scan(5.0))
            wait_until(lambda: obstacle_cost() == 0)
            request = ChangeState.Request()
            request.transition.id = 4  # deactivate before process shutdown
            future = client.call_async(request)
            wait_until(future.done)
            assert future.result().success
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
    assert process.returncode == 0, log_path.read_text()
