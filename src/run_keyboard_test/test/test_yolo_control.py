"""验证视觉平移、停车和串口帧；全部使用模拟串口，不驱动实车。"""

import asyncio
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
from unittest.mock import Mock, call

import numpy as np
import pytest
from rclpy.task import Future
from rclpy.serialization import deserialize_message, serialize_message

from run_keyboard_test.serial_velocity import (
    SerialVelocity, crc8, encode_lateral_velocity, encode_velocity,
)
from run_keyboard_test.yolo_control import BoxAlignment, LateralAlignment, YoloControlNode
from yolo_detect_box.msg import BoxDetection
from yolo_detect_box.srv import CameraUse


def detection(control, fx=100.0, detected=True, session_id=123, now=10.0):
    control.update(session_id, detected, fx, 200.0, 1080, 720, now)


def test_pixel_band_boundaries_stop_and_leaving_band_resumes_tracking():
    for fx in (432.0, 540.0, 648.0):
        control = LateralAlignment()
        control.start(123)
        detection(control, fx=fx)
        assert control.in_center
        assert control.velocity(10.0) == 0.0
        # 进入范围停车后，向任一侧移出都必须恢复追踪。
        detection(control, fx=900.0, now=10.1)
        assert not control.in_center
        assert control.velocity(10.1) < 0.0
        detection(control, fx=fx, now=10.2)
        assert control.velocity(10.2) == 0.0
        detection(control, fx=100.0, now=10.3)
        assert not control.in_center
        assert control.velocity(10.3) > 0.0
        detection(control, detected=False, now=10.4)
        assert control.velocity(10.4) == 0.0
        detection(control, fx=100.0, now=10.5)
        assert control.velocity(10.5) > 0.0
        assert control.velocity(20.0) == 0.0
        control.finish()
        control.start(124)
        detection(control, fx=900.0, session_id=124, now=30.0)
        assert not control.in_center
        assert control.velocity(30.0) < 0.0


def test_correct_lateral_direction_and_no_motion_before_detection():
    control = LateralAlignment()
    control.start(123)
    assert control.velocity(10.0) == 0.0
    detection(control, fx=431.0)
    assert control.velocity(10.0) == pytest.approx(109.0 / 1080.0)
    detection(control, fx=649.0)
    assert control.velocity(10.0) == pytest.approx(-109.0 / 1080.0)
    mirrored = LateralAlignment(direction=1.0)
    mirrored.start(123)
    detection(mirrored, fx=649.0)
    assert mirrored.velocity(10.0) > 0.0


def test_loss_staleness_invalid_values_and_foreign_sessions_stop_motion():
    control = LateralAlignment()
    control.start(123)
    detection(control, session_id=999)
    assert control.velocity(10.0) == 0.0
    detection(control)
    assert control.velocity(10.0) > 0.0
    assert control.velocity(10.6) == 0.0
    detection(control, detected=False, now=11.0)
    assert control.velocity(11.0) == 0.0
    for fx in (float('nan'), float('inf'), -1.0, 1080.0):
        detection(control, fx=fx, now=12.0)
        assert control.velocity(12.0) == 0.0
    assert control.active  # 故障或丢失停车不等于服务成功完成。


def test_speed_hard_cap_applies_to_control_and_wire_protocol():
    control = LateralAlignment(max_speed=10.0, gain=100.0)
    control.start(123)
    detection(control, fx=0.0)
    assert control.velocity(10.0) == 0.3
    for requested, expected in ((10.0, 300), (-10.0, -300), (0.0, 0)):
        frame = encode_lateral_velocity(requested, max_speed=10.0)
        assert len(frame) == 12
        assert struct.unpack('<BhhhBBBBB', frame) == (
            0xA5, 0, expected, 0, 0, 0, 0, crc8(frame[1:10]), 0xFF)
    assert crc8(b'123456789') == 0xF4  # CRC-8 标准校验向量。
    assert encode_lateral_velocity(0.03, max_speed=0.01)[3:5] == b'\x0a\x00'
    for requested in (float('nan'), float('inf')):
        with pytest.raises(ValueError):
            encode_lateral_velocity(requested)


def test_default_speed_scales_with_error_and_can_reach_point_three():
    control = LateralAlignment()
    control.start(123)
    for fx, expected in (
            (100.0, 0.3), (350.0, 190.0 / 1080.0), (432.0, 0.0),
            (540.0, 0.0), (648.0, 0.0), (730.0, -190.0 / 1080.0),
            (900.0, -0.3)):
        detection(control, fx=fx)
        assert control.velocity(10.0) == pytest.approx(expected)
    # 降低唯一的速度上限后，比例控制和串口都使用同一个值。
    slower = LateralAlignment(max_speed=0.12)
    slower.start(123)
    detection(slower, fx=100.0)
    assert slower.velocity(10.0) == 0.12
    frame = encode_lateral_velocity(slower.velocity(10.0), max_speed=slower.max_speed)
    assert struct.unpack_from('<h', frame, 3)[0] == 120


@pytest.mark.parametrize('vx, vy', [(0.1, -0.2), (-0.1, 0.2), (10.0, -10.0)])
def test_two_axis_serial_frame_sign_limits_and_crc(vx, vy):
    frame = encode_velocity(vx, vy, max_speed=0.2, max_forward_speed=0.1)
    assert struct.unpack('<BhhhBBBBB', frame) == (
        0xA5, int(max(-0.1, min(0.1, vx)) * 1000),
        int(max(-0.2, min(0.2, vy)) * 1000), 0, 0, 0, 0,
        crc8(frame[1:10]), 0xFF)
    capped = encode_velocity(10.0, -10.0, max_speed=10.0, max_forward_speed=10.0)
    assert struct.unpack_from('<hh', capped, 1) == (300, -300)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_two_axis_serial_rejects_nonfinite_velocity(bad):
    with pytest.raises(ValueError):
        encode_velocity(bad, 0.0)
    with pytest.raises(ValueError):
        encode_velocity(0.0, bad)


def box_detection(control, fy_ratio=0.6, fx=540.0, now=10.0,
                  session_id=123, detected=True):
    """按画面高度比例设置框中心 cy。"""
    control.update(session_id, detected, fx, fy_ratio * 720.0, 1080, 720, now)


@pytest.mark.parametrize('fy_ratio', [0.5, 0.6, 0.7])
def test_position_requires_horizontal_and_vertical_center_including_boundaries(fy_ratio):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, fy_ratio=fy_ratio)
    assert control.in_position
    assert control.velocity(10.0) == control.forward_velocity(10.0) == 0.0
    box_detection(control, fy_ratio=fy_ratio, fx=100.0)
    assert control.in_vertical
    assert not control.in_position
    assert control.velocity(10.0) > 0.0
    assert control.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('fy_ratio, expected', [
    (0.0, 0.3), (0.35, 0.25), (0.49, 0.11),
    (0.71, -0.11), (0.85, -0.25), (0.95, -0.3),
])
def test_forward_direction_error_scaling_and_lateral_priority(fy_ratio, expected):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, fy_ratio=fy_ratio, fx=100.0)
    assert control.velocity(10.0) > 0.0
    assert control.forward_velocity(10.0) == 0.0
    box_detection(control, fy_ratio=fy_ratio)
    assert control.velocity(10.0) == 0.0
    assert control.forward_velocity(10.0) == pytest.approx(expected)
    assert not control.in_position


def test_vertical_center_band_can_include_image_edges():
    control = BoxAlignment(top_ratio=0.0, bottom_ratio=1.0)
    control.start(123)
    for fy_ratio in (0.0, 0.5, 0.999):
        box_detection(control, fy_ratio=fy_ratio)
        assert control.in_position
        assert control.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('height', [480, 720, 1080])
def test_custom_vertical_center_band_uses_actual_image_height(height):
    control = BoxAlignment(top_ratio=0.6, bottom_ratio=0.7)
    control.start(123)
    for ratio, expected in ((0.45, 0.2), (0.6, 0.0), (0.65, 0.0),
                            (0.7, 0.0), (0.85, -0.2)):
        control.update(123, True, 320.0, ratio * height, 640, height, 10.0)
        assert control.forward_velocity(10.0) == pytest.approx(expected)
        assert control.in_position is (expected == 0.0)


@pytest.mark.parametrize('fy', [360.0, 504.0])
def test_default_cy_boundaries_remain_inclusive_after_ros_serialization(fy):
    message = BoxDetection()
    message.session_id, message.detected = 123, True
    message.fx, message.fy = 540.0, fy
    message.image_width, message.image_height = 1080, 720
    received = deserialize_message(serialize_message(message), BoxDetection)
    control = BoxAlignment()
    control.start(123)
    control.update(received.session_id, received.detected, received.fx, received.fy,
                   received.image_width, received.image_height, 100.0)
    assert control.in_position
    assert control.forward_velocity(100.0) == 0.0


def test_forward_loss_staleness_sessions_finish_and_restart():
    control = BoxAlignment()
    control.start(123)
    assert control.forward_velocity(10.0) == 0.0
    box_detection(control, fy_ratio=0.2, session_id=999)
    assert control.forward_velocity(10.0) == 0.0
    box_detection(control, fy_ratio=0.2)
    assert control.forward_velocity(10.0) > 0.0
    assert control.forward_velocity(10.6) == 0.0
    box_detection(control, detected=False)
    assert control.velocity(10.0) == control.forward_velocity(10.0) == 0.0
    box_detection(control, fy_ratio=0.2)
    box_detection(control, session_id=999, detected=False)
    assert control.forward_velocity(10.0) > 0.0  # 外部轮次不能清除本轮状态。
    control.finish()
    assert control.forward_velocity(10.0) == 0.0
    control.start(124)
    assert not control.in_position
    assert control.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('fy, width, height', [
    (float('nan'), 1080, 720), (float('inf'), 1080, 720),
    (-1.0, 1080, 720), (720.0, 1080, 720),
    (360.0, 0, 720), (360.0, 1080, 0),
])
def test_invalid_vertical_centers_or_image_sizes_stop_both_axes(fy, width, height):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, fy_ratio=0.2)
    control.update(123, True, 100.0, fy, width, height, 10.1)
    assert not control.in_position
    assert control.velocity(10.1) == control.forward_velocity(10.1) == 0.0


def test_forward_speed_cap_reversal_and_disabled_compatibility():
    control = BoxAlignment(max_forward_speed=10.0, forward_gain=100.0,
                           forward_direction=-1.0)
    control.start(123)
    box_detection(control, fy_ratio=0.2)
    assert control.forward_velocity(10.0) == -0.3
    box_detection(control, fy_ratio=0.8)
    assert control.forward_velocity(10.0) == 0.3
    slower = BoxAlignment(max_forward_speed=0.1, forward_gain=100.0)
    slower.start(123)
    box_detection(slower, fy_ratio=0.2)
    assert slower.forward_velocity(10.0) == 0.1
    lateral_only = BoxAlignment(enable_forward=False)
    lateral_only.start(123)
    detection(lateral_only, fx=540.0)  # 关闭前后控制后，cy 不限制到位。
    assert lateral_only.in_position
    assert lateral_only.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('options', [
    {'forward_gain': 0.0}, {'forward_gain': float('nan')},
    {'forward_direction': 0.0}, {'forward_direction': float('inf')},
    {'max_forward_speed': -1.0}, {'max_forward_speed': float('inf')},
    {'top_ratio': -0.1}, {'top_ratio': 1.0},
    {'bottom_ratio': 1.1},
    {'top_ratio': 0.8, 'bottom_ratio': 0.8},
    {'bottom_ratio': float('nan')},
])
def test_invalid_forward_configuration_is_rejected(options):
    with pytest.raises(ValueError):
        BoxAlignment(**options)


class FakeSerial:
    def __init__(self, **kwargs):
        self.options = kwargs
        self.frames = []
        self.closed = False

    def write(self, frame):
        self.frames.append(frame)
        return len(frame)

    def close(self):
        self.closed = True


def test_serial_lifecycle_finishes_with_a_zero_frame():
    connection = FakeSerial()
    factory = Mock(return_value=connection)
    writer = SerialVelocity('/dev/fake', 961200, 0.3, serial_factory=factory)
    assert not writer.is_open
    factory.assert_not_called()
    writer.open()
    assert factory.call_args.kwargs['exclusive'] is True
    writer.send(-0.03)
    writer.close()
    assert connection.frames[0] == encode_lateral_velocity(-0.03)
    assert connection.frames[-1] == encode_lateral_velocity(0.0)
    assert connection.closed
    writer.close()


def test_incomplete_serial_write_is_reported_and_close_releases_connection():
    connection = FakeSerial()
    connection.write = Mock(return_value=1)
    writer = SerialVelocity('/dev/fake', 961200, 0.3, serial_factory=lambda **_: connection)
    writer.open()
    with pytest.raises(OSError):
        writer.send(0.03)
    with pytest.raises(OSError):
        writer.close()
    assert not writer.is_open
    assert connection.closed


def test_serial_close_stops_both_axes_after_forward_motion():
    connection = FakeSerial()
    writer = SerialVelocity('/dev/fake', 961200, 0.3,
                            serial_factory=lambda **_: connection, max_forward_speed=0.1)
    writer.open()
    writer.send(-0.2, vx=0.3)
    writer.close()
    assert struct.unpack_from('<hhh', connection.frames[0], 1) == (100, -200, 0)
    assert connection.frames[-1] == encode_velocity(0.0, 0.0)
    assert connection.closed


def controller_harness(monkeypatch):
    """只替换 ROS 通信和串口依赖，执行真实节点回调及定时器。"""
    import run_keyboard_test.yolo_control as module
    now = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    node = object.__new__(YoloControlNode)
    node.alignment = BoxAlignment(enable_forward=False)
    node.trigger_value = 3
    node.writer = Mock()
    node.client = Mock()
    node.client.service_is_ready.return_value = True
    node.client.call_async.return_value = Future()
    node.request_future = None
    node.service_deadline = None
    node.service_wait_timeout = 5.0
    node.stop_duration = 0.5
    node.stop_until = None
    node.lock_on_center = False
    node.lock_stable_duration = 1.0
    node.lock_stop_duration = 1.0
    node.center_locked = False
    node.position_stable_since = None
    node.control_rate = 20.0
    node.log_velocity = True
    node.last_velocity_log_time = None
    node.get_logger = Mock(return_value=Mock())
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(now[0] * 1e9))))
    return node, now


@pytest.mark.parametrize('status', [1, 0, 2])
@pytest.mark.parametrize('lock_on_center', [False, True])
def test_slope_trigger_service_result_and_repeated_stop_frames(
        monkeypatch, status, lock_on_center):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = lock_on_center
    for value in (1, 2):
        node.slope_callback(SimpleNamespace(slope=value))
    node.writer.open.assert_not_called()
    node.slope_callback(SimpleNamespace(slope=3))
    node.control_tick()
    request = node.client.call_async.call_args.args[0]
    assert request.use == 1
    assert request.session_id == node.alignment.session_id != 0
    node.slope_callback(SimpleNamespace(slope=3))
    node.writer.open.assert_called_once()
    node.client.call_async.assert_called_once()
    detection(node.alignment, session_id=request.session_id, now=now[0])
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    response = CameraUse.Response()
    response.success = status
    node.finish_round = Mock(wraps=node.finish_round)
    node.request_future.set_result(response)
    node.service_done(node.request_future)
    assert node.finish_round.call_args.args[0] is (status == 1)
    assert not node.alignment.active
    node.writer.send.assert_called_with(0.0)
    node.writer.reset_mock()
    for offset in (0.1, 0.2, 0.5):
        now[0] = 100.0 + offset
        node.control_tick()
    assert node.writer.send.call_args_list == [call(0.0)] * 3
    node.writer.close.assert_called_once()
    assert node.stop_until is None


@pytest.mark.parametrize('status', [0, 1])
def test_service_integer_status_survives_ros_serialization(status):
    assert CameraUse.Response.get_fields_and_field_types()['success'] == 'int32'
    response = CameraUse.Response()
    response.success = status
    response.fx, response.fy = 200.0, 300.0
    received = deserialize_message(serialize_message(response), CameraUse.Response)
    assert type(received.success) is int
    assert received.success == status
    assert (received.fx, received.fy) == (200.0, 300.0)


def test_center_stops_immediately_and_leaving_band_resumes_serial_commands(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.alignment.start(123)
    message = BoxDetection()
    message.session_id = 123
    message.detected = True
    message.image_width = 1080
    message.image_height = 720
    message.fx, message.fy = 432.0, 200.0
    message.stamp.sec = 100
    node.detection_callback(message)
    node.writer.send.assert_called_with(0.0)
    node.request_future = Future()
    for offset in (0.05, 1.0, 10.0):
        now[0] = 100.0 + offset
        node.control_tick()
        node.writer.send.assert_called_with(0.0)
    assert node.alignment.active
    # 保持同一轮服务，更新像素坐标后应直接恢复横移，不必重新触发 slope。
    for fx, expected_sign in ((900.0, -1), (648.0, 0), (100.0, 1), (432.0, 0)):
        now[0] += 0.05
        message.fx = fx
        message.stamp.sec = int(now[0])
        message.stamp.nanosec = int((now[0] % 1.0) * 1e9)
        node.detection_callback(message)
        node.control_tick()
        velocity = node.writer.send.call_args.args[0]
        if expected_sign == 0:
            assert velocity == 0.0
        else:
            assert velocity * expected_sign > 0.0
        assert node.alignment.active
        assert not node.request_future.done()


def test_unavailable_service_and_serial_failure_do_not_start_motion(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.client.service_is_ready.return_value = False
    node.slope_callback(SimpleNamespace(slope=3))
    now[0] += 5.1
    node.control_tick()
    assert not node.alignment.active
    node.client.call_async.assert_not_called()
    node.writer.send.assert_called_with(0.0)
    node, _ = controller_harness(monkeypatch)
    node.writer.open.side_effect = OSError('fake serial busy')
    node.slope_callback(SimpleNamespace(slope=3))
    assert not node.alignment.active
    node.client.call_async.assert_not_called()


def centered_detection(node, now, fx=432.0, session_id=123, detected=True, age=0.0,
                       fy_ratio=0.6, box_height_ratio=0.2):
    """构造本轮图像消息，驱动真实节点的检测回调。"""
    message = BoxDetection()
    message.session_id = session_id
    message.detected = detected
    message.image_width, message.image_height = 1080, 720
    message.x1, message.x2 = fx - 50.0, fx + 50.0
    message.fx, message.fy = fx, fy_ratio * 720.0
    message.y1 = (fy_ratio - box_height_ratio / 2.0) * 720.0
    message.y2 = (fy_ratio + box_height_ratio / 2.0) * 720.0
    stamp = now - age
    message.stamp.sec = int(stamp)
    message.stamp.nanosec = int((stamp % 1.0) * 1e9)
    node.detection_callback(message)


def confirm_stable_position(node, now, **options):
    """持续提供有效到位帧，直到满足默认一秒稳定确认。"""
    started = now[0]
    for offset in (0.0, 0.25, 0.5, 0.75, 1.0):
        now[0] = started + offset
        centered_detection(node, now[0], **options)


@pytest.mark.parametrize('lock_on_center', [False, True])
def test_full_position_controls_serial_and_locks_only_when_cx_and_cy_match(
        monkeypatch, lock_on_center):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.lock_on_center = lock_on_center
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], fx=100.0, fy_ratio=0.35)
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    assert not node.center_locked
    centered_detection(node, now[0], fy_ratio=0.35)
    node.control_tick()
    node.writer.send.assert_called_with(0.0, vx=pytest.approx(0.25))
    assert not node.center_locked
    centered_detection(node, now[0], fy_ratio=0.71)
    node.control_tick()
    node.writer.send.assert_called_with(0.0, vx=pytest.approx(-0.11))
    assert not node.center_locked
    # cy 达标但 cx 未对中仍然横移，不会提前锁定。
    centered_detection(node, now[0], fx=100.0)
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    assert not node.center_locked
    node.writer.reset_mock()
    # 两轴中心进入边界后立即停车，连续稳定一秒后才锁定。
    centered_detection(node, now[0])
    node.writer.send.assert_called_once_with(0.0)  # 到位回调立即停止两轴。
    assert not node.center_locked
    assert node.alignment.active
    if lock_on_center:
        confirm_stable_position(node, now)
        assert node.center_locked
    node.control_tick()
    node.writer.send.assert_called_with(0.0)
    if not lock_on_center:
        centered_detection(node, now[0], fy_ratio=0.71)
        node.control_tick()
        node.writer.send.assert_called_with(0.0, vx=pytest.approx(-0.11))


def test_node_stale_or_missing_boxes_stop_forward_commands(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], fy_ratio=0.35)
    node.control_tick()
    assert node.writer.send.call_args.kwargs['vx'] > 0.0
    now[0] += 0.6
    node.control_tick()
    node.writer.send.assert_called_with(0.0)
    centered_detection(node, now[0], fy_ratio=0.35)
    centered_detection(node, now[0], detected=False)
    node.control_tick()
    node.writer.send.assert_called_with(0.0)
    centered_detection(node, now[0], fy_ratio=0.35, age=1.0)
    node.control_tick()
    node.writer.send.assert_called_with(0.0)


@pytest.mark.parametrize('fy_ratio, expected', [(0.35, 0.25), (0.6, 0.0), (0.85, -0.25)])
def test_box_height_does_not_change_cy_based_control(monkeypatch, fy_ratio, expected):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.alignment.start(123)
    node.request_future = Future()
    for height_ratio in (0.05, 0.1, 0.3):
        centered_detection(node, now[0], fy_ratio=fy_ratio, box_height_ratio=height_ratio)
        node.control_tick()
        assert node.writer.send.call_args.kwargs.get('vx', 0.0) == pytest.approx(expected)
        assert node.alignment.in_position is (expected == 0.0)


def test_lock_mode_sends_zero_for_one_second_and_never_reclaims_released_serial(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], fx=100.0)
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    node.writer.reset_mock()
    confirm_stable_position(node, now)
    assert node.center_locked
    assert not node.alignment.active
    assert node.stop_until == 102.0
    node.writer.send.assert_called_with(0.0)
    for current in (101.05, 101.5, 101.95):
        now[0] = current
        centered_detection(node, now[0], fx=900.0)
        node.control_tick()
        node.writer.send.assert_called_with(0.0)
        node.writer.close.assert_not_called()
    assert all(invocation.args == (0.0,) for invocation in node.writer.send.call_args_list)
    node.slope_callback(SimpleNamespace(slope=3))
    node.writer.open.assert_not_called()
    now[0] = 102.0
    node.control_tick()
    node.writer.close.assert_called_once()
    assert node.stop_until is None
    node.writer.reset_mock()
    # 释放串口后，后续坐标、待完成服务期间的重复触发都不能产生速度帧。
    now[0] = 103.0
    centered_detection(node, now[0], fx=100.0)
    node.slope_callback(SimpleNamespace(slope=3))
    node.control_tick()
    response = CameraUse.Response()
    response.success = 1
    node.request_future.set_result(response)
    node.service_done(node.request_future)
    node.control_tick()
    node.writer.send.assert_not_called()
    node.writer.close.assert_not_called()
    node.writer.open.assert_not_called()
    # 检测结束后，只有新的外部触发才开启下一轮，并清除上轮锁定。
    node.slope_callback(SimpleNamespace(slope=3))
    node.writer.open.assert_called_once()
    assert node.alignment.active
    assert not node.center_locked


@pytest.mark.parametrize('result', [1, 0, 'exception'])
def test_early_service_result_keeps_lock_stop_deadline(monkeypatch, result):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    confirm_stable_position(node, now)
    now[0] = 101.2
    if result == 'exception':
        node.request_future.set_exception(RuntimeError('fake camera service failure'))
    else:
        response = CameraUse.Response()
        response.success = result
        node.request_future.set_result(response)
    node.service_done(node.request_future)
    assert node.stop_until == 102.0
    now[0] = 101.95
    node.control_tick()
    node.writer.close.assert_not_called()
    node.writer.send.assert_called_with(0.0)
    now[0] = 102.0
    node.control_tick()
    node.writer.close.assert_called_once()
    node.writer.reset_mock()
    now[0] = 102.5
    node.control_tick()
    node.writer.send.assert_not_called()


def test_lock_mode_ignores_missing_stale_and_foreign_center_detections(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    for options in ({'detected': False}, {'age': 1.0}, {'session_id': 999}, {'fx': 100.0}):
        centered_detection(node, now[0], **options)
        assert not node.center_locked
        assert node.alignment.active
        assert node.stop_until is None


def test_single_frame_cy_jump_does_not_lock_and_forward_motion_resumes(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    for offset, ratio in ((0.0, 0.35), (0.1, 0.6), (0.15, 0.4)):
        now[0] = 100.0 + offset
        centered_detection(node, now[0], fy_ratio=ratio)
        node.control_tick()
        assert node.alignment.active
        assert not node.center_locked
        if ratio == 0.6:
            node.writer.send.assert_called_with(0.0)
        else:
            assert node.writer.send.call_args.kwargs['vx'] > 0.0
    assert node.position_stable_since is None
    assert not node.request_future.done()
    node.writer.close.assert_not_called()


@pytest.mark.parametrize('duration', [1.0, 2.0])
def test_lock_requires_continuous_fresh_position_frames_for_configured_duration(
        monkeypatch, duration):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.lock_on_center = True
    node.lock_stable_duration = duration
    node.alignment.start(123)
    node.request_future = Future()
    for fraction in (0.0, 0.25, 0.5, 0.75, 0.99):
        now[0] = 100.0 + duration * fraction
        centered_detection(node, now[0])
        node.control_tick()
        assert not node.center_locked
        assert node.alignment.active
        assert node.stop_until is None
        node.writer.send.assert_called_with(0.0)
    now[0] = 100.0 + duration
    centered_detection(node, now[0])
    assert node.center_locked
    assert not node.alignment.active
    assert node.stop_until == now[0] + node.lock_stop_duration


@pytest.mark.parametrize('interruption', [
    {'fx': 900.0}, {'fy_ratio': 0.35}, {'fy_ratio': 0.75},
    {'detected': False}, {'age': 1.0}, {'fy_ratio': float('nan')},
])
def test_stability_restarts_after_leaving_position_loss_or_invalid_detection(
        monkeypatch, interruption):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0])
    now[0] = 100.25
    centered_detection(node, now[0])
    now[0] = 100.5
    centered_detection(node, now[0], **interruption)
    assert node.position_stable_since is None
    assert not node.center_locked
    for current in (101.0, 101.25, 101.5, 101.75, 101.99):
        now[0] = current
        centered_detection(node, now[0])
        assert not node.center_locked
        assert node.position_stable_since == 101.0
    now[0] = 102.0
    centered_detection(node, now[0])
    assert node.center_locked


@pytest.mark.parametrize('run_timer', [False, True])
def test_detection_gap_restarts_stability_even_without_control_timer(monkeypatch, run_timer):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0])
    now[0] = 100.25
    centered_detection(node, now[0])
    if run_timer:
        now[0] = 100.8
        node.control_tick()
        assert node.position_stable_since is None
        node.writer.send.assert_called_with(0.0)
    now[0] = 101.25
    centered_detection(node, now[0])
    assert not node.center_locked
    assert node.position_stable_since == 101.25
    confirm_stable_position(node, now)
    assert node.center_locked


def test_control_timer_cannot_lock_from_one_frame_even_with_long_detection_timeout(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.detection_timeout = 2.0
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0])
    now[0] = 101.1
    node.control_tick()
    assert not node.center_locked
    assert node.alignment.active
    assert node.stop_until is None
    now[0] = 102.1
    node.control_tick()
    assert node.position_stable_since is None
    assert not node.center_locked


def test_foreign_detection_does_not_change_pending_stability_confirmation(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0])
    now[0] = 100.25
    centered_detection(node, now[0])
    now[0] = 100.5
    centered_detection(node, now[0], session_id=999, detected=False)
    assert node.position_stable_since == 100.0
    assert node.alignment.last_received == 100.25
    for current in (100.75, 100.99):
        now[0] = current
        centered_detection(node, now[0])
        assert not node.center_locked
    now[0] = 101.0
    centered_detection(node, now[0])
    assert node.center_locked


def test_new_round_does_not_inherit_unfinished_stability_confirmation(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    centered_detection(node, now[0])
    assert node.position_stable_since == 100.0
    node.finish_round(False, '模拟本轮异常结束')
    assert node.position_stable_since is None
    now[0] = 101.0
    node.control_tick()
    node.slope_callback(SimpleNamespace(slope=3))
    centered_detection(node, now[0], session_id=node.alignment.session_id)
    assert node.position_stable_since == 101.0
    assert not node.center_locked
    assert node.alignment.active


def test_velocity_info_matches_wire_values_and_logs_at_twenty_hz(monkeypatch):
    node, now = controller_harness(monkeypatch)
    connection = FakeSerial()
    node.writer = SerialVelocity('/dev/fake', 961200, 0.3,
                                 serial_factory=lambda **_: connection)
    node.writer.open()
    node.send_velocity(0.12399, vx=-0.07599, state='测试')
    assert struct.unpack_from('<hh', connection.frames[-1], 1) == (-75, 123)
    message = node.get_logger().info.call_args.args[0]
    assert 'vx=-0.075 m/s' in message
    assert 'vy=+0.123 m/s' in message
    # 同一控制周期内额外的停车帧仍会写入串口，但不会重复刷速度日志。
    now[0] = 100.01
    node.send_velocity(0.0, state='停车')
    assert len(connection.frames) == 2
    assert node.get_logger().info.call_count == 1
    for tick in range(1, 21):
        now[0] = 100.0 + tick * 0.05
        node.send_velocity(0.0, state='停车')
    assert node.get_logger().info.call_count == 21
    assert len(connection.frames) == 22


def test_disabling_velocity_info_preserves_serial_commands(monkeypatch):
    node, _ = controller_harness(monkeypatch)
    node.log_velocity = False
    node.send_velocity(-0.2, vx=0.1)
    node.writer.send.assert_called_once_with(-0.2, vx=0.1)
    node.get_logger().info.assert_not_called()


def test_failed_serial_write_does_not_log_success(monkeypatch):
    node, _ = controller_harness(monkeypatch)
    node.writer.send.side_effect = OSError('模拟串口写入失败')
    with pytest.raises(OSError):
        node.send_velocity(0.0, vx=0.2)
    node.get_logger().info.assert_not_called()
    assert node.last_velocity_log_time is None


@pytest.mark.parametrize('options, state', [
    ({'fy_ratio': 0.35}, '前后调整'),
    ({'fx': 900.0}, '横移对中'),
    ({'detected': False}, '无有效检测，停车'),
    ({}, '稳定确认'),
])
def test_velocity_info_explains_control_state(monkeypatch, options, state):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], **options)
    node.control_tick()
    assert state in node.get_logger().info.call_args.args[0]
    now[0] += 0.6
    node.control_tick()
    assert '检测超时，停车' in node.get_logger().info.call_args.args[0]


@pytest.fixture
def detector_module(monkeypatch):
    # 系统 ROS Python 不必安装模型依赖，测试使用假的预测结果。
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Mock()))
    path = Path(__file__).resolve().parents[2] / 'yolo_detect_box/scripts/yolo_detect_node.py'
    specification = importlib.util.spec_from_file_location('tested_yolo_detector', path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)
    return module


def detector_harness(module):
    node = object.__new__(module.YoloDetectNode)
    node.active = True
    node.show_window = False
    node.session_id = 123
    node.seen_target = False
    node.missing_since = None
    node.read_failed_since = None
    node.missing_timeout = 1.0
    node.camera_timeout = 2.0
    node.conf = 0.6
    node.center_left_ratio, node.center_right_ratio = 0.4, 0.6
    node.center_top_ratio, node.center_bottom_ratio = 0.5, 0.7
    node.enable_forward_control = True
    node.last_fx = -1.0
    node.last_fy = -1.0
    node.finished = Future()
    node.cap = Mock()
    node.cap.read.return_value = (True, SimpleNamespace(shape=(720, 1080, 3)))
    node.model = Mock()
    node.detection_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: BoxDetection().stamp)))
    return node


@pytest.mark.parametrize('fx_ratio', [0.15, 0.5])
@pytest.mark.parametrize('fy_ratio', [0.2, 0.3, 0.5, 0.7, 0.8])
def test_overlay_draws_fixed_cy_boundaries_matching_control(detector_module, fx_ratio, fy_ratio):
    node = detector_harness(detector_module)
    node.center_top_ratio, node.center_bottom_ratio = 0.3, 0.7
    message = BoxDetection()
    message.detected = True
    message.fx, message.fy = fx_ratio * 640.0, fy_ratio * 480.0
    # 框上沿随目标移动，水平边界仍固定在画面高度的 30% 和 70%。
    message.y1, message.y2 = message.fy - 24.0, message.fy + 24.0
    frame = node.draw_debug_overlay(np.zeros((480, 640, 3), dtype=np.uint8), message)
    control = BoxAlignment(top_ratio=0.3, bottom_ratio=0.7)
    control.start(123)
    control.update(123, True, message.fx, message.fy, 640, 480, 100.0)
    green, yellow, red = (0, 255, 0), (0, 220, 255), (0, 80, 255)
    boundary_color = green if control.in_vertical else yellow
    assert tuple(frame[144, 580]) == boundary_color
    assert tuple(frame[336, 580]) == boundary_color
    assert tuple(frame[round(message.fy), round(message.fx)]) == (
        green if control.in_position else red)


def test_overlay_hides_cy_boundaries_when_forward_control_is_disabled(detector_module):
    node = detector_harness(detector_module)
    node.enable_forward_control = False
    message = BoxDetection()
    message.detected = True
    message.fx, message.fy = 320.0, 100.0
    frame = node.draw_debug_overlay(np.zeros((480, 640, 3), dtype=np.uint8), message)
    assert tuple(frame[240, 580]) == (0, 0, 0)
    assert tuple(frame[336, 580]) == (0, 0, 0)
    assert tuple(frame[100, 320]) == (0, 255, 0)


class FakeBoxes:
    def __init__(self, detected):
        self.detected = detected
        self.xyxy = [SimpleNamespace(tolist=lambda: [100.0, 200.0, 300.0, 400.0])]

    def __len__(self):
        return int(self.detected)


def test_detector_first_seen_loss_timer_and_last_pixel_center(detector_module, monkeypatch):
    module = detector_module
    node = detector_harness(module)
    node.model.predict.return_value = [SimpleNamespace(boxes=FakeBoxes(False))]
    node.process_frame()
    assert not node.finished.done()  # 首次识别前空画面不能算完成。
    node.model.predict.return_value = [SimpleNamespace(boxes=FakeBoxes(True))]
    node.process_frame()
    published = node.detection_pub.publish.call_args.args[0]
    assert (published.fx, published.fy, published.session_id) == (200.0, 300.0, 123)
    assert (published.x1, published.y1, published.x2, published.y2) == (
        100.0, 200.0, 300.0, 400.0)
    received = deserialize_message(serialize_message(published), BoxDetection)
    assert (received.x1, received.y1, received.x2, received.y2) == (
        100.0, 200.0, 300.0, 400.0)
    node.model.predict.return_value = [SimpleNamespace(boxes=FakeBoxes(False))]
    node.process_frame()
    published = node.detection_pub.publish.call_args.args[0]
    assert (published.x1, published.y1, published.x2, published.y2) == (-1.0,) * 4
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.8)
    node.process_frame()
    assert not node.finished.done()
    # 短暂重新出现必须清零丢失计时。
    node.model.predict.return_value = [SimpleNamespace(boxes=FakeBoxes(True))]
    node.process_frame()
    node.model.predict.return_value = [SimpleNamespace(boxes=FakeBoxes(False))]
    node.process_frame()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 101.9)
    node.process_frame()
    assert node.finished.result() == (1, 200.0, 300.0)
    assert type(node.finished.result()[0]) is int
    assert not node.active


def test_camera_failure_is_not_successful_target_removal(detector_module, monkeypatch):
    node = detector_harness(detector_module)
    node.seen_target = True
    node.missing_since = 90.0
    node.cap.read.return_value = (False, None)
    node.process_frame()
    assert node.missing_since is None
    assert not node.finished.done()
    monkeypatch.setattr(detector_module.time, 'monotonic', lambda: 102.0)
    node.process_frame()
    assert node.finished.result()[0] == 0
    assert type(node.finished.result()[0]) is int


def test_busy_service_does_not_start_a_second_detection(detector_module):
    node = detector_harness(detector_module)
    request = CameraUse.Request()
    request.use = 1
    request.session_id = 456
    response = asyncio.run(node.use_callback(request, CameraUse.Response()))
    assert response.success == 0
    assert type(response.success) is int
    assert response.fx == response.fy == -1.0
    assert node.session_id == 123
