"""验证视觉平移、停车和串口帧；全部使用模拟串口，不驱动实车。"""

import asyncio
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
from unittest.mock import Mock, call

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


def box_detection(control, top=0.075, bottom=0.075, fx=540.0, now=10.0,
                  session_id=123, detected=True):
    """用占画面高度的边距比例构造一帧，确保中心和框边界一致。"""
    y1, y2 = top * 720.0, (1.0 - bottom) * 720.0
    control.update(session_id, detected, fx, (y1 + y2) / 2.0, 1080, 720, now,
                   y1=y1, y2=y2)


@pytest.mark.parametrize('top, bottom', [
    (0.05, 0.05), (0.05, 0.10), (0.10, 0.05), (0.10, 0.10), (0.075, 0.075),
])
def test_position_requires_horizontal_center_and_height_band_including_boundaries(top, bottom):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, top=top, bottom=bottom)
    assert control.in_position
    assert control.velocity(10.0) == control.forward_velocity(10.0) == 0.0
    box_detection(control, top=top, bottom=bottom, fx=100.0)
    assert control.in_size
    assert not control.in_position
    assert control.velocity(10.0) > 0.0
    assert control.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('top, bottom, expected', [
    (0.20, 0.20, 0.25), (0.075, 0.20, 0.125),
    (0.02, 0.02, -0.11), (0.02, 0.075, -0.055),
    (0.0, 0.0, -0.15),
])
def test_forward_direction_error_scaling_and_lateral_priority(top, bottom, expected):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, top=top, bottom=bottom, fx=100.0)
    assert control.velocity(10.0) > 0.0
    assert control.forward_velocity(10.0) == 0.0
    box_detection(control, top=top, bottom=bottom)
    assert control.velocity(10.0) == 0.0
    assert control.forward_velocity(10.0) == pytest.approx(expected)
    assert not control.in_position


@pytest.mark.parametrize('top, bottom, expected, in_position', [
    (0.02, 0.13, 0.0, True), (0.13, 0.02, 0.0, True),
    (0.01, 0.20, 0.06, False), (0.04, 0.001, -0.109, False),
])
def test_vertical_offset_does_not_change_size_control(top, bottom, expected, in_position):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, top=top, bottom=bottom)
    assert control.in_position is in_position
    assert control.velocity(10.0) == 0.0
    assert control.forward_velocity(10.0) == pytest.approx(expected)
    # 保持框高不变，纵向平移到画面中心，前后方向、速度和到位状态均不变。
    margin = (top + bottom) / 2.0
    box_detection(control, top=margin, bottom=margin)
    assert control.in_position is in_position
    assert control.forward_velocity(10.0) == pytest.approx(expected)


@pytest.mark.parametrize('height', [480, 720, 1080])
def test_custom_size_band_uses_actual_image_height(height):
    control = BoxAlignment(height_min_ratio=0.4, height_max_ratio=0.5)
    control.start(123)
    for ratio, expected in ((0.3, 0.15), (0.45, 0.0), (0.6, -0.15)):
        y1, y2 = height * 0.1, height * (0.1 + ratio)
        control.update(123, True, 320.0, (y1 + y2) / 2.0, 640, height, 10.0,
                       y1=y1, y2=y2)
        assert control.forward_velocity(10.0) == pytest.approx(expected)
        assert control.in_position is (expected == 0.0)


def test_forward_loss_staleness_sessions_finish_and_restart():
    control = BoxAlignment()
    control.start(123)
    assert control.forward_velocity(10.0) == 0.0
    box_detection(control, top=0.3, bottom=0.3, session_id=999)
    assert control.forward_velocity(10.0) == 0.0
    box_detection(control, top=0.3, bottom=0.3)
    assert control.forward_velocity(10.0) > 0.0
    assert control.forward_velocity(10.6) == 0.0
    box_detection(control, detected=False)
    assert control.velocity(10.0) == control.forward_velocity(10.0) == 0.0
    box_detection(control, top=0.3, bottom=0.3)
    box_detection(control, session_id=999, detected=False)
    assert control.forward_velocity(10.0) > 0.0  # 外部轮次不能清除本轮状态。
    control.finish()
    assert control.forward_velocity(10.0) == 0.0
    control.start(124)
    assert not control.in_position
    assert control.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('y1, y2', [
    (None, None), (float('nan'), 666.0), (54.0, float('inf')),
    (-1.0, 666.0), (54.0, 721.0), (400.0, 300.0),
    (360.0, 360.0), (400.0, 666.0),
])
def test_invalid_box_edges_stop_both_axes(y1, y2):
    control = BoxAlignment()
    control.start(123)
    box_detection(control, top=0.3, bottom=0.3)
    control.update(123, True, 100.0, 360.0, 1080, 720, 10.1, y1=y1, y2=y2)
    assert not control.in_position
    assert control.velocity(10.1) == control.forward_velocity(10.1) == 0.0


def test_forward_speed_cap_reversal_and_disabled_compatibility():
    control = BoxAlignment(max_forward_speed=10.0, forward_gain=100.0,
                           forward_direction=-1.0)
    control.start(123)
    box_detection(control, top=0.3, bottom=0.3)
    assert control.forward_velocity(10.0) == -0.3
    box_detection(control, top=0.02, bottom=0.02)
    assert control.forward_velocity(10.0) == 0.3
    slower = BoxAlignment(max_forward_speed=0.1, forward_gain=100.0)
    slower.start(123)
    box_detection(slower, top=0.3, bottom=0.3)
    assert slower.forward_velocity(10.0) == 0.1
    lateral_only = BoxAlignment(enable_forward=False)
    lateral_only.start(123)
    detection(lateral_only, fx=540.0)  # 关闭前后控制后无需框边界。
    assert lateral_only.in_position
    assert lateral_only.forward_velocity(10.0) == 0.0


@pytest.mark.parametrize('options', [
    {'forward_gain': 0.0}, {'forward_gain': float('nan')},
    {'forward_direction': 0.0}, {'forward_direction': float('inf')},
    {'max_forward_speed': -1.0}, {'max_forward_speed': float('inf')},
    {'height_min_ratio': -0.1}, {'height_min_ratio': 0.0},
    {'height_max_ratio': 1.1},
    {'height_min_ratio': 0.8, 'height_max_ratio': 0.8},
    {'height_max_ratio': float('nan')},
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
    node.lock_stop_duration = 1.0
    node.center_locked = False
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
                       top=0.075, bottom=0.075):
    """构造本轮图像消息，驱动真实节点的检测回调。"""
    message = BoxDetection()
    message.session_id = session_id
    message.detected = detected
    message.image_width, message.image_height = 1080, 720
    message.x1, message.x2 = fx - 50.0, fx + 50.0
    message.y1, message.y2 = top * 720.0, (1.0 - bottom) * 720.0
    message.fx, message.fy = fx, (message.y1 + message.y2) / 2.0
    stamp = now - age
    message.stamp.sec = int(stamp)
    message.stamp.nanosec = int((stamp % 1.0) * 1e9)
    node.detection_callback(message)


@pytest.mark.parametrize('lock_on_center', [False, True])
def test_full_position_controls_serial_and_locks_only_when_center_and_size_match(
        monkeypatch, lock_on_center):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.lock_on_center = lock_on_center
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], fx=100.0, top=0.2, bottom=0.2)
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    assert not node.center_locked
    centered_detection(node, now[0], top=0.2, bottom=0.2)
    node.control_tick()
    node.writer.send.assert_called_with(0.0, vx=pytest.approx(0.25))
    assert not node.center_locked
    centered_detection(node, now[0], top=0.02, bottom=0.02)
    node.control_tick()
    node.writer.send.assert_called_with(0.0, vx=pytest.approx(-0.11))
    assert not node.center_locked
    # 大小达标但水平未对中仍然横移，不会提前锁定。
    centered_detection(node, now[0], fx=100.0, top=0.13, bottom=0.02)
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    assert not node.center_locked
    node.writer.reset_mock()
    # 上下边距不对称、cy 偏移也能到位：只判断水平中心和框高 85%。
    centered_detection(node, now[0], top=0.13, bottom=0.02)
    node.writer.send.assert_called_once_with(0.0)  # 到位回调立即停止两轴。
    assert node.center_locked is lock_on_center
    node.control_tick()
    node.writer.send.assert_called_with(0.0)
    if not lock_on_center:
        centered_detection(node, now[0], top=0.02, bottom=0.02)
        node.control_tick()
        node.writer.send.assert_called_with(0.0, vx=pytest.approx(-0.11))


def test_node_stale_or_missing_boxes_stop_forward_commands(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.alignment = BoxAlignment()
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], top=0.2, bottom=0.2)
    node.control_tick()
    assert node.writer.send.call_args.kwargs['vx'] > 0.0
    now[0] += 0.6
    node.control_tick()
    node.writer.send.assert_called_with(0.0)
    centered_detection(node, now[0], top=0.2, bottom=0.2)
    centered_detection(node, now[0], detected=False)
    node.control_tick()
    node.writer.send.assert_called_with(0.0)
    centered_detection(node, now[0], top=0.2, bottom=0.2, age=1.0)
    node.control_tick()
    node.writer.send.assert_called_with(0.0)


def test_lock_mode_sends_zero_for_one_second_and_never_reclaims_released_serial(monkeypatch):
    node, now = controller_harness(monkeypatch)
    node.lock_on_center = True
    node.alignment.start(123)
    node.request_future = Future()
    centered_detection(node, now[0], fx=100.0)
    node.control_tick()
    assert node.writer.send.call_args.args[0] > 0.0
    node.writer.reset_mock()
    centered_detection(node, now[0])
    assert node.center_locked
    assert not node.alignment.active
    assert node.stop_until == 101.0
    node.writer.send.assert_called_with(0.0)
    for current in (100.05, 100.5, 100.95):
        now[0] = current
        centered_detection(node, now[0], fx=900.0)
        node.control_tick()
        node.writer.send.assert_called_with(0.0)
        node.writer.close.assert_not_called()
    assert all(invocation.args == (0.0,) for invocation in node.writer.send.call_args_list)
    node.slope_callback(SimpleNamespace(slope=3))
    node.writer.open.assert_not_called()
    now[0] = 101.0
    node.control_tick()
    node.writer.close.assert_called_once()
    assert node.stop_until is None
    node.writer.reset_mock()
    # 释放串口后，后续坐标、待完成服务期间的重复触发都不能产生速度帧。
    now[0] = 102.0
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
    centered_detection(node, now[0])
    now[0] = 100.2
    if result == 'exception':
        node.request_future.set_exception(RuntimeError('fake camera service failure'))
    else:
        response = CameraUse.Response()
        response.success = result
        node.request_future.set_result(response)
    node.service_done(node.request_future)
    assert node.stop_until == 101.0
    now[0] = 100.95
    node.control_tick()
    node.writer.close.assert_not_called()
    node.writer.send.assert_called_with(0.0)
    now[0] = 101.0
    node.control_tick()
    node.writer.close.assert_called_once()
    node.writer.reset_mock()
    now[0] = 101.5
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
