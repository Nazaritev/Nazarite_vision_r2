"""验证横移、停车和串口帧；全部使用模拟串口，不驱动实车。"""

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
    SerialVelocity, crc8, encode_lateral_velocity,
)
from run_keyboard_test.yolo_control import LateralAlignment, YoloControlNode
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


def controller_harness(monkeypatch):
    """只替换 ROS 通信和串口依赖，执行真实节点回调及定时器。"""
    import run_keyboard_test.yolo_control as module
    now = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    node = object.__new__(YoloControlNode)
    node.alignment = LateralAlignment()
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


def centered_detection(node, now, fx=432.0, session_id=123, detected=True, age=0.0):
    """构造本轮图像消息，驱动真实节点的检测回调。"""
    message = BoxDetection()
    message.session_id = session_id
    message.detected = detected
    message.image_width, message.image_height = 1080, 720
    message.fx, message.fy = fx, 200.0
    stamp = now - age
    message.stamp.sec = int(stamp)
    message.stamp.nanosec = int((stamp % 1.0) * 1e9)
    node.detection_callback(message)


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
    node.model.predict.return_value = [SimpleNamespace(boxes=FakeBoxes(False))]
    node.process_frame()
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
