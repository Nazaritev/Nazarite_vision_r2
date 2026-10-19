"""沿用 nav_to_pose.run 的串口速度协议，只允许低速横移。"""

import math
import struct


HARD_MAX_LATERAL_SPEED = 0.3


def speed_limit(value):
    """参数可以进一步降低上限，但不能突破用户指定的 0.3 m/s。"""
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError('横移速度上限必须为有限正数')
    return min(value, HARD_MAX_LATERAL_SPEED)


def crc8(data):
    """CRC-8：多项式 0x07、初值 0，与现有底盘代码一致。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07 if crc & 0x80 else crc << 1) & 0xFF
    return crc


def encode_lateral_velocity(velocity, max_speed=HARD_MAX_LATERAL_SPEED):
    """生成 12 字节小端帧；Vx、Vw 和三个标志字节始终为零。"""
    velocity = float(velocity)
    if not math.isfinite(velocity):
        raise ValueError('横移速度必须为有限数值')
    limit = speed_limit(max_speed)
    velocity = max(-limit, min(limit, velocity))
    # m/s 转 mm/s；向零取整，避免编码后超过速度上限。
    frame = struct.pack('<BhhhBBB', 0xA5, 0, int(velocity * 1000), 0, 0, 0, 0)
    return frame + bytes((crc8(frame[1:]), 0xFF))


class SerialVelocity:
    """触发时打开串口，结束并发送停车帧后关闭，待机不占用设备。"""

    def __init__(self, port, baudrate, max_speed, serial_factory=None):
        self.port = port
        self.baudrate = baudrate
        self.max_speed = speed_limit(max_speed)
        self.serial_factory = serial_factory
        self.connection = None

    @property
    def is_open(self):
        return self.connection is not None

    def open(self):
        if self.is_open:
            return
        factory = self.serial_factory
        if factory is None:
            # 延迟导入，纯协议测试不需要串口硬件或 pyserial。
            import serial
            factory = serial.Serial
        self.connection = factory(
            port=self.port, baudrate=self.baudrate,
            timeout=0.05, write_timeout=0.2, exclusive=True)

    def send(self, velocity):
        if not self.is_open:
            raise RuntimeError('串口尚未打开')
        frame = encode_lateral_velocity(velocity, self.max_speed)
        if self.connection.write(frame) != len(frame):
            raise OSError('串口速度帧未完整写出')

    def close(self):
        if not self.is_open:
            return
        try:
            self.send(0.0)
        finally:
            connection = self.connection
            self.connection = None
            connection.close()
