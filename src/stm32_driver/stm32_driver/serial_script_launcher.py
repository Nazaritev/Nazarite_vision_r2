#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

try:
    import serial

    SERIAL_AVAILABLE = hasattr(serial, "Serial")
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False


class SerialScriptLauncher(Node):
    def __init__(self):
        super().__init__("serial_script_launcher")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout_sec", 0.05)
        self.declare_parameter(
            "script_path",
            "/home/dgut/Nazarite/src/stm32_driver/scripts/start_serial_selected_stack.sh",
        )
        self.declare_parameter("start_commands", "1,start,START")
        self.declare_parameter("stop_commands", "0,stop,STOP")
        self.declare_parameter("allow_restart", False)
        self.declare_parameter("reconnect_period_sec", 1.0)
        self.declare_parameter("log_rx_bytes", True)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.timeout_sec = max(0.0, float(self.get_parameter("timeout_sec").value))
        self.script_path = str(self.get_parameter("script_path").value)
        self.start_commands = self._parse_command_set(
            str(self.get_parameter("start_commands").value)
        )
        self.stop_commands = self._parse_command_set(
            str(self.get_parameter("stop_commands").value)
        )
        self.allow_restart = self._as_bool(self.get_parameter("allow_restart").value)
        self.reconnect_period_sec = max(
            0.2, float(self.get_parameter("reconnect_period_sec").value)
        )
        self.log_rx_bytes = self._as_bool(self.get_parameter("log_rx_bytes").value)

        self.serial_handle = None
        self.process: Optional[subprocess.Popen] = None
        self.rx_buffer = bytearray()
        self._lock = threading.Lock()
        self._last_open_log_time = 0.0
        self._running = True

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self.create_timer(1.0, self._poll_process)

        self.get_logger().info(
            "串口脚本启动器已就绪: port=%s baud=%d start=%s stop=%s script=%s"
            % (
                self.serial_port,
                self.baudrate,
                sorted(self.start_commands),
                sorted(self.stop_commands),
                self.script_path,
            )
        )

    def destroy_node(self):
        self._running = False
        self._close_serial()
        return super().destroy_node()

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _parse_command_set(value: str) -> set[str]:
        return {item.strip() for item in value.split(",") if item.strip()}

    def _open_serial_if_needed(self) -> bool:
        if not SERIAL_AVAILABLE:
            self.get_logger().error("python3-serial 不可用，无法监听串口")
            return False
        if self.serial_handle is not None and self.serial_handle.is_open:
            return True
        try:
            self.serial_handle = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout_sec,
                write_timeout=1.0,
            )
            self.get_logger().info("已打开串口: %s @ %d" % (self.serial_port, self.baudrate))
            return True
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_open_log_time >= 5.0:
                self.get_logger().warning(
                    "打开串口失败: %s, error=%s" % (self.serial_port, exc)
                )
                self._last_open_log_time = now
            return False

    def _close_serial(self) -> None:
        with self._lock:
            if self.serial_handle is not None:
                try:
                    self.serial_handle.close()
                except Exception:
                    pass
                self.serial_handle = None

    def _reader_loop(self) -> None:
        while self._running and rclpy.ok():
            if not self._open_serial_if_needed():
                time.sleep(self.reconnect_period_sec)
                continue
            try:
                data = self.serial_handle.read(self.serial_handle.in_waiting or 1)
            except Exception as exc:
                self.get_logger().warning("串口读取失败，准备重连: %s" % exc)
                self._close_serial()
                time.sleep(self.reconnect_period_sec)
                continue
            if not data:
                continue
            self._handle_rx_bytes(data)

    def _handle_rx_bytes(self, data: bytes) -> None:
        if self.log_rx_bytes:
            self.get_logger().info("收到串口数据: %s" % data.hex(" "))
        self.rx_buffer.extend(data)

        while self.rx_buffer:
            newline_positions = [
                pos for pos in (self.rx_buffer.find(b"\n"), self.rx_buffer.find(b"\r")) if pos >= 0
            ]
            if newline_positions:
                end = min(newline_positions)
                raw = bytes(self.rx_buffer[:end])
                del self.rx_buffer[: end + 1]
                self._handle_command(raw)
                continue

            if len(self.rx_buffer) == 1:
                token = self.rx_buffer.decode("ascii", errors="ignore").strip()
                if token in self.start_commands or token in self.stop_commands:
                    self.rx_buffer.clear()
                    self._handle_command(token.encode("ascii"))
                    continue
            break

        if len(self.rx_buffer) > 256:
            self.get_logger().warning("串口命令缓存过长，已清空")
            self.rx_buffer.clear()

    def _handle_command(self, raw: bytes) -> None:
        command = raw.decode("utf-8", errors="ignore").strip()
        if not command:
            return

        self.get_logger().info("解析串口指令: %s" % command)
        if command in self.start_commands:
            self._start_script()
        elif command in self.stop_commands:
            self._stop_script()
        else:
            self.get_logger().warning(
                "未知串口指令: %s，可用启动指令=%s，停止指令=%s"
                % (command, sorted(self.start_commands), sorted(self.stop_commands))
            )

    def _start_script(self) -> None:
        if self.process is not None and self.process.poll() is None:
            if not self.allow_restart:
                self.get_logger().info("脚本已在运行，忽略重复启动指令")
                return
            self._stop_script()

        if not os.path.isfile(self.script_path):
            self.get_logger().error("脚本不存在: %s" % self.script_path)
            return
        if not os.access(self.script_path, os.X_OK):
            self.get_logger().error("脚本不可执行，请 chmod +x: %s" % self.script_path)
            return

        try:
            self.process = subprocess.Popen(
                [self.script_path],
                start_new_session=True,
                stdout=None,
                stderr=None,
            )
            self.get_logger().info(
                "已通过串口指令启动脚本: pid=%d script=%s"
                % (self.process.pid, self.script_path)
            )
        except Exception as exc:
            self.get_logger().error("启动脚本失败: %s" % exc)

    def _stop_script(self) -> None:
        if self.process is None or self.process.poll() is not None:
            self.get_logger().info("没有正在运行的脚本进程")
            self.process = None
            return
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.get_logger().info("已发送停止信号到脚本进程组: pid=%d" % self.process.pid)
        except Exception as exc:
            self.get_logger().error("停止脚本失败: %s" % exc)

    def _poll_process(self) -> None:
        if self.process is None:
            return
        rc = self.process.poll()
        if rc is None:
            return
        self.get_logger().info("脚本进程已退出: rc=%s" % rc)
        self.process = None


def main():
    rclpy.init()
    node = SerialScriptLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
