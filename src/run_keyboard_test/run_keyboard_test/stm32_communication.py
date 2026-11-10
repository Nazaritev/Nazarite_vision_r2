import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

class Stm32Communication(Node):
    def __init__(self):
        super().__init__('stm32_communication')
        self.cmd_vel_sub = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1.0)
        self.stop_time = 0.0
        self.stop_num = 0

    def calc_crc8(self, data: bytes, poly: int = 0x07, init_value: int = 0x00) -> int:
        """
        计算标准 CRC-8
        :param data: 需要计算的字节数据
        :param poly: 多项式 (标准通常为 0x07)
        :param init_value: 初始值
        :return: 计算出的 CRC-8 校验值 (0-255)
        """
        crc = init_value
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFF  # 确保始终是8位
        return crc

    def run(self,vx=0,vy=0,vw=0):
        vx_int = int(vx * 1000)
        vy_int = int(vy * 1000)
        vw_int = int(vw*10)
        mode = int(0)
        reserved = 0  # 保留字，当前固定填 0
        self.get_logger().info(f'(vx={vx_int/1000},vy={vy_int/1000},vw={vw_int/10})')
        # self.get_logger().info(f'{vw_int}')
        frame_head = struct.pack('<B h h h B', 
                                     0xA5,       # SOF (帧头，1字节)
                                     vx_int,     # Vx (2字节)
                                     vy_int,     # Vy (2字节)
                                     vw_int,     # Vw (2字节)
                                     mode)       # Mode (1字节)
        data_to_crc = frame_head[1:]
        crc = self.calc_crc8(data_to_crc)
                                                                
        full_frame = frame_head + struct.pack('<B B', crc, 0xFF)
        # self.get_logger().info(f'{full_frame}')
        self.ser.write(full_frame)

    def calc_vx_vy(self,v):
        return v
    
    def cmd_vel_callback(self,msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vw = msg.angular.z
        if vx==0.0 and vy==0.0 and vw==0.0:
            time_now = time.time()
            if time_now - self.stop_time > 0.5:
                self.stop_num = 0
            if self.stop_num <= 4:
                self.stop_time = time_now
                self.stop_num += 1
            else:
                self.stop_time = time_now
                return
        self.run(vx,vy,-vw)
        



def main():
    rclpy.init()
    node = Stm32Communication()
    rclpy.spin(node)
    rclpy.shutdown()