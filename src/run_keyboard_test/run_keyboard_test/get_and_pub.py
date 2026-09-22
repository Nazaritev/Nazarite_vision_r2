import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial
import struct

class GetAndPub(Node):
    def __init__(self):
        super().__init__("get_and_pub")
        self.sub = self.create_subscription(Twist,'/cmd_vel',self.sub_callback,10)
        self.timer_1 = self.create_timer(0.1,self.timer_1_callback)
        self.pub_bool = 0
        self.time_start = None
        self.v = [0,0,0,0]
        # /dev/ttyUSB0
        # /dev/pts/6
        # /dev/usbv
        self.ser = serial.Serial('/dev/ttyUSB1',961200,timeout=1.0)
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.a = 0.2 # 每次减少10%
        self.a_time = 0.12 #每次减少的时间间隔
        self.get_logger().info(f'当前停车缓冲时间{1/self.a*self.a_time}')

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

    def sub_callback(self,msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vw = msg.angular.z
        if self.time_start != None:
            if self.time_start - time.time() < 0.05 :
                if self.vx == 0 and self.vy == 0 and self.vw == 0:
                    pass
                else:
                    return

        self.time_start = time.time()

        self.pub_bool += 1
        if self.vx < 0.0 and self.vw != 0.0:
            if self.vw <= 0.0 :
                self.vy = -self.vx
                self.vx = 0.0
                self.vw = 0.0
            else:
                self.vy = self.vx
                self.vx = 0.0
                self.vw = 0.0

        # print((self.vx,self.vy,self.vw))
        self.get_logger().info(f'(vx={self.vx},vy={self.vy},vw={self.vw})')

        vx_int = int(self.vx * 1000)
        vy_int = int(self.vy * 1000)
        vw_int = int(self.vw * 10)
        flag = int(0)
        mode = int(0)
        reserved = 0  # 保留字，当前固定填 0

        frame_head = struct.pack('<B h h h B B B', 
                                     0xA5,       # SOF (帧头，1字节)
                                     vx_int,     # Vx (2字节)
                                     vy_int,     # Vy (2字节)
                                     vw_int,     # Vw (2字节)
                                     flag,       # Flag (1字节)
                                     reserved,   # Reserved (1字节)
                                     mode)       # Mode (1字节)

        # 3. 提取纯数据部分进行 CRC 校验
        # 使用 [1:] 跳过第0个字节（即 0xA5 帧头），只对后面的 9 个字节的数据进行校验
        data_to_crc = frame_head[1:]
        crc = self.calc_crc8(data_to_crc)

        full_frame = frame_head + struct.pack('<B B', crc, 0xFF)
        # print(full_frame)
        self.get_logger().info(f'{full_frame}')
        self.ser.write(full_frame)
        # self.ser.write(b'aa')


    def timer_1_callback(self):
        if self.time_start == None or self.pub_bool == 0:
            return
        else:
            if self.pub_bool == 1:
                if time.time() - self.time_start > 0.5 :
                    self.pub_bool = 0
                    vx_int = int(self.vx * 1000)
                    vy_int = int(self.vy * 1000)
                    vw_int = int(self.vw * 10)
                    flag = int(0)
                    mode = int(0)
                    bool_vx = 1
                    bool_vy = 1
                    bool_vw = 1
                    if vx_int < 0:
                        bool_vx = -1
                    if vy_int < 0:
                        bool_vy = -1
                    if vw_int < 0:
                        bool_vw = -1
                    
                    reserved = 0  # 保留字，当前固定填 0
                    a = self.a
                    a_vx = vx_int * a
                    a_vy = vy_int * a
                    a_vw = vw_int * a
                    while (vx_int*bool_vx > 0 or vy_int*bool_vy > 0 or vw_int*bool_vw >0):
                                            
                        vx_int -= a_vx
                        vy_int -= a_vy
                        vw_int -= a_vw
                                            
                        if vx_int*bool_vx < 0:
                            vx_int = 0
                        if vy_int*bool_vy < 0:
                            vy_int = 0
                        if vw_int*bool_vw < 0:
                               vw_int = 0
                                            
                        self.get_logger().info(f'(vx={vx_int/1000},vy={vy_int/1000},vw={vw_int/10})')
                        vx_int = int(vx_int)
                        vy_int = int(vy_int)
                        vw_int = int(vw_int)
                        # self.get_logger().info(f'{vw_int}')
                        frame_head = struct.pack('<B h h h B B B', 
                                                     0xA5,       # SOF (帧头，1字节)
                                                     vx_int,     # Vx (2字节)
                                                     vy_int,     # Vy (2字节)
                                                     vw_int,     # Vw (2字节)
                                                     flag,       # Flag (1字节)
                                                     reserved,   # Reserved (1字节)
                                                     mode)       # Mode (1字节)
                        data_to_crc = frame_head[1:]
                        crc = self.calc_crc8(data_to_crc)
                                                                
                        full_frame = frame_head + struct.pack('<B B', crc, 0xFF)
                        self.get_logger().info(f'{full_frame}')
                        self.ser.write(full_frame)
                        time.sleep(self.a_time)
                else:
                    pass
            elif self.pub_bool >= 2:
                
                if time.time() - self.time_start > 0.09 :
                    self.pub_bool = 0
                    vx_int = int(self.vx * 1000)
                    vy_int = int(self.vy * 1000)
                    vw_int = int(self.vw * 10)
                    flag = int(0)
                    mode = int(0)
                    bool_vx = 1
                    bool_vy = 1
                    bool_vw = 1
                    if vx_int < 0:
                        bool_vx = -1
                    if vy_int < 0:
                        bool_vy = -1
                    if vw_int < 0:
                        bool_vw = -1

                    reserved = 0  # 保留字，当前固定填 0
                    a = self.a
                    a_vx = vx_int * a
                    a_vy = vy_int * a
                    a_vw = vw_int * a
                    while (vx_int*bool_vx > 0 or vy_int*bool_vy > 0 or vw_int*bool_vw >0):
                        
                        vx_int -= a_vx
                        vy_int -= a_vy
                        vw_int -= a_vw
                        
                        if vx_int*bool_vx < 0:
                            vx_int = 0
                        if vy_int*bool_vy < 0:
                            vy_int = 0
                        if vw_int*bool_vw < 0:
                            vw_int = 0
                        
                        self.get_logger().info(f'(vx={vx_int/1000},vy={vy_int/1000},vw={vw_int/10})')
                        vx_int = int(vx_int)
                        vy_int = int(vy_int)
                        vw_int = int(vw_int)
                        # self.get_logger().info(f'{vw_int}')
                        frame_head = struct.pack('<B h h h B B B', 
                                                     0xA5,       # SOF (帧头，1字节)
                                                     vx_int,     # Vx (2字节)
                                                     vy_int,     # Vy (2字节)
                                                     vw_int,     # Vw (2字节)
                                                     flag,       # Flag (1字节)
                                                     reserved,   # Reserved (1字节)
                                                     mode)       # Mode (1字节)
                        data_to_crc = frame_head[1:]
                        crc = self.calc_crc8(data_to_crc)
                                            
                        full_frame = frame_head + struct.pack('<B B', crc, 0xFF)
                        self.get_logger().info(f'{full_frame}')
                        self.ser.write(full_frame)
                        time.sleep(self.a_time)

                            


def main():
    rclpy.init()
    node = GetAndPub()
    rclpy.spin(node)
    rclpy.shutdown()

# 终端1:socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 终端2:picocom /dev/pts/6 -b 961200
# 终端3:虚拟接口代码写入程序运行代码
# colcon build --packages-select run_keyboard_test
# ros2 launch run_keyboard_test teleop_run.launch.py