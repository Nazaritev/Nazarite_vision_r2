import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion  # 四元数转欧拉角函数
import time
from nav_msgs.msg import Odometry
import serial
import struct
# -0.162
# ros2 run tur3_run_test run_slope
class ImuYawListener(Node):
    def __init__(self):
        super().__init__('imu_yaw_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.imu_callback,
            10
        )
        self.ser = serial.Serial('/dev/ttyUSB1',961200,timeout=1.0)
        self.continue_time = 0.3
        self.start = False
        # self.next()
        self.pitch_init = 0
        self.pitch_init_bool = False
        self.pitch = 0
        self.bool_slope = 0
        self.get_logger().info('imu_yaw_listener started, waiting for /imu ...')
        self.start_time = time.time()
        self.vx1 = 0.4
        self.vx2 = 0.2
        self.fix = 0.04
        self.bool_fix = True
        self.out_time_slop = None
        self.start_time_slop = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.bool_pitch = self.create_timer(0.1,self.bool_pitch_callback)
        self.bool_stop = self.create_timer(0.1,self.bool_stop_callback)

    # def nav_service(self):
    #     self.service = self.create_subscription(Slope,'/slope',self.go_slope,5)
    #     self.pub = self.create_publisher(Slope,'/slope',5)

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

    def next(self):
        start = time.time()
        while time.time() - start < 2.2:
            self.run(vy0=0.4)
        start = time.time()
        while time.time() - start < 2.3:
            self.run(vw0=0.5)
        self.run(0)

    def fix_vx(self,vx):
        return -vx*self.fix

    def run(self,vx=0,fix_vx = False,vy0=0,vw0=0):
        vy = 0
        if fix_vx == True:
            vy = self.fix_vx(vx)
        vx_int = int(vx * 1000)
        vy_int = int((vy+vy0) * 1000)
        vw_int = int(vw0*10)
        flag = int(0)
        mode = int(0)
        reserved = 0  # 保留字，当前固定填 0
        #self.get_logger().info(f'(vx={vx_int/1000},vy={vy_int/1000},vw={vw_int/10})')
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
        # self.get_logger().info(f'{full_frame}')
        self.ser.write(full_frame)

    def go_slope(self,msg):
        if msg.slope == 1:
            self.start = True
            self.pitch_init_bool = False
            self.fix = 0.04
            # self.continue_time = 0.2

        
    def bool_stop_callback(self):
        if self.start == False:
            return
        if self.bool_slope == 2:
            # self.vx = 0.2
            # self.run(0.2)
            self.bool_fix = False
            #self.run(self.vx2,self.bool_fix)
            time.sleep(self.continue_time)
            self.run(0)
            self.start = False
            self.start_time_slop = None
            self.bool_slope = 0
            self.next()
            # sl = Slope()
            # sl.slope = 2
            # self.pub.publish(sl)
            # while True:
            # 	self.get_logger().info(f'1111111111111111')
            # 	time.sleep(0.2)
            return
        self.run(self.vx1,self.bool_fix)
        
            


    def bool_pitch_callback(self):
        if self.start == False:
            return
        if self.start_time_slop != None and time.time() - self.start_time_slop > 0.4:
            self.bool_slope = 1
            self.bool_fix = True
            self.fix = 0.1
        if self.pitch < -0.12:
            if self.start_time_slop == None:
                self.start_time_slop = time.time()
        if self.bool_slope == 1 and self.pitch > -0.08:
            self.bool_slope = 2
        if self.start_time_slop != None and time.time() - self.start_time_slop > 6:
            self.vx1 = 0.25
        #     self.publisher.publish(self.twist)
            


    def imu_callback(self, msg):
        q = msg.pose.pose.orientation
        

        # 四元数 -> 欧拉角 (roll, pitch, yaw)，单位：弧度
        rotation_euler = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w,
        ])

        roll, pitch, yaw = rotation_euler
        if self.pitch_init_bool == False and self.start == True:
            self.pitch_init = pitch
            self.pitch_init_bool = True
        self.pitch = pitch - self.pitch_init
        

        # self.get_logger().info(f'四元数: x={q.x:.4f}, y={q.y:.4f}, z={q.z:.4f}, w={q.w:.4f}')
        if time.time() - self.start_time > 0.1:
            # self.get_logger().info(f'{self.pitch} = {pitch} - {self.pitch_init}')
            self.get_logger().info(f'rool={rotation_euler[0]:.3f},pithch={self.pitch:.3f},yaw={rotation_euler[2]:.3f}')
            self.start_time = time.time()
        # self.get_logger().info(f'Yaw: {yaw:.4f} rad ({yaw * 180.0 / 3.141592653589793:.2f} deg)')


def main():
    rclpy.init()
    node = ImuYawListener()
    rclpy.spin(node)
    rclpy.shutdown()
def main():
    rclpy.init()
    node = ImuYawListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, stopping robot...')
        node.run(0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # 关闭串口，避免占用
        # try:
        #     node.ser.close()
        # except Exception:
        #     pass

if __name__ == '__main__':
    main()
