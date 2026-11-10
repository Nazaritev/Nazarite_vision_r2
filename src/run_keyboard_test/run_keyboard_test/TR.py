import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion,quaternion_from_euler  # 四元数转欧拉角函数
from tf2_ros import TransformListener,Buffer #坐标监听器,缓存
from tf2_ros import StaticTransformBroadcaster #静态坐标发布器
from geometry_msgs.msg import TransformStamped #消息接口
from tur3_run_interfaces.msg import Slope,AdjYaw
import serial
import struct
from nav_msgs.msg import Odometry

class TR(Node):
    def __init__(self):
        super().__init__("TR")
        self.buffer = Buffer()
        # self.timer = self.create_timer(self.timer_period, self.timer_cb)
        self.imu_subscription = self.create_subscription(Odometry,'/Odometry',self.imu_callback,10)
        self.v_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.yaw_auto_timer = self.create_timer(0.05,self.yaw_auto_callback)
        self.listener = TransformListener(self.buffer,self) #监听器（坐标变换缓存处，目标节点）
        self.TF_lister_timer = self.create_timer(0.1,self.get_transform)
        self.slop_sub = self.create_subscription(Slope,'/slope',self.start_goal,5)
        self.slop_pub = self.create_publisher(Slope,'/slope',5)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1.0)
        self.publish_static_tf()
        self.twist = Twist()
        self.start_time = 0.0
        self.dy = 0.0
        self.dx = 0.0
        self.yaw_init_bool = False
        self.yaw_goal = 3.1415
        self.yaw_init = 0.0
        self.num_imu = 0
        self.yaw_auto = False
        self.vw_yaw_first = True
        self.sign = 1
        self.sign2 = 1
        self.vw_yaw_init = 2.6
        self.vw_yaw_2 = 1.6
        self.vw_yaw_3 = 0.8
        self.vw_yaw = 0
        self.aw_yaw = 4.0
        self.a_wy = 2.0
        self.a_wx = 2.0
        self.n_adv_yaw = 0
        self.v_wy_max = 1.6
        self.v_wx_max = 0.8
        self.v_wy_sign = 1
        self.v_wx_sign = 1
        self.run_goal_bool = False
        self.step_num = 0
        self.how_run = []

    def start_goal(self,msg):
        if msg.slope == 1:
            self.yaw_auto = True
            self.yaw_goal = 3.1415
            self.run_goal_bool = True
            self.how_run = self.calc_vy_max(self.dy,self.dx,self.a_wy,self.v_wy_max,self.a_wx,self.v_wx_max,0.05)
            self.get_logger().info(f'{self.how_run},7777777777')

    def calc_vy_max(self, dy, dx, a_wy, v_wy_max, a_wx, v_wx_max, time_once):
        """
        二维梯形速度规划（位移-加速度形式）
        返回: [[vy, vx], [vy, vx], ...]  每个元素是一个时间步的 (vy, vx) 指令
        """

        def _trap_1d(d, a, v_max, dt):
            """单轴梯形规划，返回该轴的速度序列（list）"""
            seq = []
            if d <= 0.0:
                return seq  # 无位移，不产生任何步

            # 加速到 v_max 所需位移：d_acc = v_max^2 / (2a)
            d_acc = v_max * v_max / (2.0 * a)

            if d_acc < d:
                # 能到顶速：加速段 + 匀速段 + 减速段
                v_peak = v_max
                d_uniform = d - 2.0 * d_acc
                n_uniform = int(round(d_uniform / v_max / dt))  # 匀速步数
            else:
                # 到不了顶速：三角形速度曲线，峰值速度由 d/2 决定
                v_peak = math.sqrt(a * d)  # 加速到 d/2 时的速度
                n_uniform = 0

            # 加速段：v = a*dt, 2a*dt, ... 直到 >= v_peak
            acc = []
            v = 0.0
            while v < v_peak - 1e-9:
                v += a * dt
                if v > v_peak:
                    v = v_peak
                acc.append(v)
            # 减速段是加速段的镜像
            dec = acc[::-1]

            seq = acc + [v_peak] * n_uniform + dec
            return seq

        seq_y = _trap_1d(dy, a_wy, v_wy_max, time_once)
        seq_x = _trap_1d(dx, a_wx, v_wx_max, time_once)

        # 两轴步数对齐：取较长者，短的在末尾补 0（已经到位的轴不再输出速度）
        n = max(len(seq_y), len(seq_x))
        seq_y += [0.0] * (n - len(seq_y))
        seq_x += [0.0] * (n - len(seq_x))

        how_run = [[seq_y[i], seq_x[i]] for i in range(n)]
        return how_run
        

    def calc_vx_vy(self,d_yaw,v_wy,v_wx,vw_yaw,time_after):
        yaw_new = d_yaw - vw_yaw * (time_after/2)
        
        # 世界系目标：world_vx=0，world_vy = v_wy
        # 转到本体坐标系
        vx1 = -float(v_wy * math.sin(yaw_new))*self.v_wy_sign
        vy1 = float(v_wy * math.cos(yaw_new))*self.v_wy_sign
        vx2 = float(v_wx * math.cos(yaw_new)) * self.v_wx_sign
        vy2 = float(v_wx * math.sin(yaw_new)) * self.v_wx_sign
        return vx1+vx2, vy1+vy2


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

    def run(self,vx=0.0,vy=0.0,vw=0.0):
        # self.twist.linear.x = vx
        # self.twist.linear.y = vy
        # self.twist.angular.z = vw
        # self.get_logger().info(f'vx={vx},vy={vy},vw={vw}')
        # self.v_publisher.publish(self.twist)
        vx_int = int(vx * 1000)
        vy_int = int(vy * 1000)
        vw_int = int(vw*10)
        mode = int(0)
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
        #self.get_logger().info(f'{full_frame}')
        self.ser.write(full_frame)

    def yaw_auto_callback(self):
        if self.yaw_auto == False and self.run_goal_bool == False:
            return
        v_wy = 0
        v_wx = 0
        if self.step_num < len(self.how_run):
            v_wy = self.how_run[self.step_num][0]
            v_wx = self.how_run[self.step_num][1]
            self.step_num += 1
            # self.get_logger().info(f'{222}')
        else:
            self.run_goal_bool = False
            # self.get_logger().info(f'{333}')
            # self.get_logger().info(f'{self.run_goal_bool},{self.yaw_auto}')
            
        # self.get_logger().info(f'{self.step_num}')
        # self.get_logger().info(f'{self.run_goal_bool},{self.yaw_auto}')
        yaw_change = math.atan2(math.sin(self.yaw_goal - self.yaw), math.cos(self.yaw_goal - self.yaw))
        
        
        if abs(yaw_change)<0.01:#!!!!!!!!!!!!!!!!!!!!!!!
            self.vw_yaw = 0.0
            self.vw_yaw_3 = 0.1
            self.n_adv_yaw += 1
            self.vw_yaw_first = True
            if self.n_adv_yaw == 4 :
                self.yaw_auto = False
                self.vw_yaw_3 = 0.4
                self.n_adv_yaw = 0
        else:
            self.n_adv_yaw = 0

            self.sign = math.copysign(1, yaw_change)
            if self.vw_yaw_first == True:
                self.sign2 = self.sign
                # self.vw_yaw_first = False
            if self.vw_yaw == self.vw_yaw_init:
                pass
            else:
                self.sign2 = self.sign

            if yaw_change*self.sign2 > math.pi/3:
                if self.vw_yaw_first == True:
                    self.vw_yaw = self.vw_yaw_init
            elif yaw_change*self.sign2 > math.pi/10:
                if self.vw_yaw_first == True:
                    self.vw_yaw = self.vw_yaw_2
                if self.vw_yaw > self.vw_yaw_2:
                    self.vw_yaw = self.vw_yaw - self.aw_yaw*0.05
                    if self.vw_yaw < self.vw_yaw_2:
                        self.vw_yaw = self.vw_yaw_2
            else :
            # yaw_change*self.sign2 > math.pi/10:
                if self.vw_yaw_first == True:
                    self.vw_yaw = self.vw_yaw_3
                if self.vw_yaw > self.vw_yaw_3:
                    self.vw_yaw = self.vw_yaw - self.aw_yaw*0.05
                    if self.vw_yaw < self.vw_yaw_3:
                        self.vw_yaw = self.vw_yaw_3
            self.vw_yaw_first = False
        # self.get_logger().info(f'{self.vw_yaw}')
        # self.get_logger().info(f'{yaw_change}')

        if self.run_goal_bool == True:
            vx,vy = self.calc_vx_vy(yaw_change,v_wy,v_wx,-self.vw_yaw*self.sign2,0.05)
            self.run(vx=vx,vy=vy,vw=float(self.vw_yaw*self.sign2))
        else:
            self.run(vw=float(self.vw_yaw*self.sign2))
        
        if self.run_goal_bool == False and self.yaw_auto == False:
            sl = Slope()
            sl.slope = 0
            self.slop_pub.publish(sl)
            self.vw_yaw_first = True
            self.step_num = 0
            self.get_logger().info(f'9999999999999999')
            self.run(0.0)
            self.run(0.0)
            return


    def publish_static_tf(self):
        """
        发布TF 从 base_link 到 camera_link 之间的坐标关系
        """
        transform = TransformStamped() #初始化TF消息接口
        transform.header.frame_id = 'map' #父坐标系
        transform.child_frame_id = 'goal' #子坐标系
        transform.header.stamp = self.get_clock().now().to_msg() #时间戳
        # 子在父......
        # translation 平移
        transform.transform.translation.x = 8.4
        transform.transform.translation.y = 2.1
        transform.transform.translation.z = 0.0
        # 欧拉角转四元数
        q = quaternion_from_euler(0,0,math.pi) #绕x轴旋转180度
        # 旋转部分进行赋值
        # rotation 旋转
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        # 发布静态坐标
        self.static_broadcaster.sendTransform([transform])
        self.get_logger().info(f'发布静态TF:{transform}')

    def imu_callback(self, msg):
        q = msg.pose.pose.orientation
        if self.yaw_init_bool == False:
            self.num_imu += 1

        # 四元数 -> 欧拉角 (roll, pitch, yaw)，单位：弧度
        rotation_euler = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w,
        ])

        roll, pitch, yaw = rotation_euler
        if self.yaw_init_bool == False and self.num_imu == 4:
            self.yaw_init = yaw
            self.yaw_init_bool = True
            self.num_imu = 0
        self.yaw = yaw - self.yaw_init
        if time.time() - self.start_time > 0.2:
            self.get_logger().info(f'rool={rotation_euler[0]:.3f},pithch={rotation_euler[1]:.3f},yaw={self.yaw:.3f}')
            self.start_time = time.time()

    def get_transform(self):
        try:
            result = self.buffer.lookup_transform(
                'goal', #目标坐标
                'footprint_link', #源坐标
            #    时间0=最新                    超时1秒，等不到就报错
                rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
            t = result.transform.translation
            self.dx = t.x
            self.dy = t.y
            self.v_wy_sign = -math.copysign(1,t.y)
            self.v_wx_sign = -math.copysign(1,t.x)
            self.get_logger().info(f'x={self.dx}  y={self.dy}')
            self.dy = abs(self.dy)
            self.dx = abs(self.dx)
        except Exception as e: #错误信息放在e里
            self.get_logger().warn(f'获取坐标变换失败：原因{str(e)}')

def main(args=None):
    rclpy.init()
    node = TR()
    node.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, False)])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()