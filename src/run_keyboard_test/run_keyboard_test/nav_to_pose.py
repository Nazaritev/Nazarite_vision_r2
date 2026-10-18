from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.duration import Duration
import math
from tf_transformations import quaternion_from_euler,euler_from_quaternion
import threading
from geometry_msgs.msg import Twist
import time
from tur3_run_interfaces.srv import UseNavToPose,NavOnce
from tur3_run_interfaces.msg import Slope
# ros2 run tur3_run_test nav_to_pose
# 单点导航
# 增加初始化位置设置

class NavToPose(BasicNavigator):
    def __init__(self):
        super().__init__('nav_to_pose')
        self.yaw = 0
        self.bool_slope = False
        # thread = threading.Thread(target=self.get)
        self.get_pose = self.create_subscription(Odometry,'/odom',self.nav_pose_callback,1)
        self.after_slop = self.create_subscription(Slope,'/slope',self.slope_callback,1)
        # thread2 = threading.Thread(target=self.slope_thread2)
        # thread2.start()
        self.slope_publisher = self.create_publisher(Slope,'/slope',5)
        self.vw_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.nav_mode_client = self.create_client(UseNavToPose, '/nav_mode/slope')
        # self.run_slop(0.3)

        # thread.start()
        self.pose_init(-0.4,3.2,3.1415626)
        self.nav2_mult([[1.8,3.6],[7.3,3.6],[8.1,2.5]])
        
        sl = Slope()
        sl.slope = 1
        self.slope_publisher.publish(sl)
        
        
    def nav2_mult(self,poses):
        self.waitUntilNav2Active()
        goal_poses = []
        for pose in poses:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose[0]
            goal_pose.pose.position.y = pose[1]
            # goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)
        self.get_logger().info(f'{goal_poses}')
        self.goThroughPoses(goal_poses)
        # 判断结束及获取反馈
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            # self.get_logger().info(
            #     f'当前目标编号：{feedback.current_waypoint}')
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')   

    def slope_thread2(self):
        self.after_slop = self.create_subscription(Slope,'/slope',self.slope_callback,1)

    def slope_callback(self,msg):
        self.get_logger().info(f'{msg.slope}')
        if msg.slope == 2:
            self.bool_slope = True
            self.get_logger().info(f'1111111111111111')

    def back(self,vx,dis):
        msg = Twist()
        msg.linear.x = -vx
        self.vw_publisher.publish(msg)
        time.sleep(dis/vx)
        self.vw_publisher.publish(Twist())


    def get(self):
        self.get_pose = self.create_subscription(Odometry,'/odom',self.nav_pose_callback,1)

    def nav_pose_callback(self,msg):
        q = msg.pose.pose.orientation
        # 四元数转欧拉角，顺序 Z-Y-X，返回 (roll, pitch, yaw)
        roll, pitch, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def pose_init(self,x,y,yaw):
        # navigator = BasicNavigator()
        # ---------- 设置初始化位姿 ----------
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = 0.0
        # 朝向：绕 Z 轴转 yaw 弧度
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.setInitialPose(initial_pose)

    def adj_angle(self,yaw_now,yaw_goal):
        yaw = math.atan2(math.sin(yaw_goal - yaw_now), math.cos(yaw_goal - yaw_now))
        self.get_logger().info(f'修正角度{yaw}')
        twis = Twist()
        if yaw < 0:
            twis.angular.z = -1.0
        else:
            twis.angular.z = 1.0
        sleep_time = int(yaw/twis.angular.z*10000)/10000
        time.sleep(1.5)
        self.vw_publisher.publish(twis)
        self.get_logger().info(f'{sleep_time}')
        t = sleep_time-0.0
        if t < 0 :
            t = sleep_time
        time.sleep(t)
        self.vw_publisher.publish(Twist())
        self.get_logger().info(f'角度调节完成')
        time.sleep(0.5)

    # def run_slop(self,continue_time):
    #     while self.nav_mode_client.wait_for_service(timeout_sec=1.0) is False:
    #         self.get_logger().info('等待参数更新服务端上线！')
    #     self.get_logger().info(f'上坡')
    #     request = UseNavToPose.Request()
    #     request.use_nav = 2
    #     request.continue_time = continue_time
    #     future = self.nav_mode_client.call_async(request)
    #     # while not future.done():
    #     #     time.sleep(0.01)
    #     rclpy.spin_until_future_complete(self, future) # 等待服务端返回响应
    #     response = future.result()
    #     self.get_logger().info(f'{response}')

        

        # nav_mode_client = self.create_client(UseNavToPose, '/nav_mode/slope')
        # while nav_mode_client.wait_for_service(timeout_sec=1.0) is False:
        #     self.get_logger().info('等待参数更新服务端上线！')
        # self.get_logger().info(f'上坡')
        # request = UseNavToPose.Request()
        # request.use_nav = 1
        # future = nav_mode_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future) # 等待服务端返回响应
        # response = future.result()
        # self.get_logger().info(f'{response}')


    def nav_pose(self,x,y,yaw_goal,adjust_angle):
        # navigator = BasicNavigator()
    
        # 等待导航启动完成
        self.waitUntilNav2Active()
    
        # ---------- 设置目标点坐标 ----------
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
    
        yaw = yaw_goal
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
    
        # 发送目标接收反馈结果
        self.goToPose(goal_pose)
    
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            # self.get_logger().info(
            #     f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
            # 超时自动取消
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.cancelTask()
    
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')

        if adjust_angle == True and result == TaskResult.SUCCEEDED:
            yaw_now = self.yaw
            self.adj_angle(yaw_now,yaw_goal)


def main():
    rclpy.init()
    node = NavToPose()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
