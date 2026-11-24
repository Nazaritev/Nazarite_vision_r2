#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os

class LidarPcdSaver(Node):
    def __init__(self):
        super().__init__('lidar_pcd_saver')
        
        # --- 配置区域 ---
        self.topic_name = '/cloud_accumulated'  # 你的话题名
        self.save_dir = './output_pcd'           # 保存路径
        self.save_intensity = True             # 是否保存强度
        # ----------------
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.listener_callback,
            10
        )
        self.count = 0
        self.get_logger().info(f'等待雷达数据: {self.topic_name} ...')

    def listener_callback(self, msg):
        try:
            # 1. 设定我们要提取的字段
            # 注意：这里我们明确指定只提取 float 兼容的数据
            field_names = ['x', 'y', 'z']
            if self.save_intensity:
                # 检查是否存在强度字段，通常叫 'intensity' 或 'i'
                all_fields = [f.name for f in msg.fields]
                if 'intensity' in all_fields:
                    field_names.append('intensity')
                elif 'i' in all_fields:
                    field_names.append('i')
            
            # 2. 从消息中读取点 (返回的是一个生成器)
            # 关键修改：我们需要把生成器里的元组(tuple)强制转成列表(list)，
            # 这样 numpy 才会把它识别为二维矩阵，而不是一维的结构体数组。
            cloud_generator = pc2.read_points(msg, field_names=field_names, skip_nans=True)
            
            # --- 修复核心：强制转换为纯 float 列表 ---
            # 直接 np.array(list(generator)) 可能会因为类型不一致(float vs int)导致变成一维结构体
            # 我们手动把所有数据都转成 float
            points_list = []
            for point in cloud_generator:
                points_list.append([float(x) for x in point])
            
            if not points_list:
                return

            # 3. 此时再转 numpy，绝对是二维矩阵 (N, 3) 或 (N, 4)
            points_np = np.array(points_list, dtype=np.float32)

            # --- 调试信息：如果报错，请把这两行取消注释查看形状 ---
            # print(f"Shape: {points_np.shape}") 
            
            # 4. 分离 XYZ
            # 只有当 shape 是 (N, M) 时，切片 [:, 0:3] 才是合法的
            if points_np.ndim != 2:
                self.get_logger().error(f"数据维度异常: {points_np.shape}，跳过此帧")
                return

            xyz = points_np[:, 0:3]
            
            # 5. 创建 Open3D 对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)

            # 6. 处理强度颜色 (如果存在第4列)
            if points_np.shape[1] >= 4:
                intensities = points_np[:, 3]
                # 归一化强度到 0-1 用于显示
                max_i = np.max(intensities)
                if max_i > 0:
                    intensities = intensities / max_i
                
                # 变成灰度色 (R=G=B)
                colors = np.stack((intensities, intensities, intensities), axis=-1)
                pcd.colors = o3d.utility.Vector3dVector(colors)

            # 7. 保存
            file_name = os.path.join(self.save_dir, f"lidar_{self.count:06d}.pcd")
            o3d.io.write_point_cloud(file_name, pcd, write_ascii=False)
            
            self.get_logger().info(f"已保存: {file_name} [点数: {len(xyz)}]")
            self.count += 1

        except Exception as e:
            # 打印详细错误方便调试
            self.get_logger().error(f"处理出错: {str(e)}")
            import traceback
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    saver = LidarPcdSaver()
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    finally:
        saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()