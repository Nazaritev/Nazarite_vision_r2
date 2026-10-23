"""启动 YOLO 和直接串口前后、横移控制，不启动其他串口写入节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 每个参数显式声明类型，避免把数字和布尔值当作字符串。
    detector = {
        'camera_index': ('0', int, 'OpenCV 摄像头编号'),
        'conf': ('0.6', float, '检测置信度阈值'),
        'missing_timeout': ('1.0', float, '首次识别后连续丢失多久才返回完成状态 1，秒'),
        'camera_timeout': ('2.0', float, '连续读图失败的超时，秒'),
        'show_window': ('true', bool, '是否显示检测窗口'),
    }
    alignment = {
        'center_left_ratio': ('0.55', float, '到位范围左边界占图像宽度的比例'),
        'center_right_ratio': ('0.65', float, '到位范围右边界占图像宽度的比例'),
        'box_height_min_ratio': ('0.35', float, '到位时框高占图像高度的最小比例'),
        'box_height_max_ratio': ('0.55', float, '到位时框高占图像高度的最大比例'),
    }
    controller = {
        'slope_topic': ('/slope', str, '外部触发话题'),
        'trigger_value': ('3', int, '启动一轮视觉平移的 slope 值'),
        'serial_port': ('/dev/ttyUSB0', str, '底盘串口，需由本控制节点独占写入'),
        'baudrate': ('961200', int, '沿用现有速度协议的波特率'),
        'max_lateral_speed': ('0.3', float, '最大横移速度，m/s，硬限制不超过 0.3'),
        'lateral_gain': ('1.0', float, '水平归一化偏差到横移速度的比例增益'),
        'enable_forward_control': ('true', bool, '前视相机按框高比例调整前后距离'),
        'max_forward_speed': ('0.3', float, '最大前后速度，m/s，硬限制不超过 0.3'),
        'forward_gain': ('1.0', float, '框高比例偏差到前后速度的比例增益'),
        'forward_direction': ('1.0', float, '框偏小时前进对应 Vx 的符号，取 -1 或 1'),
        'lock_on_center': ('true', bool, '连续到位后锁车并保持零速度，直至 YOLO 确认目标消失'),
        'lateral_direction': ('-1.0', float, '水平偏差到 Vy 的方向，取 -1 或 1'),
        'control_rate': ('20.0', float, '速度帧发送频率，Hz'),
        'detection_timeout': ('0.5', float, '检测结果过期多久停车，秒'),
        'service_wait_timeout': ('5.0', float, '等待 YOLO 服务就绪的超时，秒'),
        'stop_duration': ('0.5', float, '服务结束后额外发送零速度的时长，秒'),
    }
    shared = {
        'detection_topic': ('/yolo/box_detection', str, '实时检测结果话题'),
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, _, description) in {
            **detector, **alignment, **controller, **shared
        }.items()
    ]

    def parameters(specification):
        return {
            name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
            for name, (_, value_type, _) in specification.items()
        }

    return LaunchDescription(arguments + [
        Node(
            package='yolo_detect_box', executable='yolo_detect_node', output='screen',
            parameters=[parameters({**detector, **alignment, **shared})]),
        Node(
            package='run_keyboard_test', executable='yolo_control', output='screen',
            parameters=[parameters({**alignment, **controller, **shared})]),
    ])
