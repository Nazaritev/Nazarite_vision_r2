"""启动使用普通摄像头 0 的 YOLO 检测节点。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_detect_box',
            executable='yolo_detect_node',
            output='screen',
        ),
    ])
