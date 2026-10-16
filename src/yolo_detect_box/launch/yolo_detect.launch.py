"""启动使用指定普通摄像头的 YOLO 检测节点。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_index', default_value='0',
            description='OpenCV 摄像头编号',
        ),
        DeclareLaunchArgument(
            'conf', default_value='0.6',
            description='检测置信度阈值（0 到 1），每帧最多保留一个框',
        ),
        Node(
            package='yolo_detect_box',
            executable='yolo_detect_node',
            output='screen',
            parameters=[{
                'camera_index': ParameterValue(
                    LaunchConfiguration('camera_index'), value_type=int),
                'conf': ParameterValue(LaunchConfiguration('conf'), value_type=float),
            }],
        ),
    ])
