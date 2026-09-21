import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'arm-new23'
    
    # 获取 urdf 文件路径
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'arm-new23.urdf')
    pkg_share = get_package_share_directory(pkg_name)
    # 读取 URDF 内容作为字符串
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'default.rviz')
    # 节点：robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # 在这里传入 robot_description
        parameters=[{'robot_description': robot_desc}]
    )

    # 节点：joint_state_publisher_gui
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # 必须也在这里传入 robot_description，否则读不到关节信息！
        parameters=[{'robot_description': robot_desc}]
    )

    # 节点：rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
