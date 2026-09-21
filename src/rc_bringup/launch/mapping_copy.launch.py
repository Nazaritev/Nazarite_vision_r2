import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rc_bringup')
    config_dir = os.path.join(pkg_dir, 'config', 'reality')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'reality', 'model.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    joint_state_publisher = Node(package='joint_state_publisher', executable='joint_state_publisher')

    #  雷达驱动 (Livox MID360)
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        parameters=[{
            'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
            'publish_freq': 100.0, 'output_data_type': 0, 'frame_id': 'livox_frame',
            'user_config_path': os.path.join(config_dir, 'MID360_driver.json')
        }]
    )

    #  Fast-LIO 3D里程计 
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[os.path.join(config_dir, 'fast_lio.yaml')]
    )
    slope_filtered_node = Node(
        package='fast_lio',
        executable='slope_filter_node',
    )

    # 静态 TF 
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    #  3D点云压扁成 2D 激光雷达 (用于供 slam_toolbox 生成 2D 地图)
    pc_to_ls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/cloud_obstacles'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link', 'transform_tolerance': 2.0,
            'min_height': 0.15, 'max_height': 0.60,
            'angle_min': -3.14159, 'angle_max': 3.14159,
            'angle_increment': 0.0043, 'scan_time': 0.3333,
            'range_min': 0.45, 'range_max': 10.0, 'use_inf': True, 'inf_epsilon': 1.0
        }]
    )

    # 2D 栅格地图生成器
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')]
    )
    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file])


    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    # ld.add_action(livox_node)
    ld.add_action(slope_filtered_node)
    ld.add_action(fast_lio_node)
    ld.add_action(tf_map_to_odom)
    ld.add_action(pc_to_ls_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld
