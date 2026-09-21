import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc_bringup')
    config_dir = os.path.join(pkg_dir, 'config', 'reality')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'reality', 'model.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    loc_map = LaunchConfiguration('loc_map')
    initialpose_preset = LaunchConfiguration('initialpose_preset')
    active_region = LaunchConfiguration('active_region')
    registration_method = LaunchConfiguration('registration_method')
    start_livox = LaunchConfiguration('start_livox')
    start_rviz = LaunchConfiguration('start_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_loc_map_cmd = DeclareLaunchArgument(
        'loc_map',
        default_value='scans_1.9_filtered_half_std_best_for_loc.pcd',
        description='Localization map file under rc_bringup/map'
    )
    declare_initialpose_preset_cmd = DeclareLaunchArgument(
        'initialpose_preset',
        default_value='start_origin',
        description='Initial pose preset defined in rc_bringup/config/reality/relocalization_presets.yaml'
    )
    declare_active_region_cmd = DeclareLaunchArgument(
        'active_region',
        default_value='global',
        description='Localization runtime profile defined in rc_bringup/config/reality/relocalization_presets.yaml'
    )
    declare_registration_method_cmd = DeclareLaunchArgument(
        'registration_method',
        default_value='NDT_OMP',
        description='Registration backend used by lidar_localization_ros2'
    )
    declare_start_livox_cmd = DeclareLaunchArgument(
        'start_livox',
        default_value='true',
        description='Start Livox MID360 driver'
    )
    declare_start_rviz_cmd = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    livox_node = Node(
        condition=IfCondition(start_livox),
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 100.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': os.path.join(config_dir, 'MID360_driver.json')
        }]
    )

    slope_filtered_node = Node(
        package='fast_lio',
        executable='slope_filter_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(config_dir, 'fast_lio.yaml'),
            {
                'use_sim_time': use_sim_time,
                'pcd_save.pcd_save_en': False
            }
        ]
    )

    relocalization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'relocalization_rsasaki.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'loc_map': loc_map,
            'initialpose_preset': initialpose_preset,
            'active_region': active_region,
            'registration_method': registration_method,
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_loc_map_cmd)
    ld.add_action(declare_initialpose_preset_cmd)
    ld.add_action(declare_active_region_cmd)
    ld.add_action(declare_registration_method_cmd)
    ld.add_action(declare_start_livox_cmd)
    ld.add_action(declare_start_rviz_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    # ld.add_action(livox_node)
    ld.add_action(slope_filtered_node)
    ld.add_action(fast_lio_node)
    ld.add_action(relocalization_node)
    ld.add_action(rviz_node)
    return ld
