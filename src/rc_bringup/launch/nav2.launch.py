import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc_bringup')
    config_dir = os.path.join(pkg_dir, 'config', 'reality')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'reality', 'model.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    default_bt_xml_file = os.path.join(
        config_dir, 'navigate_to_pose_teb_mppi_selector.xml'
    )
    default_nav2_params_path = os.path.join(
        config_dir, 'nav2_use_map.yaml'
    )
    # 已保存的 2D 栅格地图
    default_map_path = '/home/dgut/code/Nazarite/src/rc_bringup/map/map2.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz if true'
    )
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        'use_joint_state_publisher', default_value='true',
        description='Start joint_state_publisher if true'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to map yaml file to load'
    )
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file', default_value=default_nav2_params_path,
        description='Nav2 params file'
    )
    declare_bt_xml_file_cmd = DeclareLaunchArgument(
        'bt_xml_file', default_value=default_bt_xml_file,
        description='Behavior tree XML file for NavigateToPose'
    )

    # 把地图路径和 BT 路径重写进 nav2 参数
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_xml_file,
            'map_server.ros__parameters.yaml_filename': map_yaml_file,
        },
        convert_types=True
    )

    # ---------------- 机器人描述 ----------------
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joint_state_publisher)
    )

    # ---------------- 底盘与里程计 ----------------
    odom_cmd_vel_fuser_node = Node(
        package='stm32_driver',
        executable='odom_cmd_vel_fuser',
        name='odom_cmd_vel_fuser',
        output='screen',
        parameters=[{
            'odom_in_topic': '/Odometry',
            'cmd_vel_topic': '/cmd_vel',
            'odom_out_topic': '/nav2_odom',
        }],
    )
    stm32_driver_node = Node(
        package='stm32_driver',
        executable='stm32_driver_node',
        name='stm32_driver',
        output='screen',
    )
    nav_agent_node = Node(
        package='stm32_driver',
        executable='nav2_agent',
        name='nav_agent',
        output='screen',
        parameters=[{
            'odom_topic': '/nav2_odom',
            'short_distance_controller': 'TEB_Short',
            'long_distance_controller': 'MPPI_Long',
            'controller_switch_distance': 0.50,
            'area1_to_area2_controller_switch_distance': 0.20,
            'controller_switch_hysteresis': 0.10,
            'grab_contact_arrival_enabled': False,
            'grab_contact_window_sec': 0.3,
            'grab_contact_position_epsilon': 0.02,
            'grab_contact_max_remaining': 0.04,
            'grab_contact_min_samples': 3,
            'grab_contact_require_preferred_side': True,
            'low_speed_arrival_enabled': True,
            'low_speed_arrival_vx_threshold': 0.05,
            'low_speed_arrival_vy_threshold': 0.05,
            'low_speed_arrival_max_remaining': 0.50,
        }],
    )

    # ---------------- 雷达与 FAST-LIO ----------------
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        parameters=[{
            'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
            'publish_freq': 100.0, 'output_data_type': 0, 'frame_id': 'livox_frame',
            'user_config_path': os.path.join(config_dir, 'MID360_driver.json')
        }]
    )
    point_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(config_dir, 'fast_lio.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    slope_filtered_node = Node(
        package='fast_lio',
        executable='slope_filter_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------- 点云压扁为 2D 激光 ----------------
    # AMCL 需要 /scan。这里用 FAST-LIO 的 /cloud_registered 生成 /scan
    pc_to_ls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link', 'transform_tolerance': 2.0,
            'min_height': -0.15, 'max_height': 4.0,
            'angle_min': -3.14159, 'angle_max': 3.14159,
            'angle_increment': 0.0043, 'scan_time': 0.3333,
            'range_min': 0.45, 'range_max': 10.0, 'use_inf': True, 'inf_epsilon': 1.0
        }]
    )

    # ---------------- Nav2 导航栈 ----------------
    # 使用 nav2_bringup 的 bringup_launch.py，它会同时启动
    # map_server + amcl + planner + controller + bt_navigator 等
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': configured_nav2_params,
            'autostart': 'true',
            'use_composition': 'False',
            'slam': 'False',          # 关闭 slam，改用 AMCL 定位
            'log_level': 'error',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_bt_xml_file_cmd)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(odom_cmd_vel_fuser_node)
    ld.add_action(stm32_driver_node)
    ld.add_action(livox_node)
    ld.add_action(point_lio_node)
    ld.add_action(slope_filtered_node)
    ld.add_action(pc_to_ls_node)

    # 注意：这里不再添加静态 map->odom TF，由 AMCL 动态发布
    ld.add_action(nav2)
    ld.add_action(nav_agent_node)
    ld.add_action(rviz_node)

    return ld
