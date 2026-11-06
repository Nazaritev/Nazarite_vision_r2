import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    IncludeLaunchDescription, 
)
import launch_ros
import launch
from launch_ros.substitutions import FindPackageShare
import math
from launch.launch_description_sources import PythonLaunchDescriptionSource

def build_nav2_params(config_dir, use_sim_time, enable_gripper_filter,
                      costmap_scan_topic='/scan_costmap'):
    filtering_inactive = PythonExpression([
        "'", enable_gripper_filter, "'.lower() not in ('true', '1')",
    ])
    observation_sources = PythonExpression([
        "'scan gripper_marking gripper_clearing' if '",
        enable_gripper_filter,
        "'.lower() in ('true', '1') else 'scan'",
    ])
    nav2_params = [
        ParameterFile(
            RewrittenYaml(
                source_file=os.path.join(config_dir, 'nav2_params_forest_small_teb_mppi.yaml'),
                param_rewrites={
                    'use_sim_time': use_sim_time,
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.observation_sources': observation_sources,
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.marking': filtering_inactive,
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.clearing': filtering_inactive,
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.gripper_marking.marking': enable_gripper_filter,
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.gripper_marking.topic': costmap_scan_topic,
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.gripper_marking.clearing': 'false',
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.gripper_clearing.marking': 'false',
                    'local_costmap.local_costmap.ros__parameters.obstacle_layer.gripper_clearing.clearing': enable_gripper_filter,
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.marking': filtering_inactive,
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.observation_sources': observation_sources,
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.clearing': filtering_inactive,
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.gripper_marking.marking': enable_gripper_filter,
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.gripper_marking.topic': costmap_scan_topic,
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.gripper_marking.clearing': 'false',
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.gripper_clearing.marking': 'false',
                    'global_costmap.global_costmap.ros__parameters.obstacle_layer.gripper_clearing.clearing': enable_gripper_filter,
                },
                convert_types=True,
            ),
            allow_substs=True,
        )
    ]
    nav2_params.append(
        ParameterFile(
            RewrittenYaml(
                source_file=os.path.join(config_dir, 'mapping_nav_copy.yaml'),
                param_rewrites={'use_sim_time': use_sim_time},
                convert_types=True,
            ),
            allow_substs=True,
        )
    )
    return nav2_params


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc_bringup')
    config_dir = os.path.join(pkg_dir, 'config', 'reality')
    map_yaml_file = os.path.join(pkg_dir, 'map', 'map4.yaml')
    bt_xml_file = os.path.join(
        config_dir, 'navigate_to_pose_teb_mppi_selector.xml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_gripper_filter = LaunchConfiguration('enable_gripper_filter')
    filter_input_topic = LaunchConfiguration('filter_input_topic')
    filtered_cloud_topic = LaunchConfiguration('filtered_cloud_topic')
    costmap_scan_topic = LaunchConfiguration('costmap_scan_topic')
    nav2_params = build_nav2_params(
        config_dir, use_sim_time, enable_gripper_filter, costmap_scan_topic)

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('filter_input_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument('filtered_cloud_topic', default_value='/cloud_registered_costmap'),
        DeclareLaunchArgument('costmap_scan_topic', default_value='/scan_costmap'),
        DeclareLaunchArgument(
            'enable_gripper_filter',
            default_value='true',
            choices=['true', 'false'],
            description='Enable the measured gripper self-filtered costmap scan',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ',
                    os.path.join(pkg_dir, 'urdf', 'reality', 'model.xacro'),
                ]),
            }],
        ),
        Node(
            package='stm32_driver',
            executable='odom_cmd_vel_fuser',
            name='odom_cmd_vel_fuser',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_in_topic': '/Odometry',
                'cmd_vel_topic': '/cmd_vel',
                'odom_out_topic': '/nav2_odom',
            }],
        ),
        Node(
            package='stm32_driver',
            executable='stm32_driver_node',
            name='stm32_driver',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            output='screen',
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 40.0, # 20.0
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': os.path.join(
                    config_dir, 'MID360_driver.json'
                ),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'fast_lio.yaml'),
                {'use_sim_time': use_sim_time, 'pcd_save.pcd_save_en': False},
            ],
        ),
    ]

    localized_actions = [
        Node(
            package='rc_bringup',
            executable='gripper_pointcloud_filter.py',
            name='gripper_pointcloud_filter',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'gripper_filter.yaml'),
                {'use_sim_time': use_sim_time, 'enabled': enable_gripper_filter,
                 'input_topic': filter_input_topic, 'output_topic': filtered_cloud_topic},
            ],
            condition=IfCondition(enable_gripper_filter),
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_costmap',
            output='screen',
            remappings=[
                ('cloud_in', filtered_cloud_topic),
                ('scan', costmap_scan_topic),
            ],
            parameters=[
                os.path.join(config_dir, 'gripper_filter.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            condition=IfCondition(enable_gripper_filter),
        ),
        # 堆叠检测：订阅 /cloud_registered（odom 系），在 map 系下查询若干点，
        # 发布 /block_stacking/markers、/block_stacking/corner_cloud、
        # /block_stacking/layers（每个查询点的层数）
        # ★ points = [x1,y1, x2,y2, ...]，map 系坐标，务必按场地改
        # ★ 雷达倒装：z 轴朝下，地面在 z = ground_z（0.55，即雷达离地高度），
        #   物体高度 h = ground_z - z；layer_height=0.35 表示 h>0.35 算一层
        # ★ 没有定位（无 map->odom）时把 target_frame 改成 'odom'
        Node(
            package='fast_lio',
            executable='block_stacking_detection',
            name='block_stacking_detection',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cloud_topic': '/cloud_registered',
                'target_frame': 'map',
                 'tf_timeout': 0.2,
                'ground_z': -0.73,
                'inverted': False,
                'layer_height': 0.35,
                'roi_half_size': 0.30,
                'max_stack_height': 2.0,
                'top_percentile': 0.99,
                'min_points': 10,
                'process_period': 0.2,
                'accumulate_window': 0.2,   # 累积最近 0.2s 的多帧点云再统计
                'outlier_mean_k': 8,        # 小点集的 SOR，不能再大
                'outlier_stddev': 1.0,
                'point_names': ['corner_1', 'corner_2', 'corner_3', 'corner_4','corner_5','corner_6','corner_7','corner_8'],
                'points': [
                    5.04, -2.47,   # corner_1
                     3.37, -2.64,   # corner_2
                     3.46,  0.06,   # corner_3
                    5.79,  -0.05,   # corner_4
                    7.09, -1.66,   # corner_5
                     2.02, -1.48,   # corner_6
                     7.23,  1.30,   # corner_7   
                    1.96,  1.43,   # corner_8
                ],
            }],
        ),
    ]

    actions.extend([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'footprint_link',
                'transform_tolerance': 0.1,
                'min_height': -0.70,
                'max_height': 4.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.05,
                'range_min': 0.30,# 0.65 过滤了夹爪
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=nav2_params + [{'yaml_filename': map_yaml_file}],
        ),
        #Node(
        #    package='rc_bringup',
        #    executable='amcl_pose_counter.py',
        #    name='amcl_pose_counter',
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=nav2_params,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])

    localized_actions.append(
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node', # 注意：请确认它的可执行文件名，通常是这个
            name='pointcloud_to_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cloud_in_topic': '/cloud_registered', # 输入点云话题
                'z_min': -2.0  ,                   # 高度筛选下限
                'z_max':2.0  ,                     # 高度筛选上限.
                'cell_size': 0.05 ,
                # height_grid 的数值换算：value = (z + height_offset) * height_factor
                # OccupancyGrid 只有 0..100（灰度），必须把关心的地面以上高度摊满
                # 整个量程，否则所有格子都是 0/1，图上就是一片同色。
                # ground 在 map 系约 -0.55m（base_link 比 motion_link 高 0.576m），
                # 所以 offset=0.55 让地面→0；factor=100 让地面以上 1m→100。
                'height_offset': -0.68,
                'height_factor': 150.0,
                # 自车（车身）点剔除：框在 base_link 下（x 前  y 左  z 上），
                # 默认 = 雷达左后方 0.5m x 0.5m（雷达在车身右上角）。
                'ego_filter_enable': True,
                'ego_frame': 'base_link',
                'ego_x_min': -0.5, 'ego_x_max': 0.0,
                'ego_y_min': 0.0,  'ego_y_max': 0.5,
            }],
        ),
    )

    navigation_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    for package, executable in navigation_nodes:
        parameters = list(nav2_params)
        remappings = []
        if executable in ('controller_server', 'behavior_server'):
            remappings = [('cmd_vel', '/cmd_vel_nav')]
        elif executable == 'velocity_smoother':
            remappings = [
                ('cmd_vel', '/cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel'),
            ]
        elif executable == 'bt_navigator':
            parameters.append({'default_nav_to_pose_bt_xml': bt_xml_file})
        localized_actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ))

    action1= launch_ros.actions.Node(
            package='run_keyboard_test',
            executable='nav_to_pose',
            output='screen'
        )
    action2= launch_ros.actions.Node(
            package='run_keyboard_test',
            executable='run_slope',
            output='screen'
        )
    action_group = launch.actions.GroupAction([
        # 动作5-定时器
        launch.actions.TimerAction(period=3.0, actions=[action1]), #2秒后执行
        # launch.actions.TimerAction(period=3.0, actions=[action2]) #4秒后执行
        ])

    localized_actions.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    executable for package, executable in navigation_nodes
                ],
            }],
        ),
    )

    localization_ready = Node(
        package='rc_bringup',
        executable='wait_for_localization.py',
        name='wait_for_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'map',
            'source_frame': 'footprint_link',
        }],
    )

    def start_localized_nodes(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            launch.logging.get_logger('mapping_nav2').error(
                '定位 TF 等待进程异常退出，未启动导航和堆叠检测。'
            )
            return []
        return localized_actions

    actions.extend([
        RegisterEventHandler(OnProcessExit(
            target_action=localization_ready,
            on_exit=start_localized_nodes,
        )),
        localization_ready,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'default.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
    return LaunchDescription(actions + [action_group])
    #return LaunchDescription(actions)
