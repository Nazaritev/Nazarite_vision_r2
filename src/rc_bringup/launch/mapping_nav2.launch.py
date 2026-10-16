import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc_bringup')
    config_dir = os.path.join(pkg_dir, 'config', 'reality')
    map_yaml_file = os.path.join(pkg_dir, 'map', 'map2.yaml')
    bt_xml_file = os.path.join(
        config_dir, 'navigate_to_pose_teb_mppi_selector.xml'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    nav2_params = [
        ParameterFile(
            RewrittenYaml(
                source_file=os.path.join(config_dir, filename),
                param_rewrites={'use_sim_time': use_sim_time},
                convert_types=True,
            ),
            allow_substs=True,
        )
        for filename in (
            'nav2_params_forest_small_teb_mppi.yaml',
            'mapping_nav.yaml',
        )
    ]

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
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
                'publish_freq': 20.0,
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
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_link',
                'transform_tolerance': 2.0,
                'min_height': -0.15,
                'max_height': 4.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.05,
                'range_min': 0.45,
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
        Node(
            package='rc_bringup',
            executable='amcl_pose_counter.py',
            name='amcl_pose_counter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
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
    ]

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
        actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=parameters,
            remappings=remappings,
        ))

    actions.extend([
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
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'default.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
    return LaunchDescription(actions)