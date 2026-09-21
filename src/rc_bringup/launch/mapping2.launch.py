import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
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
        config_dir, 'nav2_params_forest_small_teb_mppi.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
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
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file', default_value=default_nav2_params_path,
        description='Nav2 params file'
    )
    declare_bt_xml_file_cmd = DeclareLaunchArgument(
        'bt_xml_file', default_value=default_bt_xml_file,
        description='Behavior tree XML file for NavigateToPose'
    )

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_xml_file,
        },
        convert_types=True
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joint_state_publisher)
    )

    # Livox MID360 driver. Kept disabled by default to match mapping.launch.py.
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
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'point_lio.yaml'),
            {
                'use_sim_time': use_sim_time,
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'space_down_sample': True,
                'ivox_nearby_type': 6,
                'runtime_pos_log_enable': False,
            }
        ]
    )
    slope_filtered_node = Node(
        package='point_lio',
        executable='slope_filter_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # TF tree:
    #   map -> odom: static transform here
    #   odom -> base_link: Point-LIO odometry publisher
    #   base_link -> imu_link -> livox_frame: robot_state_publisher from model.xacro
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': 'true'
        }.items()
    )

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

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': use_sim_time}
        ]
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
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_bt_xml_file_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    # ld.add_action(livox_node)
    ld.add_action(nav2)
    ld.add_action(slope_filtered_node)
    ld.add_action(point_lio_node)
    ld.add_action(tf_map_to_odom)
    # ld.add_action(pc_to_ls_node)
    # ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld
