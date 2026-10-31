from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')
    rc_bringup_share = FindPackageShare('rc_bringup')
    fast_lio_share = FindPackageShare('fast_lio')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    start_fast_lio_arg = DeclareLaunchArgument(
        'start_fast_lio',
        default_value='true',
        description='Start FAST-LIO to provide the point cloud and odometry'
    )

    start_livox_arg = DeclareLaunchArgument(
        'start_livox',
        default_value=LaunchConfiguration('start_fast_lio'),
        description='Start the MID360 driver; disable for bags or an existing driver'
    )

    start_robot_state_publisher_arg = DeclareLaunchArgument(
        'start_robot_state_publisher',
        default_value=LaunchConfiguration('start_fast_lio'),
        description='Publish the robot mount transforms required by FAST-LIO'
    )

    robot_description_file_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=PathJoinSubstitution([
            rc_bringup_share, 'urdf', 'reality', 'model.xacro'
        ]),
        description='Robot description containing the base, IMU and LiDAR transforms'
    )

    livox_config_arg = DeclareLaunchArgument(
        'livox_config',
        default_value=PathJoinSubstitution([
            rc_bringup_share, 'config', 'reality', 'MID360_driver.json'
        ]),
        description='MID360 network configuration file'
    )

    fast_lio_config_arg = DeclareLaunchArgument(
        'fast_lio_config',
        default_value=PathJoinSubstitution([
            fast_lio_share, 'config', 'mid360.yaml'
        ]),
        description='FAST-LIO parameter file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Show the localization result in RViz'
    )

    loc_map_arg = DeclareLaunchArgument(
        'loc_map',
        default_value='laser_map.pcd',
        description='Absolute map path or file name under rc_bringup/map'
    )

    initialpose_preset_arg = DeclareLaunchArgument(
        'initialpose_preset',
        default_value='start_origin',
        description='Initial pose preset name defined in loc_param_g1.yaml'
    )

    active_region_arg = DeclareLaunchArgument(
        'active_region',
        default_value='global',
        description='Localization crop/threshold profile defined in loc_param_g1.yaml'
    )

    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_g1.yaml'
    ])

    map_file = PathJoinSubstitution([
        rc_bringup_share,
        'map',
        LaunchConfiguration('loc_map')
    ])

    robot_state_publisher_node = Node(
        condition=IfCondition(LaunchConfiguration('start_robot_state_publisher')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command([
                'xacro ', LaunchConfiguration('robot_description_file')
            ]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    livox_node = Node(
        condition=IfCondition(LaunchConfiguration('start_livox')),
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': LaunchConfiguration('livox_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    fast_lio_node = Node(
        condition=IfCondition(LaunchConfiguration('start_fast_lio')),
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[
            LaunchConfiguration('fast_lio_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'pcd_save.pcd_save_en': False
            }
        ]
    )

    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                'loc_map': LaunchConfiguration('loc_map'),
                'initialpose_preset': LaunchConfiguration('initialpose_preset'),
                'active_region': LaunchConfiguration('active_region'),
                'path_map': map_file,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            open3d_loc_share, 'rviz_cfg', 'fastlio.rviz'
        ])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    pointcloud_transformer_node = Node(
        package='open3d_loc',
        executable='pointcloud_transformer_node',
        name='pointcloud_transformer_node',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered_body',
            'output_topic': '/cloud_registered_map',
            'global_map_topic': '/global_map',
            'source_frame': 'imu_link',
            'target_frame': 'map',
            'voxel_leaf_size': 0.1,
            'map_voxel_leaf_size': 0.2,
            'max_global_points': 1000000,
            'map_publish_frequency': 1.0,
            'enable_global_map': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_fast_lio_arg,
        start_livox_arg,
        start_robot_state_publisher_arg,
        robot_description_file_arg,
        livox_config_arg,
        fast_lio_config_arg,
        rviz_arg,
        loc_map_arg,
        initialpose_preset_arg,
        active_region_arg,
        robot_state_publisher_node,
        livox_node,
        fast_lio_node,
        global_localization_node,
        rviz_node,
        # pointcloud_transformer_node
    ])

