from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')
    rc_bringup_share = FindPackageShare('rc_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    loc_map_arg = DeclareLaunchArgument(
        'loc_map',
        default_value='scans_1.9_filtered_half_std_best_for_loc.pcd',
        description='Localization map file name under rc_bringup/map'
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
        loc_map_arg,
        initialpose_preset_arg,
        active_region_arg,
        global_localization_node,
        # pointcloud_transformer_node
    ])

