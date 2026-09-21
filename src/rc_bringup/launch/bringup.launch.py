import os
from typing import Iterable, Text

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.frontend import expose_substitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.utilities import ensure_argument_type
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


@expose_substitution("read_dict")
class ReadDict(Substitution):
    def __init__(self, config: dict, keys: SomeSubstitutionsType) -> None:
        super().__init__()

        ensure_argument_type(config, dict, "config", "ReadConfig")

        from launch.utilities import normalize_to_list_of_substitutions

        self.__config = config
        self.__keys = normalize_to_list_of_substitutions(keys)

    @classmethod
    def parse(cls, data: Iterable[SomeSubstitutionsType]):
        raise "read_dict don't have parse method"

    @property
    def config(self) -> dict[Text, dict[Text, Substitution]]:
        return self.__config

    @property
    def keys(self) -> list[Text | Substitution]:
        return self.__keys

    def describe(self) -> Text:
        return "ReadConfig({} + {})".format(
            str(self.config), " + ".join([sub.describe() for sub in self.keys])
        )

    def perform(self, context: LaunchContext) -> Text:
        result = self.config
        for key in self.keys:
            result = result[context.perform_substitution(key)]
        return str(result)


def get_on_exit_action(node_name: str):
    return Shutdown(reason=f"{node_name} exited")


def generate_launch_description():
    # 声明启动参数
    environment = LaunchConfiguration("environment")
    world = LaunchConfiguration("world")
    loc_map = LaunchConfiguration("loc_map")
    initialpose_preset = LaunchConfiguration("initialpose_preset")
    active_region = LaunchConfiguration("active_region")
    use_relocalization = LaunchConfiguration("use_relocalization")
    # 获取配置文件路径
    use_sim_time = PythonExpression(["'", environment, "' != 'reality'"])
    package_dir = get_package_share_directory("rc_bringup")
    open3d_loc_dir = get_package_share_directory("open3d_loc")
    config_dir = PathJoinSubstitution([package_dir, "config"])

    nav2_params = RewrittenYaml(
        source_file=PathJoinSubstitution([config_dir, environment, "nav2.yaml"]),
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            # 'robot_base_frame': 'base_link_fake',
            "yaml_filename": (
                PathJoinSubstitution([package_dir, "map", world]),
                ".yaml",
            ),
        },
        convert_types=True,
    )
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([package_dir, "urdf", environment, "model.xacro"]),
        ]
    )

    # 声明启动参数
    declare_environment_cmd = DeclareLaunchArgument(
        "environment",
        default_value="reality",
        description="Choose mode: reality, simulation",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="RMUC2025",
        description="Select world (map file, pcd file, world file share the same name prefix as the this parameter)",
    )

    declare_loc_map_cmd = DeclareLaunchArgument(
        "loc_map",
        default_value="scans_1.9_filtered.pcd",
        description="Localization map file stored under rc_bringup/map",
    )

    declare_initialpose_preset_cmd = DeclareLaunchArgument(
        "initialpose_preset",
        default_value="start_origin",
        description="Initial pose preset name used by open3d_loc",
    )

    declare_active_region_cmd = DeclareLaunchArgument(
        "active_region",
        default_value="global",
        description="Localization crop/threshold profile used by open3d_loc",
    )

    declare_mode_cmd = DeclareLaunchArgument(
        "mode",
        default_value="mapping",
        description="Choose mode: mapping, nav",
    )

    declare_lio_cmd = DeclareLaunchArgument(
        "lio",
        default_value="fastlio",
        description="Choose lio alogrithm: fastlio, pointlio",
    )

    declare_use_relocalization_cmd = DeclareLaunchArgument(
        "use_relocalization",
        default_value="True",
        description="whether use relocalization when bring up",
    )

    # 声明启动节点
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    start_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    start_livox_ros_driver2 = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "xfer_format": 1
            },  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            {
                "multi_topic": 0
            },  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
            {"data_src": 0},  # 0-lidar, others-Invalid data src
            {"publish_freq": 100.0},  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"lvx_file_path": "/home/livox/livox_test.lvx"},
            {
                "user_config_path": PathJoinSubstitution(
                    [config_dir, environment, "MID360_driver.json"]
                )
            },
            {"cmdline_input_bd_code": "livox0000000001"},
        ],
    )

    start_fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        parameters=[
            PathJoinSubstitution([config_dir, environment, "fast_lio.yaml"]),
            {
                "use_sim_time": use_sim_time,
                "pcd_save.pcd_save_en": PythonExpression(
                    ["'", LaunchConfiguration("mode"), "' == 'mapping'"]
                ),
            },
        ],
        on_exit=get_on_exit_action("fast_lio"),
    )

    start_point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        parameters=[
            PathJoinSubstitution([config_dir, environment, "point_lio.yaml"]),
            {
                "use_sim_time": use_sim_time,
                "use_imu_as_input": False,  # Change to True to use IMU as input of Point-LIO
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                "point_filter_num": 3,  # Options: 1, 3
                "space_down_sample": True,
                "filter_size_surf": 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                "filter_size_map": 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
                "ivox_nearby_type": 6,  # Options: 0, 6, 18, 26
                "runtime_pos_log_enable": False,  # Option: True
                "pcd_save.pcd_save_en": PythonExpression(
                    ["'", LaunchConfiguration("mode"), "' == 'mapping'"]
                ),
            },
        ],
        on_exit=get_on_exit_action("point_lio"),
    )

    start_lio = GroupAction(
        [
            GroupAction(
                condition=LaunchConfigurationEquals("lio", "fastlio"),
                actions=[start_fast_lio],
            ),
            GroupAction(
                condition=LaunchConfigurationEquals("lio", "pointlio"),
                actions=[start_point_lio],
            ),
        ]
    )

    start_open3d_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(open3d_loc_dir, "launch", "open3d_loc_g1.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "loc_map": loc_map,
            "initialpose_preset": initialpose_preset,
            "active_region": active_region,
        }.items(),
    )

    start_static_map2odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    start_map2odom = GroupAction(
        [
            GroupAction(
                condition=IfCondition(use_relocalization),
                actions=[start_open3d_localization],
            ),
            GroupAction(
                condition=IfCondition(PythonExpression(["not ", use_relocalization])),
                actions=[start_static_map2odom],
            ),
        ]
    )

    start_slam_toolbox = Node(
        parameters=[
            PathJoinSubstitution([config_dir, "mapper_params_online_async.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        on_exit=get_on_exit_action("slam_toolbox"),
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(package_dir, "rviz", "default.rviz")],
        # arguments=['-d', os.path.join(package_dir, 'rviz', 'lio.rviz')],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nav2_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_nav2_container = Node(
        name="nav2_container",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[nav2_params, {"autostart": True}],
        remappings=nav2_remappings,
    )

    start_nav2_mapping = LoadComposableNodes(
        target_container="/nav2_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapSaver",
                name="map_saver",
                parameters=[nav2_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_mapping",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_saver"],
                    }
                ],
            ),
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="pointcloud_to_laserscan",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "target_frame": "base_link",
                        "transform_tolerance": 2.0,
                        "min_height": -2.0,
                        "max_height": 2.0,
                        "angle_min": -3.14159,  # -M_PI
                        "angle_max": 3.14159,  # M_PI
                        "angle_increment": 0.0043,  # M_PI/360.0
                        "scan_time": 0.3333,
                        "range_min": 0.45,
                        "range_max": 10.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                remappings=[("cloud_in", "/cloud_registered"), ("scan", "/scan")],
            ),
        ],
    )

    start_nav2_localization = LoadComposableNodes(
        target_container="/nav2_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[nav2_params],
                remappings=nav2_remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
        ],
    )


    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        ),
        launch_arguments={"gui_required": "true"}.items(),
    )

    world_configs = {
        "RMUL2024": {
            "x": "4.3",
            "y": "3.35",
            "z": "1.2",
            "yaw": "0.0",
            "world_path": "RMUL2024_world/RMUL2024_world.world",
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'
        },
        "RMUC2024": {
            "x": "6.35",
            "y": "7.6",
            "z": "0.2",
            "yaw": "0.0",
            "world_path": "RMUC2024_world/RMUC2024_world.world",
        },
        "RMUL2025": {
            "x": "-4.5",
            "y": "2.0",
            "z": "0.07",
            "yaw": "0.0",
            "world_path": "RMUL2025_world/RMUL2025_world.world",
        },
        "RMUC2025": {
            "x": "-10.115",
            "y": "-0.5115",
            "z": "0.06",
            "yaw": "0.0",
            "world_path": "RMUC2025_world/RMUC2025_world.world",
        },
    }

    start_gazebo_server = GroupAction(
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "robot",
                    "-topic",
                    "robot_description",
                    "-x",
                    ReadDict(world_configs, [world, "x"]),
                    "-y",
                    ReadDict(world_configs, [world, "y"]),
                    "-z",
                    ReadDict(world_configs, [world, "z"]),
                    "-Y",
                    ReadDict(world_configs, [world, "yaw"]),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("gazebo_ros"),
                        "launch",
                        "gzserver.launch.py",
                    )
                ),
                launch_arguments={
                    "world": PathJoinSubstitution(
                        [
                            package_dir,
                            "world",
                            ReadDict(world_configs, [world, "world_path"]),
                        ]
                    ),
                    "server_required": "true",
                }.items(),
            ),
        ]
    )

    start_gazebo = GroupAction(
        actions=[
            AppendEnvironmentVariable(
                "GAZEBO_PLUGIN_PATH",
                os.path.join(
                    package_dir, "meshes", "obstacles", "obstacle_plugin", "lib"
                ),
            ),
            start_gazebo_client,
            start_gazebo_server,
        ]
    )

    start_mapping = GroupAction(
        condition=LaunchConfigurationEquals("mode", "mapping"),
        actions=[
            GroupAction(
                actions=[
                    start_robot_state_publisher,
                    start_joint_state_publisher,
                    start_lio,
                    start_static_map2odom,
                    start_nav2_container,
                    start_nav2_mapping,
                    start_slam_toolbox,
                    # start_nav2_navigation,
                    start_rviz,
                ]
            ),
            GroupAction(
                condition=LaunchConfigurationEquals("environment", "reality"),
                actions=[
                    start_livox_ros_driver2,
                    # start_rm_serial_driver,
                ],
            ),
            GroupAction(
                condition=LaunchConfigurationEquals("environment", "simulation"),
                actions=[
                    start_gazebo,
                ],
            ),
        ],
    )

    start_nav = GroupAction(
        condition=LaunchConfigurationEquals("mode", "nav"),
        actions=[
            GroupAction(
                actions=[
                    start_robot_state_publisher,
                    start_joint_state_publisher,
                    start_lio,
                    start_map2odom,
                    start_nav2_container,
                    start_nav2_localization,
                    # start_nav2_navigation,
                    start_rviz,
                ]
            ),
            GroupAction(
                condition=LaunchConfigurationEquals("environment", "reality"),
                actions=[
                    start_livox_ros_driver2,
                    # start_rm_serial_driver,
                ],
            ),
            GroupAction(
                condition=LaunchConfigurationEquals("environment", "simulation"),
                actions=[
                    start_gazebo,
                ],
            ),
        ],
    )

    # 添加启动行为
    ld = LaunchDescription()

    ld.add_action(declare_environment_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_loc_map_cmd)
    ld.add_action(declare_initialpose_preset_cmd)
    ld.add_action(declare_active_region_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lio_cmd)
    ld.add_action(declare_use_relocalization_cmd)

    ld.add_action(start_mapping)
    ld.add_action(start_nav)

    return ld
