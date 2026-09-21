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
    pkg_dir = get_package_share_directory("rc_bringup")
    config_dir = os.path.join(pkg_dir, "config", "reality")
    urdf_file = os.path.join(
        pkg_dir, "urdf", "reality", "combined_geometry_new23.xacro"
    )
    rviz_file = os.path.join(pkg_dir, "rviz", "navigation_combined_new23.rviz")
    default_bt_xml_file = os.path.join(
        config_dir, "navigate_to_pose_teb_mppi_selector.xml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_pointcloud_to_laserscan = LaunchConfiguration("use_pointcloud_to_laserscan")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    arm_mount_x = LaunchConfiguration("arm_mount_x")
    arm_mount_y = LaunchConfiguration("arm_mount_y")
    arm_mount_z = LaunchConfiguration("arm_mount_z")
    arm_mount_roll = LaunchConfiguration("arm_mount_roll")
    arm_mount_pitch = LaunchConfiguration("arm_mount_pitch")
    arm_mount_yaw = LaunchConfiguration("arm_mount_yaw")

    default_nav2_params_path = os.path.join(
        config_dir, "nav2_params_forest_small_teb_mppi.yaml"
    )

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": bt_xml_file,
        },
        convert_types=True,
    )

    robot_description = Command(
        [
            "xacro ",
            urdf_file,
            " arm_mount_x:=",
            arm_mount_x,
            " arm_mount_y:=",
            arm_mount_y,
            " arm_mount_z:=",
            arm_mount_z,
            " arm_mount_roll:=",
            arm_mount_roll,
            " arm_mount_pitch:=",
            arm_mount_pitch,
            " arm_mount_yaw:=",
            arm_mount_yaw,
        ]
    )

    launch_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_pointcloud_to_laserscan", default_value="true"),
        DeclareLaunchArgument("arm_mount_x", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_y", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_z", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_roll", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_pitch", default_value="0.0"),
        DeclareLaunchArgument("arm_mount_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params_path,
            description="Nav2 params file",
        ),
        DeclareLaunchArgument(
            "bt_xml_file",
            default_value=default_bt_xml_file,
            description="Behavior tree XML file for NavigateToPose",
        ),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_joint_state_publisher),
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        parameters=[
            os.path.join(config_dir, "fast_lio.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    slope_filtered_node = Node(
        package="fast_lio",
        executable="slope_filter_node",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_nav2_params,
            "autostart": "true",
            "log_level": "error",
        }.items(),
        condition=IfCondition(use_nav2),
    )

    pc_to_ls_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/cloud_registered"), ("scan", "/scan")],
        parameters=[
            {
                "target_frame": "base_link",
                "transform_tolerance": 2.0,
                "min_height": 0.15,
                "max_height": 0.60,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0043,
                "scan_time": 0.3333,
                "range_min": 0.45,
                "range_max": 10.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        condition=IfCondition(use_pointcloud_to_laserscan),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    for action in launch_arguments:
        ld.add_action(action)

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(slope_filtered_node)
    ld.add_action(fast_lio_node)
    ld.add_action(tf_map_to_odom)
    ld.add_action(pc_to_ls_node)
    ld.add_action(nav2)
    ld.add_action(rviz_node)

    return ld
