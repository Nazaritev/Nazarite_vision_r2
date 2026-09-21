import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _normalize_pose(raw_pose, fallback_pose):
    if isinstance(raw_pose, list) and len(raw_pose) == 6:
        return raw_pose
    return fallback_pose


def _load_relocalization_presets():
    package_dir = get_package_share_directory("rc_bringup")
    config_path = os.path.join(
        package_dir, "config", "reality", "relocalization_presets.yaml"
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file) or {}
    return document.get("relocalization", {}).get("ros__parameters", {})


def _region_profile(active_region: str):
    profiles = {
        "global": {
            "score_threshold": "6.0",
            "ndt_resolution": "0.35",
            "voxel_leaf_size": "0.06",
            "scan_min_range": "0.5",
            "scan_max_range": "6.4",
            "local_map_radius": "7.5",
            "min_scan_interval_sec": "0.08",
        },
        "key_zone": {
            "score_threshold": "5.0",
            "ndt_resolution": "0.30",
            "voxel_leaf_size": "0.05",
            "scan_min_range": "0.5",
            "scan_max_range": "5.4",
            "local_map_radius": "6.0",
            "min_scan_interval_sec": "0.08",
        },
    }
    return profiles.get(active_region, profiles["global"])


def _build_localization(context):
    package_dir = get_package_share_directory("rc_bringup")
    localizer_dir = get_package_share_directory("lidar_localization_ros2")

    loc_map = LaunchConfiguration("loc_map").perform(context)
    initialpose_preset = LaunchConfiguration("initialpose_preset").perform(context)
    active_region = LaunchConfiguration("active_region").perform(context)
    registration_method = LaunchConfiguration("registration_method").perform(context)

    relocalization_params = _load_relocalization_presets()
    default_pose = _normalize_pose(
        relocalization_params.get("initialpose_default", [0.0] * 6), [0.0] * 6
    )
    preset_pose = _normalize_pose(
        relocalization_params.get("initialpose_presets", {}).get(initialpose_preset, default_pose),
        default_pose,
    )
    region = relocalization_params.get("regions", {}).get(
        active_region, _region_profile(active_region)
    )
    qx, qy, qz, qw = _euler_to_quaternion(
        float(preset_pose[3]),
        float(preset_pose[4]),
        float(preset_pose[5]),
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localizer_dir, "launch", "nazarite_localization.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "localization_param_dir": os.path.join(
                    localizer_dir, "param", "nazarite_mid360_half_field.yaml"
                ),
                "cloud_topic": "/cloud_registered",
                "odom_topic": "/Odometry",
                "imu_topic": "/livox/imu",
                "map_path": os.path.join(package_dir, "map", loc_map),
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "enable_map_odom_tf": "true",
                "set_initial_pose": "true",
                "initial_pose_x": str(float(preset_pose[0])),
                "initial_pose_y": str(float(preset_pose[1])),
                "initial_pose_z": str(float(preset_pose[2])),
                "initial_pose_qx": str(qx),
                "initial_pose_qy": str(qy),
                "initial_pose_qz": str(qz),
                "initial_pose_qw": str(qw),
                "registration_method": registration_method,
                "score_threshold": str(region["score_threshold"]),
                "ndt_resolution": str(region["ndt_resolution"]),
                "voxel_leaf_size": str(region["voxel_leaf_size"]),
                "scan_min_range": str(region["scan_min_range"]),
                "scan_max_range": str(region["scan_max_range"]),
                "local_map_radius": str(region["local_map_radius"]),
                "min_scan_interval_sec": str(region["min_scan_interval_sec"]),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "loc_map",
                default_value="scans_1.9_filtered_half_std_best_for_loc.pcd",
                description="Localization map file stored under rc_bringup/map",
            ),
            DeclareLaunchArgument(
                "initialpose_preset",
                default_value="start_origin",
                description="Initial pose preset name from rc_bringup/config/reality/relocalization_presets.yaml",
            ),
            DeclareLaunchArgument(
                "active_region",
                default_value="global",
                description="Runtime tuning profile from rc_bringup/config/reality/relocalization_presets.yaml",
            ),
            DeclareLaunchArgument(
                "registration_method",
                default_value="NDT_OMP",
                description="Registration backend for lidar_localization_ros2",
            ),
            OpaqueFunction(function=_build_localization),
        ]
    )
