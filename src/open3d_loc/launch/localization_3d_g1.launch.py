from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    # The shared entry point starts FAST-LIO, ICP localization and RViz.
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'open3d_loc_g1.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        open3d_loc_launch,
    ])
