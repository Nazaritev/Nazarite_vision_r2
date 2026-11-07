"""Stationary 3D localization test. This entry point starts no motion programs."""

import os
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _test_chain(context):
    share = get_package_share_directory('rc_bringup')
    reality = os.path.join(share, 'config', 'reality')
    map_file = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('map_file').perform(context)))
    params_file = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('localization_params_file').perform(context)))
    for name, path in [('map_file', map_file), ('localization_params_file', params_file)]:
        if not os.path.isfile(path):
            raise RuntimeError(f'{name} is not a readable file: {path}')

    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    overrides = {
        'use_sim_time': clock,
        'map_file': map_file,
        'relocalization_backend': ParameterValue(
            LaunchConfiguration('relocalization_backend'), value_type=str),
    }
    for component in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        name = f'initial_pose_{component}'
        overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=float)

    localizer = Node(
        package='rc_cloud_localization', executable='cloud_localizer',
        name='cloud_localizer', output='screen',
        parameters=[params_file, overrides],
    )
    test_chain = [
        LogInfo(msg=(
            'Cloud localization test: stop AMCL/other map->odom publishers yourself. '
            'Defaults reuse existing lidar, FAST-LIO and robot_state_publisher. '
            'No Nav2, tasks, keyboard, serial controller or velocity publisher is started.')),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='cloud_test_robot_state_publisher', output='screen',
            condition=IfCondition(LaunchConfiguration('start_robot_state_publisher')),
            parameters=[{
                'use_sim_time': clock,
                'robot_description': ParameterValue(Command([
                    'xacro ', os.path.join(share, 'urdf', 'reality', 'model.xacro'),
                ]), value_type=str),
            }],
        ),
        Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='cloud_test_livox', output='screen',
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            parameters=[{
                'use_sim_time': clock, 'xfer_format': 1, 'multi_topic': 0,
                'data_src': 0, 'publish_freq': 40.0, 'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': os.path.join(reality, 'MID360_driver.json'),
            }],
        ),
        Node(
            package='fast_lio', executable='fastlio_mapping',
            name='cloud_test_fastlio', output='screen',
            condition=IfCondition(LaunchConfiguration('start_lio')),
            parameters=[os.path.join(reality, 'fast_lio.yaml'), {
                'use_sim_time': clock,
                'pcd_save.pcd_save_en': False,
                'map_file_path': '',
                'publish.scan_publish_en': True,
            }],
        ),
        # A fatal localizer configuration error also closes inputs started by this launch.
        RegisterEventHandler(OnProcessExit(
            target_action=localizer,
            on_exit=[EmitEvent(event=Shutdown(reason='cloud_localizer exited'))],
        )),
        localizer,
        Node(
            package='rviz2', executable='rviz2', name='cloud_localization_rviz',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', os.path.join(share, 'rviz', 'cloud_localization_test.rviz')],
            parameters=[{'use_sim_time': clock}], output='screen',
        ),
    ]
    return test_chain


def _after_preflight(event, context, *, test_chain):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason='localization preflight found a conflict'))]
    return test_chain


def _nodes(context):
    test_chain = _test_chain(context)
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    preflight = Node(
        package='rc_cloud_localization', executable='cloud_localization_preflight.py',
        name='cloud_localization_preflight', output='screen',
        parameters=[{
            'use_sim_time': clock,
            **{name: ParameterValue(LaunchConfiguration(name), value_type=bool)
               for name in ('start_lidar', 'start_lio', 'start_robot_state_publisher')},
        }],
    )

    return [RegisterEventHandler(OnProcessExit(
        target_action=preflight,
        on_exit=partial(_after_preflight, test_chain=test_chain))), preflight]


def generate_launch_description():
    share = get_package_share_directory('rc_bringup')
    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_lio', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_robot_state_publisher', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('map_file', description='Required 3D localization PCD file path'),
        DeclareLaunchArgument('localization_params_file', default_value=os.path.join(
            share, 'config', 'reality', 'cloud_localization.yaml')),
        DeclareLaunchArgument('relocalization_backend', default_value='icp', choices=['icp', 'ndt'],
                              description='Seeded local registration, not unseeded global search'),
    ]
    for component in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        declarations.append(DeclareLaunchArgument(f'initial_pose_{component}', default_value='0.0'))
    return LaunchDescription(declarations + [OpaqueFunction(function=_nodes)])
