"""Offline action/parameter checks. Never execute a launch or create a ROS node."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


SHARE = Path(get_package_share_directory('rc_bringup'))
ENTRY = SHARE / 'launch' / 'cloud_localization_test.launch.py'
SPEC = importlib.util.spec_from_file_location('cloud_test_launch', ENTRY)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def text(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def params(context, node):
    # Humble Node exposes no public pre-execution parameter accessor. Evaluating
    # these descriptions runs only substitutions (including the xacro CLI).
    merged = {}
    for value in evaluate_parameters(context, node._Node__parameters):
        if isinstance(value, dict):
            merged.update(value)
        else:
            document = yaml.safe_load(Path(value).read_text())
            root = document.get('cloud_localizer', document.get('/**', {}))
            merged.update(root.get('ros__parameters', {}))
    return merged


def context_for(tmp_path, **overrides):
    context = LaunchContext()
    for action in MODULE.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            context.launch_configurations[action.name] = text(context, action.default_value)
    map_file = tmp_path / 'synthetic.pcd'
    map_file.write_text(
        'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        'WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n')
    context.launch_configurations['map_file'] = str(map_file)
    context.launch_configurations.update(overrides)
    return context


def nodes_for(context):
    return {text(context, action.node_executable): action
            for action in MODULE._test_chain(context) if isinstance(action, Node)}


def test_installed_resources_and_safe_defaults(tmp_path):
    assert ENTRY.is_file()
    assert (SHARE / 'config/reality/cloud_localization.yaml').is_file()
    assert (SHARE / 'rviz/cloud_localization_test.rviz').is_file()
    context = context_for(tmp_path)
    nodes = nodes_for(context)
    assert set(nodes) == {
        'cloud_localizer', 'robot_state_publisher', 'livox_ros_driver2_node',
        'fastlio_mapping', 'rviz2',
    }
    for executable in ('robot_state_publisher', 'livox_ros_driver2_node', 'fastlio_mapping'):
        assert not nodes[executable].condition.evaluate(context)
    assert params(context, nodes['cloud_localizer'])['use_sim_time'] is False
    declarations = {action.name: action for action in
                    MODULE.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)}
    assert declarations['map_file'].default_value is None


@pytest.mark.parametrize('clock', ['false', 'true'])
@pytest.mark.parametrize('lidar,lio,rsp', [
    ('false', 'false', 'false'), ('true', 'false', 'false'),
    ('false', 'true', 'false'), ('true', 'true', 'true'),
])
def test_mode_conditions_and_actual_parameter_wiring(tmp_path, clock, lidar, lio, rsp):
    custom = tmp_path / 'custom.yaml'
    custom.write_text('cloud_localizer:\n  ros__parameters:\n    max_rmse: 0.123\n')
    values = dict(zip(
        ['initial_pose_x', 'initial_pose_y', 'initial_pose_z',
         'initial_pose_roll', 'initial_pose_pitch', 'initial_pose_yaw'],
        ['1.2', '-2.3', '0.4', '0.1', '-0.2', '0.7']))
    context = context_for(
        tmp_path, use_sim_time=clock, start_lidar=lidar, start_lio=lio,
        start_robot_state_publisher=rsp, use_rviz='false',
        localization_params_file=str(custom), relocalization_backend='ndt', **values)
    nodes = nodes_for(context)
    local = params(context, nodes['cloud_localizer'])
    assert local['map_file'] == str(tmp_path / 'synthetic.pcd')
    assert local['max_rmse'] == 0.123
    assert local['relocalization_backend'] == 'ndt'
    for name, value in values.items():
        assert local[name] == float(value)
    for executable, enabled in [('livox_ros_driver2_node', lidar), ('fastlio_mapping', lio),
                                ('robot_state_publisher', rsp), ('rviz2', 'false')]:
        assert nodes[executable].condition.evaluate(context) == (enabled == 'true')
    for node in nodes.values():
        assert params(context, node)['use_sim_time'] == (clock == 'true')
    fastlio = params(context, nodes['fastlio_mapping'])
    assert fastlio['pcd_save.pcd_save_en'] is False
    assert fastlio['publish.scan_publish_en'] is True
    assert fastlio['map_file_path'] == ''
    driver = params(context, nodes['livox_ros_driver2_node'])
    assert driver['xfer_format'] == 1
    assert Path(driver['user_config_path']) == SHARE / 'config/reality/MID360_driver.json'


def test_urdf_and_fastlio_extrinsics_agree(tmp_path):
    context = context_for(tmp_path, start_robot_state_publisher='true')
    nodes = nodes_for(context)
    robot = ET.fromstring(params(context, nodes['robot_state_publisher'])['robot_description'])
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
    lidar_xyz = [float(v) for v in joints['imu_to_livox'].find('origin').attrib['xyz'].split()]
    lio = yaml.safe_load((SHARE / 'config/reality/fast_lio.yaml').read_text())
    mapping = lio['/**']['ros__parameters']['mapping']
    assert lidar_xyz == mapping['extrinsic_T']
    assert mapping['extrinsic_R'] == [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    rpy = [float(v) for v in joints['base_to_imu'].find('origin').attrib['rpy'].split()]
    assert math.isclose(rpy[1], math.pi, abs_tol=1e-6)
    assert all(joint.attrib['type'] == 'fixed' for joint in joints.values())
    assert all(joint.find('child').attrib['link'] not in ('map', 'odom')
               for joint in joints.values())


def test_rviz_has_required_topics_and_no_motion_tool():
    rviz = yaml.safe_load((SHARE / 'rviz/cloud_localization_test.rviz').read_text())
    manager = rviz['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    topics = {display.get('Topic', {}).get('Value'): display for display in manager['Displays']}
    assert topics['/cloud_localization/map']['Topic']['Durability Policy'] == 'Transient Local'
    for name in ('map', 'aligned_scan', 'pose', 'status_text'):
        assert topics['/cloud_localization/' + name]['Enabled']
    assert not topics['/map']['Enabled']
    tools = manager['Tools']
    assert any(tool.get('Topic', {}).get('Value') == '/initialpose' for tool in tools)
    assert all('Goal' not in tool['Class'] for tool in tools)


def test_missing_map_fails_without_starting_any_action(tmp_path):
    context = context_for(tmp_path, map_file=str(tmp_path / 'missing.pcd'))
    with pytest.raises(RuntimeError, match='map_file'):
        MODULE._nodes(context)


def test_preflight_is_the_only_initial_node_and_blocks_chain_on_failure(tmp_path):
    context = context_for(tmp_path, start_lidar='true', start_lio='true',
                          start_robot_state_publisher='true')
    initial_nodes = [a for a in MODULE._nodes(context) if isinstance(a, Node)]
    assert len(initial_nodes) == 1
    assert text(context, initial_nodes[0].node_executable) == 'cloud_localization_preflight.py'
    preflight = params(context, initial_nodes[0])
    assert all(preflight[name] is True for name in
               ('start_lidar', 'start_lio', 'start_robot_state_publisher'))
    chain = MODULE._test_chain(context)
    failed = MODULE._after_preflight(SimpleNamespace(returncode=1), context, test_chain=chain)
    assert len(failed) == 1 and isinstance(failed[0], EmitEvent)
    assert MODULE._after_preflight(SimpleNamespace(returncode=0), context,
                                  test_chain=chain) is chain
