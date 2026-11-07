"""No rclpy.init, no nodes, no discovery: test the preflight decision policy."""

import importlib.util
from pathlib import Path

import pytest


PATH = Path(__file__).parents[1] / 'scripts/cloud_localization_preflight.py'
SPEC = importlib.util.spec_from_file_location('preflight_policy', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_default_reuse_allows_one_existing_input_chain():
    assert MODULE.conflict_reasons(
        ['/fastlio', '/robot_state_publisher'],
        {'/Odometry': 1, '/cloud_registered': 1, '/robot_description': 1, '/livox/lidar': 1},
        {('odom', 'base_link'), ('base_link', 'imu_link')}, False, False, False) == []


@pytest.mark.parametrize('flags,publishers,transforms', [
    ((True, False, False), {'/livox/lidar': 1}, set()),
    ((True, False, False), {'/livox/imu': 1}, set()),
    ((False, True, False), {'/Odometry': 1}, set()),
    ((False, True, False), {}, {('odom', 'base_link')}),
    ((False, False, True), {'/robot_description': 1}, set()),
    ((False, False, True), {}, {('base_link', 'imu_link')}),
])
def test_requested_duplicate_input_is_rejected(flags, publishers, transforms):
    assert MODULE.conflict_reasons([], publishers, transforms, *flags)


@pytest.mark.parametrize('names,publishers,transforms', [
    (['/cloud_localizer'], {}, set()),
    (['/cloud_localization_rviz'], {}, set()),
    (['/cloud_localization_preflight'] * 2, {}, set()),
    ([], {}, {('map', 'odom')}),
    ([], {}, {('world', 'map')}),
    ([], {'/cloud_registered': 2}, set()),
])
def test_reuse_also_rejects_localizer_tf_or_duplicate_scan_conflicts(names, publishers, transforms):
    assert MODULE.conflict_reasons(names, publishers, transforms, False, False, False)


def test_from_zero_allows_explicit_full_input_chain():
    assert MODULE.conflict_reasons([], {}, set(), True, True, True) == []
