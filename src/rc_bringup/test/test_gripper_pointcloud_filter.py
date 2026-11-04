import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformException


SCRIPT = Path(__file__).parents[1] / "scripts" / "gripper_pointcloud_filter.py"
SPEC = importlib.util.spec_from_file_location("gripper_pointcloud_filter", SCRIPT)
filter_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = filter_module
SPEC.loader.exec_module(filter_module)


def box():
    return filter_module.Box("base_link", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))


def test_box_interior_exterior_and_inclusive_boundary():
    points = np.array([
        [0.5, 0.5, 0.5],
        [-0.001, 0.5, 0.5],
        [1.0, 0.5, 0.5],
        [1.001, 0.5, 0.5],
    ])
    keep = filter_module.filter_mask(points, [(points.copy(), box())])
    assert keep.tolist() == [False, True, False, True]


def test_tf_lookup_uses_pointcloud_timestamp_and_timeout():
    stamp = TimeMsg(sec=42, nanosec=123456789)

    class Buffer:
        def lookup_transform(self, target, source, time, timeout):
            assert (target, source) == ("base_link", "odom")
            assert time.to_msg() == stamp
            assert timeout.nanoseconds == 75_000_000
            return "transform-at-cloud-time"

    result = filter_module.lookup_transform_at_stamp(Buffer(), "base_link", "odom", stamp, 0.075)
    assert result == "transform-at-cloud-time"


def test_cloud_points_are_transformed_into_configured_box_frame():
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0
    points = filter_module.transform_xyz(np.array([[0.25, -0.5, 2.0]]), transform)
    np.testing.assert_allclose(points, [[1.25, -0.5, 2.0]])


def test_tf_unavailable_fails_closed():
    class MissingBuffer:
        def lookup_transform(self, *_args, **_kwargs):
            raise TransformException("no transform at requested stamp")

    with pytest.raises(TransformException):
        filter_module.lookup_transform_at_stamp(
            MissingBuffer(), "gripper_link", "odom", TimeMsg(sec=12), 0.01)


def test_node_publishes_empty_cloud_when_timestamped_tf_is_missing(monkeypatch):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Logger:
        def error(self, _message):
            pass

    node = object.__new__(filter_module.GripperPointCloudFilter)
    node.enabled = True
    node.publisher = Publisher()
    node.status_publisher = Publisher()
    node.tf_timeout = 0.01
    node.tf_buffer = object()
    node.boxes = [box()]
    node.block_boxes = []
    node.state = filter_module.EMPTY
    node.block_filter_enabled = False
    monkeypatch.setattr(filter_module.GripperPointCloudFilter, "get_logger", lambda _self: Logger())

    def missing_tf(*_args, **_kwargs):
        raise TransformException("no transform at point-cloud timestamp")

    monkeypatch.setattr(filter_module, "lookup_transform_at_stamp", missing_tf)
    header = Header(stamp=TimeMsg(sec=12), frame_id="odom")
    fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8))
    ]
    cloud = point_cloud2.create_cloud(header, fields, [(0.5, 0.5, 0.5)])

    node.cloud_callback(cloud)

    assert node.publisher.messages[0].width == 0
    assert "ERROR TF_UNAVAILABLE" in node.status_publisher.messages[0].data


def test_default_empty_state_does_not_add_held_block_filter():
    self_boxes = [box()]
    held = [filter_module.Box("footprint_link", (2, 2, 2), (3, 3, 3))]
    selected = filter_module.boxes_for_state(filter_module.EMPTY, self_boxes, held, True)
    assert selected == self_boxes
    config_path = Path(__file__).parents[1] / "config" / "reality" / "gripper_filter.yaml"
    config = yaml.safe_load(config_path.read_text())["gripper_pointcloud_filter"]["ros__parameters"]
    assert config["block_filter_enabled"] is False
    assert config["state_input_enabled"] is False


def test_adjacent_obstacle_outside_box_is_preserved():
    points = np.array([
        [0.9, 0.5, 0.5],
        [1.01, 0.5, 0.5],  # adjacent obstacle, just outside the measured box
    ])
    keep = filter_module.filter_mask(points, [(points.copy(), box())])
    assert points[keep].tolist() == [[1.01, 0.5, 0.5]]


def test_occluded_infinite_ray_cannot_clear_existing_obstacle():
    config_path = Path(__file__).parents[1] / "config" / "reality" / "nav2_params_forest_small_teb_mppi.yaml"
    config = yaml.safe_load(config_path.read_text())
    for costmap_name in ("local_costmap", "global_costmap"):
        layer = config[costmap_name][costmap_name]["ros__parameters"]["obstacle_layer"]
        assert layer["observation_sources"] == "scan"  # legacy default outside this launch
        marking = layer["gripper_marking"]
        clearing = layer["gripper_clearing"]
        assert marking["topic"] == "/scan_costmap"
        assert marking["marking"] is True
        assert marking["clearing"] is False
        assert clearing["topic"] == "/scan"
        assert clearing["marking"] is False
        assert clearing["clearing"] is True
        assert clearing["inf_is_valid"] is False

    # Under the configured Nav2 source policy an +Inf beam is not a clearing
    # observation. Keeping an existing endpoint behind the gripper therefore
    # cannot be raytraced away by the shadow sector.
    shadow_range = math.inf
    assert not math.isfinite(shadow_range) and not clearing["inf_is_valid"]


def test_pcl_padding_fields_row_stride_and_endianness_are_preserved():
    from sensor_msgs.msg import PointCloud2
    import struct

    fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
              for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 16))]
    for bigendian in (False, True):
        raw = bytearray(80)  # two 32-byte points with 8-byte row padding
        for index, x in enumerate((0.5, 1.01)):
            struct.pack_into(('>' if bigendian else '<') + 'fff', raw, 40 * index, x, 0.5, 0.5)
            struct.pack_into(('>' if bigendian else '<') + 'f', raw, 40 * index + 16, 23.0)
        cloud = PointCloud2(height=2, width=1, fields=fields, point_step=32,
                            row_step=40, data=bytes(raw), is_bigendian=bigendian)
        xyz, records = filter_module.cloud_records(cloud)
        keep = filter_module.filter_mask(xyz, [(xyz, box())])
        result = filter_module.select_cloud(cloud, records, keep)
        assert result.width == 1 and result.point_step == 32
        assert result.fields == fields and result.is_bigendian == bigendian
        assert bytes(result.data) == bytes(raw[40:72])


def test_launch_selects_costmap_sources_and_restores_legacy_source():
    from launch import LaunchContext

    launch_file = Path(__file__).parents[1] / 'launch' / 'mapping_nav2.launch.py'
    spec = importlib.util.spec_from_file_location('gripper_mapping_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config_dir = str(Path(__file__).parents[1] / 'config' / 'reality')
    for enabled in ('true', 'false', 'True', 'False'):
        context = LaunchContext()
        files = module.build_nav2_params(config_dir, 'false', enabled, '/test/costmap_scan')
        config = yaml.safe_load(Path(files[0].evaluate(context)).read_text())
        active = enabled.lower() == 'true'
        for name in ('local_costmap', 'global_costmap'):
            layer = config[name][name]['ros__parameters']['obstacle_layer']
            assert layer['scan']['marking'] is (not active)
            assert layer['scan']['clearing'] is (not active)
            assert ('gripper_marking' in layer['observation_sources'].split()) is active
            assert layer['gripper_marking']['topic'] == '/test/costmap_scan'
            assert layer['gripper_marking']['clearing'] is False
            assert layer['gripper_clearing']['inf_is_valid'] is False
        for parameter_file in files:
            parameter_file.cleanup()
