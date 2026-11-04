#!/usr/bin/env python3
"""Filter robot self points from a PointCloud2 before costmap projection.

The node deliberately does not know a serial protocol.  The future serial
reader should validate its frame and call ``update_state_from_serial`` with
either ``EMPTY`` or ``HOLDING``.  It must not infer HOLDING from a command
being sent.
"""

from __future__ import annotations

import math
from copy import deepcopy
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


EMPTY = "EMPTY"
HOLDING = "HOLDING"


@dataclass(frozen=True)
class Box:
    frame: str
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]


def point_in_box(point: Sequence[float], minimum: Sequence[float], maximum: Sequence[float]) -> bool:
    """Return true for an inclusive axis-aligned box boundary."""
    return all(lo <= value <= hi for value, lo, hi in zip(point, minimum, maximum))


def transform_xyz(points: np.ndarray, transform: TransformStamped) -> np.ndarray:
    """Apply a ROS TransformStamped to an ``N x 3`` float array."""
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("TF contains an invalid zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])
    translation = np.array([
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ])
    if not np.isfinite(translation).all():
        raise ValueError("TF contains a non-finite translation")
    return points @ rotation.T + translation


def cloud_records(message: PointCloud2):
    """Read XYZ and raw records, including PCL point padding and row padding."""
    from sensor_msgs_py.point_cloud2 import dtype_from_fields

    if message.point_step <= 0 or message.row_step < message.width * message.point_step:
        raise ValueError("invalid PointCloud2 point_step/row_step")
    if len(message.data) != message.height * message.row_step:
        raise ValueError("PointCloud2 data length does not match dimensions")
    dtype = dtype_from_fields(message.fields, point_step=message.point_step)
    dtype = dtype.newbyteorder(">" if message.is_bigendian else "<")
    points = np.ndarray(
        (message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step))
    if not {"x", "y", "z"}.issubset(points.dtype.names or ()):
        raise ValueError("PointCloud2 must contain scalar x, y and z fields")
    xyz = np.column_stack([points[name].reshape(-1) for name in ("x", "y", "z")])
    if xyz.shape != (message.height * message.width, 3):
        raise ValueError("PointCloud2 XYZ fields must be scalar")
    records = np.ndarray(
        (message.height, message.width), dtype=f"V{message.point_step}",
        buffer=message.data, strides=(message.row_step, message.point_step))
    return xyz.astype(np.float64), records.reshape(-1)


def select_cloud(message: PointCloud2, records=None, keep=None) -> PointCloud2:
    """Preserve every original point field and endian flag in selected records."""
    result = deepcopy(message)
    result.height = 1
    result.width = 0 if records is None else int(np.count_nonzero(keep))
    result.row_step = result.width * result.point_step
    result.data = b"" if records is None else records[keep].tobytes()
    return result


def filter_mask(
    points: np.ndarray,
    transformed_boxes: Iterable[tuple[np.ndarray, Box]],
) -> np.ndarray:
    """Return a mask that removes points in any transformed box.

    NaN points are retained; the downstream pointcloud-to-laserscan node
    already rejects them, and retaining them avoids changing unrelated fields
    or neighbouring obstacle points.
    """
    keep = np.ones(points.shape[0], dtype=bool)
    for transformed, box in transformed_boxes:
        inside = np.all(
            (transformed >= np.asarray(box.minimum)) &
            (transformed <= np.asarray(box.maximum)), axis=1)
        inside &= np.isfinite(transformed).all(axis=1)
        keep &= ~inside
    return keep


def boxes_for_state(
    state: str,
    self_boxes: Sequence[Box],
    block_boxes: Sequence[Box],
    block_filter_enabled: bool,
) -> list[Box]:
    """Select self boxes and only add a held block after confirmed HOLDING."""
    selected = list(self_boxes)
    if state == HOLDING and block_filter_enabled:
        selected.extend(block_boxes)
    return selected


def lookup_transform_at_stamp(buffer, target_frame: str, source_frame: str, stamp, timeout_sec: float):
    """Look up exactly at the cloud timestamp; never fall back to latest TF."""
    return buffer.lookup_transform(
        target_frame,
        source_frame,
        Time.from_msg(stamp),
        timeout=Duration(seconds=float(timeout_sec)),
    )


class GripperPointCloudFilter(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("gripper_pointcloud_filter", **kwargs)
        self.declare_parameter("enabled", True)
        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("output_topic", "/cloud_registered_costmap")
        self.declare_parameter("status_topic", "/gripper_filter/status")
        self.declare_parameter("tf_timeout", 0.10)
        self.declare_parameter("box_count", 0)
        self.declare_parameter("box_frames", ["footprint_link"])
        self.declare_parameter("box_min", [-0.50, -0.50, -0.50])
        self.declare_parameter("box_max", [0.50, 0.50, 0.50])
        # Reserved for a confirmed held block.  It is intentionally disabled
        # until a validated EMPTY/HOLDING state source is connected.
        self.declare_parameter("block_filter_enabled", False)
        self.declare_parameter("block_box_count", 0)
        self.declare_parameter("block_box_frames", ["footprint_link"])
        self.declare_parameter("block_box_min", [-0.10, -0.10, -0.10])
        self.declare_parameter("block_box_max", [0.10, 0.10, 0.10])
        self.declare_parameter("state_input_enabled", False)
        self.declare_parameter("state_topic", "/gripper_filter/state")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        if not math.isfinite(self.tf_timeout) or self.tf_timeout < 0:
            raise ValueError("tf_timeout must be finite and nonnegative")
        self.state = EMPTY
        self.block_filter_enabled = bool(self.get_parameter("block_filter_enabled").value)
        self.state_input_enabled = bool(self.get_parameter("state_input_enabled").value)
        self.boxes = self._read_boxes("box")
        self.block_boxes = self._read_boxes("block_box")

        self.tf_buffer = Buffer()
        # Keep TF callbacks in this node's executor.  The point-cloud callback
        # performs a bounded TF lookup, so the executor needs another worker
        # to receive /tf while that callback is waiting.
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)
        qos = rclpy.qos.qos_profile_sensor_data
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.state_subscription = None
        if self.state_input_enabled:
            self.state_subscription = self.create_subscription(
                String, str(self.get_parameter("state_topic").value),
                self._state_callback, 10)
        self.get_logger().info(
            f"gripper filter {'enabled' if self.enabled else 'disabled'}; "
            f"{len(self.boxes)} self boxes, block filtering "
            f"{'enabled' if self.block_filter_enabled else 'disabled'}")
        if not self.boxes and self.enabled:
            self.get_logger().warning(
                "No self-filter boxes are configured; configure measured box bounds "
                "before relying on gripper removal; costmap output will be empty")
        # Future serial integration: validate framing, checksum, freshness and
        # explicit grasp confirmation in the serial reader, then call
        # update_state_from_serial(EMPTY/HOLDING). Never infer HOLDING from a
        # grasp command alone. The optional String topic is disabled by default
        # and is only a test/debug hook.

    def _read_boxes(self, prefix: str) -> list[Box]:
        count = int(self.get_parameter(f"{prefix}_count").value)
        frames = list(self.get_parameter(f"{prefix}_frames").value)
        minimum = list(self.get_parameter(f"{prefix}_min").value)
        maximum = list(self.get_parameter(f"{prefix}_max").value)
        if count < 0 or (count and (
                len(frames) != count or len(minimum) != 3 * count or len(maximum) != 3 * count)):
            raise ValueError(f"Invalid {prefix} box count/array lengths")
        boxes = []
        for index in range(count):
            lo = tuple(float(value) for value in minimum[3 * index:3 * index + 3])
            hi = tuple(float(value) for value in maximum[3 * index:3 * index + 3])
            if not all(math.isfinite(value) for value in (*lo, *hi)) or any(a > b for a, b in zip(lo, hi)):
                raise ValueError(f"Invalid {prefix} box bounds at index {index}")
            if not str(frames[index]).strip():
                raise ValueError(f"Empty {prefix} box frame at index {index}")
            boxes.append(Box(str(frames[index]), lo, hi))
        return boxes

    def _state_callback(self, message: String) -> None:
        self.update_state_from_serial(message.data)

    def update_state_from_serial(self, validated_state: str) -> bool:
        """Future serial hook: call only after protocol and checksum validation."""
        state = str(validated_state).strip().upper()
        if state not in (EMPTY, HOLDING):
            self.get_logger().warning("Ignoring unknown gripper state; state remains EMPTY/HOLDING")
            return False
        self.state = state
        return True

    def _publish_status(self, text: str) -> None:
        self.status_publisher.publish(String(data=text))

    def cloud_callback(self, message: PointCloud2) -> None:
        if not self.enabled:
            self.publisher.publish(message)
            self._publish_status("DISABLED passthrough")
            return
        if not message.header.frame_id:
            self.get_logger().error("PointCloud2 has no frame_id; publishing an empty filtered cloud")
            self.publisher.publish(select_cloud(message))
            self._publish_status("ERROR missing cloud frame_id")
            return
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self.get_logger().error(
                "PointCloud2 timestamp is zero; refusing latest-TF lookup and "
                "publishing an empty filtered cloud")
            self.publisher.publish(select_cloud(message))
            self._publish_status("ERROR zero cloud stamp filtered output empty")
            return
        if not self.boxes:
            self.get_logger().error(
                "No measured self-filter boxes are configured; refusing to pass raw "
                "points as filtered data")
            self.publisher.publish(select_cloud(message))
            self._publish_status("ERROR NO_BOXES_CONFIGURED filtered output empty")
            return

        try:
            xyz, records = cloud_records(message)
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"Invalid PointCloud2 data: {error}; publishing an empty filtered cloud")
            self.publisher.publish(select_cloud(message))
            self._publish_status("ERROR invalid PointCloud2 filtered output empty")
            return
        boxes = boxes_for_state(
            self.state, self.boxes, self.block_boxes, self.block_filter_enabled)
        transformed_boxes = []
        transformed_by_frame = {}
        try:
            for box in boxes:
                if box.frame not in transformed_by_frame:
                    transform = lookup_transform_at_stamp(
                        self.tf_buffer, box.frame, message.header.frame_id,
                        message.header.stamp, self.tf_timeout)
                    transformed_by_frame[box.frame] = transform_xyz(xyz, transform)
                transformed_boxes.append((transformed_by_frame[box.frame], box))
        except (TransformException, ValueError, TypeError) as error:
            self.get_logger().error(
                f"TF unavailable/invalid at cloud stamp for self filtering: {error}; "
                "publishing an empty filtered cloud")
            self.publisher.publish(select_cloud(message))
            self._publish_status("ERROR TF_UNAVAILABLE filtered output empty")
            return

        keep = filter_mask(xyz, transformed_boxes)
        filtered = select_cloud(message, records, keep)
        self.publisher.publish(filtered)
        removed = int(np.count_nonzero(~keep))
        self._publish_status(f"OK state={self.state} removed={removed} boxes={len(boxes)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperPointCloudFilter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
