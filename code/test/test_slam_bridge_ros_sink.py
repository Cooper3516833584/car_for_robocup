"""Hardware-free checks for the ROS-facing half of SlamBridge.

``_RosSlamSink`` is the only place that touches ROS message types, so it cannot
run in the normal test suite.  These tests install minimal fakes for the ROS
modules it imports and drive the real adapter code against them -- the 2026-10-05
field run caught ``AttributeError: '_RosSlamSink' object has no attribute
'_LaserScan'`` this way, which a fake-sink test could never see.
"""

from __future__ import annotations

from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.d500_laserscan import LaserScanGrid
from components import slam_bridge as bridge_module
from components.slam_bridge import RosTimeMapper, _RosSlamSink
from config.v2_loader import load_v2_config
from core.types import Pose2D


class _Vec3:
    def __init__(self) -> None:
        self.x = self.y = self.z = 0.0


class _Quaternion:
    def __init__(self) -> None:
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class _Transform:
    def __init__(self) -> None:
        self.translation = _Vec3()
        self.rotation = _Quaternion()


class _Header:
    def __init__(self) -> None:
        self.frame_id = ""
        self.stamp = None


class _TransformStamped:
    def __init__(self) -> None:
        self.header = _Header()
        self.child_frame_id = ""
        self.transform = _Transform()


class _LaserScan:
    def __init__(self) -> None:
        self.header = _Header()
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None
        self.scan_time = None
        self.time_increment = None
        self.ranges = None


class _Time:
    def __init__(self, *, nanoseconds: int = 0) -> None:
        self.nanoseconds = int(nanoseconds)

    def to_msg(self):
        return ("stamp", self.nanoseconds)


class _RecordingBroadcaster:
    def __init__(self, node) -> None:
        self.node = node
        self.sent: list = []

    def sendTransform(self, message) -> None:  # noqa: N802 - ROS API name
        self.sent.append(message)


class _RecordingPublisher:
    def __init__(self, topic_type, topic, qos) -> None:
        self.topic_type = topic_type
        self.topic = topic
        self.qos = qos
        self.published: list = []

    def publish(self, message) -> None:
        self.published.append(message)


class _FakeNode:
    def __init__(self) -> None:
        self.publisher = _RecordingPublisher(None, None, None)
        self.created: list = []

    def create_publisher(self, topic_type, topic, qos):  # noqa: N802 - ROS API name
        self.created.append((topic_type, topic, qos))
        self.publisher.topic_type = topic_type
        self.publisher.topic = topic
        return self.publisher


class _FakeBroadcasterFactory:
    instances: list = []

    def __init__(self, node) -> None:
        self.node = node
        self.sent: list = []
        _FakeBroadcasterFactory.instances.append(self)

    def sendTransform(self, message) -> None:  # noqa: N802 - ROS API name
        self.sent.append(message)


def _fake_ros_modules():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TransformStamped = _TransformStamped
    geometry_msgs.msg = geometry_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = _LaserScan
    sensor_msgs.msg = sensor_msgs_msg

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.TransformBroadcaster = _FakeBroadcasterFactory
    tf2_ros.StaticTransformBroadcaster = _FakeBroadcasterFactory

    rclpy = types.ModuleType("rclpy")
    rclpy_time = types.ModuleType("rclpy.time")
    rclpy_time.Time = _Time
    rclpy.time = rclpy_time

    return {
        "geometry_msgs": geometry_msgs, "geometry_msgs.msg": geometry_msgs_msg,
        "sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msgs_msg,
        "tf2_ros": tf2_ros, "rclpy": rclpy, "rclpy.time": rclpy_time,
    }


class RosSlamSinkTests(unittest.TestCase):
    def setUp(self) -> None:
        _FakeBroadcasterFactory.instances = []
        self.modules = _fake_ros_modules()
        self.patcher = patch.dict(sys.modules, self.modules)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.node = _FakeNode()
        self.mapper = RosTimeMapper(10_000_000_000, 2_000_000_000)
        self.sink = _RosSlamSink(self.node, self.mapper)

    def test_publisher_is_created_for_the_d500_scan_topic(self) -> None:
        self.assertEqual(len(self.node.created), 1)
        topic_type, topic, qos = self.node.created[0]
        self.assertIs(topic_type, _LaserScan)
        self.assertEqual(topic, "/d500/scan")
        self.assertEqual(qos, 1)

    def test_static_mount_transform_is_published_between_base_link_and_d500(self) -> None:
        self.sink.publish_static(load_v2_config().d500_mount)
        static = _FakeBroadcasterFactory.instances[1]
        self.assertEqual(len(static.sent), 1)
        message = static.sent[0]
        self.assertEqual(message.header.frame_id, "base_link")
        self.assertEqual(message.child_frame_id, "d500_link")

    def test_t265_transform_uses_the_pose_stamp(self) -> None:
        self.sink.publish_tf(Pose2D(1.0, 2.0, 0.3, 2.5), 2.5)
        broadcaster = _FakeBroadcasterFactory.instances[0]
        message = broadcaster.sent[0]
        self.assertEqual(message.header.frame_id, "t265_odom")
        self.assertEqual(message.child_frame_id, "base_link")
        self.assertAlmostEqual(message.transform.translation.x, 1.0)
        self.assertAlmostEqual(message.transform.translation.y, 2.0)
        # mapper offset is 8 s; the scan stamp must be the mapped measurement time
        self.assertEqual(message.header.stamp, ("stamp", self.mapper.to_ros_ns(2.5)))

    def test_scan_message_is_built_from_the_grid(self) -> None:
        grid = LaserScanGrid((1.0, float("inf"), 2.5), 0.14, 0.001)
        self.sink.publish_scan(grid, 2.6)
        self.assertEqual(len(self.node.publisher.published), 1)
        message = self.node.publisher.published[0]
        self.assertIsInstance(message, _LaserScan)
        self.assertEqual(message.header.frame_id, "d500_link")
        self.assertEqual(message.header.stamp, ("stamp", self.mapper.to_ros_ns(2.6)))
        self.assertAlmostEqual(message.angle_min, grid.angle_min_rad)
        self.assertAlmostEqual(message.angle_increment, grid.angle_increment_rad)
        self.assertAlmostEqual(message.range_max, grid.range_max_m)
        self.assertAlmostEqual(message.scan_time, grid.scan_time_s)
        self.assertEqual(list(message.ranges), list(grid.ranges_m))

    def test_module_does_not_import_ros_at_load_time(self) -> None:
        # The module must stay importable on a workstation without ROS.
        self.assertNotIn("rclpy", getattr(bridge_module, "__dict__", {}))
        self.assertFalse(hasattr(bridge_module, "LaserScan"))


if __name__ == "__main__":
    unittest.main()
