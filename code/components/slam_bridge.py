"""Optional ROS 2 localization sidecar; no ROS import at module load time."""

from __future__ import annotations

import logging
import math
import threading
import time

from core.types import Pose2D, PoseQuality
from .d500_laserscan import D500LaserScanConverter
from .radar_driver import RadarScan
from .slam_anchor import SlamAnchor


LOG = logging.getLogger("robocup-slam-bridge")


class RosTimeMapper:
    """Map process monotonic measurement times to the ROS clock epoch."""

    def __init__(self, ros_now_ns: int, monotonic_now_ns: int) -> None:
        self.offset_ns = int(ros_now_ns) - int(monotonic_now_ns)

    def to_ros_ns(self, monotonic_s: float) -> int:
        if not math.isfinite(monotonic_s):
            raise ValueError("measurement timestamp must be finite")
        return int(monotonic_s * 1_000_000_000) + self.offset_ns

    def to_monotonic_s(self, ros_ns: int) -> float:
        return (int(ros_ns) - self.offset_ns) / 1_000_000_000


class SlamBridge:
    """A one-slot producer/consumer bridge to slam_toolbox's map-to-odom TF."""

    def __init__(self, mount, *, tf_hz: float = 50.0, scan_hz: float = 5.0) -> None:
        self.mount = mount
        self.tf_period_s = 1.0 / tf_hz
        self.scan_period_s = 1.0 / scan_hz
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pending_t265: tuple[Pose2D, PoseQuality] | None = None
        self._pending_scan: tuple[RadarScan, float] | None = None
        self._anchor: SlamAnchor | None = None
        self._ready = False
        self._failed = False
        self._loop_pending_until_s = 0.0
        self._loop_count = 0
        self._scan_input_count = 0
        self._scan_publish_count = 0
        self._scan_drop_count = 0
        self._queue_overwrite_count = 0
        self._anchor_count = 0
        self._tf_lookup_fail_count = 0
        self._last_scan_age_ms: float | None = None
        self._nearest_t265_tf_age_ms: float | None = None
        self._tf_lookup_success = False

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("SLAM bridge already started")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="robocup-slam-bridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None

    def push_t265(self, pose: Pose2D, quality: PoseQuality) -> None:
        if not quality.valid or not all(math.isfinite(v) for v in (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)):
            return
        with self._lock:
            if self._pending_t265 is not None:
                self._queue_overwrite_count += 1
            self._pending_t265 = (pose, quality)

    def push_d500_scan(self, scan: RadarScan, measurement_monotonic_s: float) -> None:
        if not math.isfinite(measurement_monotonic_s):
            return
        with self._lock:
            self._scan_input_count += 1
            if self._pending_scan is not None:
                self._queue_overwrite_count += 1
                self._scan_drop_count += 1
            self._pending_scan = (scan, measurement_monotonic_s)

    def latest_slam_anchor(self) -> SlamAnchor | None:
        with self._lock:
            return self._anchor

    def state(self, now_s: float | None = None) -> str:
        now = time.monotonic() if now_s is None else now_s
        with self._lock:
            if self._failed:
                return "SLAM_FAILED"
            if self._anchor is None:
                return "SLAM_STARTING" if self._ready else "SLAM_FAILED"
            age = self._anchor.age_s(now)
        return "SLAM_OK" if age <= 0.5 else "SLAM_STALE" if age <= 2.0 else "SLAM_FAILED"

    def healthy(self, now_s: float | None = None) -> bool:
        return self.state(now_s) == "SLAM_OK"

    def metrics(self) -> dict:
        with self._lock:
            return {
                "slam.loop_closure_count": self._loop_count,
                "slam.pose_received_count": self._anchor_count,
                "slam.tf_lookup_fail_count": self._tf_lookup_fail_count,
                "slam.scan_input_count": self._scan_input_count,
                "slam.scan_publish_count": self._scan_publish_count,
                "slam.scan_drop_count": self._scan_drop_count,
                "slam.bridge_queue_overwrite_count": self._queue_overwrite_count,
                "slam.scan_age_ms": self._last_scan_age_ms,
                "slam.nearest_t265_tf_age_ms": self._nearest_t265_tf_age_ms,
                "slam.tf_lookup_success": self._tf_lookup_success,
            }

    def _run(self) -> None:
        try:
            self._run_ros()
        except Exception:
            LOG.exception("ROS SLAM bridge stopped; runtime keeps its existing safety policy")
            with self._lock:
                self._failed = True
                self._ready = False

    def _run_ros(self) -> None:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

        context = rclpy.Context()
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = Node("robocup_slam_bridge", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        mapper = RosTimeMapper(node.get_clock().now().nanoseconds, time.monotonic_ns())
        tf_pub = TransformBroadcaster(node)
        static_pub = StaticTransformBroadcaster(node)
        scan_pub = node.create_publisher(LaserScan, "/d500/scan", 1)
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer, node, spin_thread=False)
        try:
            from slam_toolbox.msg import LoopClosureEvent
            node.create_subscription(LoopClosureEvent, "/slam_toolbox/loop_closure_event",
                                     lambda _msg: self._on_loop_closure(), 10)
        except ImportError:
            LOG.warning("slam_toolbox LoopClosureEvent unavailable; large anchors remain gated")
        converter = D500LaserScanConverter()
        last_tf_pub_s = float("-inf")
        last_tf_stamp_s = float("-inf")
        first_tf_stamp_s = float("inf")
        last_scan_pub_s = float("-inf")
        last_anchor_stamp_ns = -1

        def stamp(monotonic_s):
            return Time(nanoseconds=mapper.to_ros_ns(monotonic_s)).to_msg()

        def quaternion(roll, pitch, yaw):
            cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
            cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
            cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
            return (sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy,
                    cr * cp * sy - sr * sp * cy,
                    cr * cp * cy + sr * sp * sy)

        def make_tf(parent, child, x, y, z, yaw, timestamp_s, roll=0.0, pitch=0.0):
            msg = TransformStamped()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.header.stamp = stamp(timestamp_s)
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = z
            (msg.transform.rotation.x, msg.transform.rotation.y,
             msg.transform.rotation.z, msg.transform.rotation.w) = quaternion(roll, pitch, yaw)
            return msg

        mount = self.mount
        static_pub.sendTransform(make_tf(
            "base_link", "d500_link", mount.x_m, mount.y_m, mount.z_m,
            mount.yaw_rad, time.monotonic(), mount.roll_rad, mount.pitch_rad,
        ))
        with self._lock:
            self._ready = True
        try:
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.01)
                with self._lock:
                    t265 = self._pending_t265
                    self._pending_t265 = None
                    pending_scan = self._pending_scan
                if t265 is not None:
                    pose, _quality = t265
                    if pose.timestamp_s > last_tf_stamp_s and pose.timestamp_s - last_tf_pub_s >= self.tf_period_s - 0.001:
                        tf_pub.sendTransform(make_tf(
                            "t265_odom", "base_link", pose.x_m, pose.y_m, 0.0,
                            pose.yaw_rad, pose.timestamp_s,
                        ))
                        last_tf_pub_s = last_tf_stamp_s = pose.timestamp_s
                        first_tf_stamp_s = min(first_tf_stamp_s, pose.timestamp_s)
                if pending_scan is not None:
                    scan, measurement_s = pending_scan
                    if measurement_s - last_scan_pub_s < self.scan_period_s - 0.001:
                        with self._lock:
                            if self._pending_scan is pending_scan:
                                self._pending_scan = None
                            self._scan_drop_count += 1
                    elif first_tf_stamp_s <= measurement_s <= last_tf_stamp_s:
                        with self._lock:
                            if self._pending_scan is pending_scan:
                                self._pending_scan = None
                        if time.monotonic() - measurement_s > 0.25:
                            with self._lock:
                                self._scan_drop_count += 1
                        else:
                            grid = converter.convert(scan)
                            msg = LaserScan()
                            msg.header.frame_id = "d500_link"
                            msg.header.stamp = stamp(measurement_s)
                            msg.angle_min = grid.angle_min_rad
                            msg.angle_max = grid.angle_max_rad
                            msg.angle_increment = grid.angle_increment_rad
                            msg.range_min = grid.range_min_m
                            msg.range_max = grid.range_max_m
                            msg.scan_time = grid.scan_time_s
                            msg.time_increment = grid.time_increment_s
                            msg.ranges = list(grid.ranges_m)
                            scan_pub.publish(msg)
                            last_scan_pub_s = measurement_s
                            with self._lock:
                                self._scan_publish_count += 1
                                self._last_scan_age_ms = (time.monotonic() - measurement_s) * 1000.0
                                self._nearest_t265_tf_age_ms = abs(last_tf_stamp_s - measurement_s) * 1000.0
                    elif time.monotonic() - measurement_s > 0.25:
                        with self._lock:
                            if self._pending_scan is pending_scan:
                                self._pending_scan = None
                            self._scan_drop_count += 1

                try:
                    transform = tf_buffer.lookup_transform("slam_map", "t265_odom", Time())
                except Exception:
                    with self._lock:
                        self._tf_lookup_fail_count += 1
                        self._tf_lookup_success = False
                    continue
                header = transform.header.stamp
                transform_ns = header.sec * 1_000_000_000 + header.nanosec
                if transform_ns <= last_anchor_stamp_ns:
                    continue
                now_s = time.monotonic()
                transform_age = now_s - mapper.to_monotonic_s(transform_ns)
                if transform_age > 0.5 or transform_age < -0.25:
                    continue
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                                 1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
                pose = Pose2D(translation.x, translation.y, yaw, now_s)
                if not all(math.isfinite(value) for value in (pose.x_m, pose.y_m, pose.yaw_rad)):
                    continue
                with self._lock:
                    self._anchor = SlamAnchor(
                        pose, now_s, loop_closure=now_s <= self._loop_pending_until_s
                    )
                    self._anchor_count += 1
                    self._tf_lookup_success = True
                last_anchor_stamp_ns = transform_ns
        finally:
            with self._lock:
                self._ready = False
            del listener
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)

    def _on_loop_closure(self) -> None:
        with self._lock:
            self._loop_pending_until_s = time.monotonic() + 1.0
            self._loop_count += 1
