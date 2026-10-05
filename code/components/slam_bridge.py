"""Optional ROS 2 localization sidecar; no ROS import at module load time.

The scan-release rules and the TF starvation watchdog live in
``SlamBridge.process_iteration``, which talks to a small sink protocol instead
of ROS types.  That keeps the failure-prone timing logic unit-testable without
a ROS graph, and confines every ROS import to ``_run_ros``/``_RosSlamSink``.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Protocol

from core.types import Pose2D, PoseQuality
from .d500_laserscan import D500LaserScanConverter, LaserScanGrid
from .radar_driver import RadarScan
from .slam_anchor import SlamAnchor


LOG = logging.getLogger("robocup-slam-bridge")

# A scan whose mapped measurement time trails the host monotonic clock by more
# than this is dropped as stale: the device-clock offset is wrong, not the map.
SCAN_STALE_S = 0.25

# A published t265_odom -> base_link TF is the only source of the scan-release
# time window.  If no TF has been published for this long the bridge holds the
# last known T265 pose at the current time so slam_toolbox can still bootstrap.
# This is a liveness backstop for the upstream data flow only; it relaxes no
# navigation, readiness or watchdog gate.
TF_STARVE_FORCE_S = 3.0

# Upper bound on the measured T265 TF publication interval used as window slack.
# The TF is only republished when a new T265 pose arrives, so the newest stamp
# legitimately trails a freshly assembled scan by one T265 period: 52 ms at the
# T265's native 19 Hz, far more than one 50 Hz TF period (20 ms).  Measured
# 2026-10-05 (run06): with the T265 read at its native 19 Hz a fixed 20 ms
# tolerance discarded 155 of 292 scans (53 %) while both sensors were healthy,
# which then starved slam_toolbox into >0.5 s anchor gaps and made the fused
# state flap out of ``ok``.
TF_INTERVAL_CAP_S = 0.25


class TfScanSink(Protocol):
    """Everything the bridge needs from ROS, so the logic stays importable."""

    def publish_tf(self, pose: Pose2D, stamp_s: float) -> None: ...

    def publish_scan(self, grid: LaserScanGrid, measurement_s: float) -> None: ...


@dataclass(slots=True)
class _WindowState:
    """Mutable per-run timing state for the window rules."""

    loop_started_mono_s: float = 0.0
    first_tf_stamp_s: float = float("inf")
    last_tf_stamp_s: float = float("-inf")
    last_tf_pub_s: float = float("-inf")
    last_scan_pub_s: float = float("-inf")
    last_tf_publish_mono_s: float | None = None
    tf_interval_s: float = 0.0


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

    def __init__(self, mount, *, tf_hz: float = 50.0, scan_hz: float = 5.0,
                 tf_starve_force_s: float = TF_STARVE_FORCE_S) -> None:
        if not math.isfinite(tf_starve_force_s) or tf_starve_force_s <= 0.0:
            raise ValueError("tf_starve_force_s must be finite and positive")
        self.mount = mount
        self.tf_period_s = 1.0 / tf_hz
        self.scan_period_s = 1.0 / scan_hz
        self.tf_starve_force_s = float(tf_starve_force_s)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pending_t265: tuple[Pose2D, PoseQuality] | None = None
        self._pending_scan: tuple[RadarScan, float] | None = None
        self._anchor: SlamAnchor | None = None
        self._ready = False
        self._started = False
        self._failed = False
        self._loop_pending_until_s = 0.0
        self._loop_count = 0
        self._scan_input_count = 0
        self._scan_publish_count = 0
        self._scan_drop_count = 0
        self._scan_drop_no_tf_window_count = 0
        self._first_tf_stamp_s: float | None = None
        self._last_tf_stamp_s: float | None = None
        self._last_no_window_measurement_s: float | None = None
        self._queue_overwrite_count = 0
        self._anchor_count = 0
        self._tf_lookup_fail_count = 0
        self._tf_starved_force_count = 0
        self._last_scan_age_ms: float | None = None
        self._nearest_t265_tf_age_ms: float | None = None
        self._tf_lookup_success = False
        # Last T265 pose the runtime ever handed over; the starvation watchdog
        # replays it so the scan window cannot stay empty.
        self._last_t265_pose: Pose2D | None = None
        self._converter = D500LaserScanConverter()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("SLAM bridge already started")
        self._stop.clear()
        with self._lock:
            self._started = True
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
            self._last_t265_pose = pose

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
                # Started but not yet through rclpy setup is STARTING, not a
                # failure: reporting FAILED for the first ~0.5 s of every run
                # made a healthy startup look like a dead bridge in the logs.
                return "SLAM_STARTING" if (self._ready or self._started) else "SLAM_FAILED"
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
                "slam.scan_drop_no_tf_window_count": self._scan_drop_no_tf_window_count,
                "slam.first_tf_stamp_s": self._first_tf_stamp_s,
                "slam.last_tf_stamp_s": self._last_tf_stamp_s,
                "slam.no_window_measurement_s": self._last_no_window_measurement_s,
                "slam.tf_starved_force_count": self._tf_starved_force_count,
                "slam.bridge_queue_overwrite_count": self._queue_overwrite_count,
                "slam.scan_age_ms": self._last_scan_age_ms,
                "slam.nearest_t265_tf_age_ms": self._nearest_t265_tf_age_ms,
                "slam.tf_lookup_success": self._tf_lookup_success,
            }

    def new_window_state(self, now_mono: float | None = None) -> _WindowState:
        """Create the per-run window state, anchored at a loop start time."""
        start = time.monotonic() if now_mono is None else float(now_mono)
        return _WindowState(loop_started_mono_s=start)

    # ------------------------------------------------------------------
    # ROS-free core: one iteration of the TF/scan window rules.
    # ------------------------------------------------------------------

    def process_iteration(self, *, t265, pending_scan, sink: TfScanSink,
                          state: _WindowState, now_mono: float) -> None:
        """Run one iteration of the publish/watchdog rules.

        ``t265`` is ``(Pose2D, PoseQuality) | None`` and ``pending_scan`` is
        ``(RadarScan, measurement_monotonic_s) | None``; both are already
        popped off the one-slot queues by the caller.
        """
        if t265 is not None:
            self._publish_t265_tf(t265[0], sink=sink, state=state, now_mono=now_mono)
        self._force_tf_if_starved(sink=sink, state=state, now_mono=now_mono)
        if pending_scan is not None:
            self._release_scan(pending_scan, sink=sink, state=state, now_mono=now_mono)

    def _publish_t265_tf(self, pose: Pose2D, *, sink: TfScanSink,
                         state: _WindowState, now_mono: float) -> bool:
        if not (pose.timestamp_s > state.last_tf_stamp_s
                and pose.timestamp_s - state.last_tf_pub_s >= self.tf_period_s - 0.001):
            return False
        sink.publish_tf(pose, pose.timestamp_s)
        state.last_tf_pub_s = state.last_tf_stamp_s = pose.timestamp_s
        state.first_tf_stamp_s = min(state.first_tf_stamp_s, pose.timestamp_s)
        self._note_tf_publish(state, now_mono)
        self._record_window(state)
        return True

    def _note_tf_publish(self, state: _WindowState, now_mono: float) -> None:
        """Record when the TF was published and how far apart publications are."""
        previous = state.last_tf_publish_mono_s
        if previous is not None:
            interval = now_mono - previous
            if 0.0 < interval <= TF_INTERVAL_CAP_S:
                state.tf_interval_s = interval
        state.last_tf_publish_mono_s = now_mono

    def _window_upper_s(self, state: _WindowState) -> float:
        """Newest TF stamp plus one observed T265 period.

        The TF is only republished when a new T265 pose arrives, so a scan
        assembled just before the next sample is legitimately newer than the
        newest stamp by up to one T265 period.  Using the observed interval
        instead of the nominal 50 Hz TF period keeps the window correct when
        the T265 is read at its native 19 Hz.
        """
        slack = max(self.tf_period_s, state.tf_interval_s) + self.tf_period_s
        return state.last_tf_stamp_s + slack

    def _force_tf_if_starved(self, *, sink: TfScanSink, state: _WindowState,
                             now_mono: float) -> bool:
        """Hold the last known pose when the T265 TF stream has gone quiet.

        Only the upstream data flow is kept alive here: the held pose is
        restamped at the current time because slam_toolbox looks up
        ``t265_odom -> base_link`` at the scan stamp, so replaying the pose at
        its own (possibly seconds old) time would still leave the scan outside
        the buffer window.  The window's lower bound never moves forward, so
        scans already assembled stay publishable.
        """
        reference = state.last_tf_publish_mono_s
        if reference is None:
            reference = state.loop_started_mono_s
        if now_mono - reference < self.tf_starve_force_s:
            return False
        with self._lock:
            held = self._last_t265_pose
        if held is None:
            return False
        sink.publish_tf(held, now_mono)
        state.last_tf_pub_s = state.last_tf_stamp_s = now_mono
        state.first_tf_stamp_s = min(state.first_tf_stamp_s, held.timestamp_s)
        # Do not feed the forced stamp into the cadence estimate: it is a
        # starvation recovery, not a T265 publication.
        state.last_tf_publish_mono_s = now_mono
        with self._lock:
            self._tf_starved_force_count += 1
        self._record_window(state)
        LOG.warning(
            "T265 TF starved for %.1f s; holding last known pose to keep the scan window open",
            now_mono - reference,
        )
        return True

    def _record_window(self, state: _WindowState) -> None:
        with self._lock:
            self._first_tf_stamp_s = state.first_tf_stamp_s
            self._last_tf_stamp_s = state.last_tf_stamp_s

    def _drop_pending_scan(self, pending_scan) -> None:
        with self._lock:
            if self._pending_scan is pending_scan:
                self._pending_scan = None

    def _release_scan(self, pending_scan, *, sink: TfScanSink,
                      state: _WindowState, now_mono: float) -> None:
        scan, measurement_s = pending_scan
        age_s = now_mono - measurement_s
        if measurement_s - state.last_scan_pub_s < self.scan_period_s - 0.001:
            self._drop_pending_scan(pending_scan)
            with self._lock:
                self._scan_drop_count += 1
            return
        if not (state.first_tf_stamp_s <= measurement_s <= self._window_upper_s(state)):
            # The upper bound carries one observed T265 period of slack: the TF
            # is republished only when a new T265 pose arrives, so a scan
            # assembled between two publications legitimately carries a
            # measurement time later than the newest stamp.
            self._drop_pending_scan(pending_scan)
            if age_s > SCAN_STALE_S:
                with self._lock:
                    self._scan_drop_count += 1
                    # Expose why the scan was dropped: a mapped D500 timestamp
                    # that trails the host monotonic clock by more than the
                    # staleness bound means the device-clock offset is wrong,
                    # not that the lidar or the map is broken.
                    self._last_scan_age_ms = age_s * 1000.0
            else:
                # Fresh and not rate limited, yet outside the published T265 TF
                # span.  Without this branch the pending scan was never
                # released, so every later scan was discarded at enqueue and
                # the bridge stayed dead until the process restarted.
                with self._lock:
                    self._scan_drop_no_tf_window_count += 1
                    self._last_no_window_measurement_s = measurement_s
            return
        self._drop_pending_scan(pending_scan)
        if age_s > SCAN_STALE_S:
            with self._lock:
                self._scan_drop_count += 1
                self._last_scan_age_ms = age_s * 1000.0
            return
        grid = self._converter.convert(scan)
        sink.publish_scan(grid, measurement_s)
        state.last_scan_pub_s = measurement_s
        with self._lock:
            self._scan_publish_count += 1
            self._last_scan_age_ms = age_s * 1000.0
            self._nearest_t265_tf_age_ms = abs(state.last_tf_stamp_s - measurement_s) * 1000.0

    # ------------------------------------------------------------------
    # ROS side
    # ------------------------------------------------------------------

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
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        context = rclpy.Context()
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = Node("robocup_slam_bridge", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        mapper = RosTimeMapper(node.get_clock().now().nanoseconds, time.monotonic_ns())
        sink = _RosSlamSink(node, mapper)
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer, node, spin_thread=False)
        try:
            from slam_toolbox.msg import LoopClosureEvent
            node.create_subscription(LoopClosureEvent, "/slam_toolbox/loop_closure_event",
                                     lambda _msg: self._on_loop_closure(), 10)
        except ImportError:
            LOG.warning("slam_toolbox LoopClosureEvent unavailable; large anchors remain gated")

        mount = self.mount
        sink.publish_static(mount)
        state = self.new_window_state()
        last_anchor_stamp_ns = -1
        with self._lock:
            self._ready = True
        try:
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.01)
                now_mono = time.monotonic()
                with self._lock:
                    t265 = self._pending_t265
                    self._pending_t265 = None
                    pending_scan = self._pending_scan
                self.process_iteration(
                    t265=t265, pending_scan=pending_scan, sink=sink,
                    state=state, now_mono=now_mono,
                )

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
                        pose, now_s, loop_closure=now_s <= self._loop_pending_until_s,
                        source_timestamp_s=mapper.to_monotonic_s(transform_ns),
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


class _RosSlamSink:
    """Adapter owning every ROS message type used by the bridge."""

    def __init__(self, node, mapper: RosTimeMapper) -> None:
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import LaserScan
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

        self.mapper = mapper
        self._TransformStamped = TransformStamped
        self._LaserScan = LaserScan
        self._tf_pub = TransformBroadcaster(node)
        self._static_pub = StaticTransformBroadcaster(node)
        self._scan_pub = node.create_publisher(LaserScan, "/d500/scan", 1)

    def _stamp(self, monotonic_s: float):
        from rclpy.time import Time
        return Time(nanoseconds=self.mapper.to_ros_ns(monotonic_s)).to_msg()

    @staticmethod
    def _quaternion(roll: float, pitch: float, yaw: float):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy)

    def make_tf(self, parent: str, child: str, x: float, y: float, z: float,
                yaw: float, timestamp_s: float, roll: float = 0.0, pitch: float = 0.0):
        msg = self._TransformStamped()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.header.stamp = self._stamp(timestamp_s)
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = self._quaternion(roll, pitch, yaw)
        return msg

    def publish_static(self, mount) -> None:
        self._static_pub.sendTransform(self.make_tf(
            "base_link", "d500_link", mount.x_m, mount.y_m, mount.z_m,
            mount.yaw_rad, time.monotonic(), mount.roll_rad, mount.pitch_rad,
        ))

    def publish_tf(self, pose: Pose2D, stamp_s: float) -> None:
        self._tf_pub.sendTransform(self.make_tf(
            "t265_odom", "base_link", pose.x_m, pose.y_m, 0.0, pose.yaw_rad, stamp_s,
        ))

    def publish_scan(self, grid: LaserScanGrid, measurement_s: float) -> None:
        msg = self._LaserScan()
        msg.header.frame_id = "d500_link"
        msg.header.stamp = self._stamp(measurement_s)
        msg.angle_min = grid.angle_min_rad
        msg.angle_max = grid.angle_max_rad
        msg.angle_increment = grid.angle_increment_rad
        msg.range_min = grid.range_min_m
        msg.range_max = grid.range_max_m
        msg.scan_time = grid.scan_time_s
        msg.time_increment = grid.time_increment_s
        msg.ranges = list(grid.ranges_m)
        self._scan_pub.publish(msg)
