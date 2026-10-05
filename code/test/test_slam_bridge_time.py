from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.slam_bridge import RosTimeMapper, SlamBridge
from components.radar_driver import RadarScan
from config.v2_loader import load_v2_config
from core.types import Pose2D, PoseQuality


class _RecordingSink:
    """Stand-in for the ROS publisher; records what the bridge released."""

    def __init__(self) -> None:
        self.tfs: list[tuple[Pose2D, float]] = []
        self.scans: list[tuple[object, float]] = []

    def publish_tf(self, pose: Pose2D, stamp_s: float) -> None:
        self.tfs.append((pose, stamp_s))

    def publish_scan(self, grid, measurement_s: float) -> None:
        self.scans.append((grid, measurement_s))


def _good_t265() -> PoseQuality:
    return PoseQuality("t265", True, False)


def _scan() -> RadarScan:
    return RadarScan((), 0, 1000)


class SlamBridgeTimeTests(unittest.TestCase):
    def test_measurement_time_not_publish_time(self) -> None:
        mapper = RosTimeMapper(10_000_000_000, 2_000_000_000)
        self.assertEqual(mapper.to_ros_ns(2.25), 10_250_000_000)
        self.assertAlmostEqual(mapper.to_monotonic_s(10_250_000_000), 2.25)

    def test_one_slot_scan_and_t265_inputs(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount)
        scan = RadarScan((), 0, 1000)
        bridge.push_d500_scan(scan, 1.0)
        bridge.push_d500_scan(scan, 1.1)
        good = PoseQuality("t265", True, False)
        bridge.push_t265(Pose2D(0.0, 0.0, 0.0, 1.0), good)
        bridge.push_t265(Pose2D(0.1, 0.0, 0.0, 1.1), good)
        self.assertEqual(bridge._pending_scan[1], 1.1)
        self.assertAlmostEqual(bridge._pending_t265[0].x_m, 0.1)
        self.assertEqual(bridge.metrics()["slam.bridge_queue_overwrite_count"], 2)

    def test_a_started_but_not_ready_bridge_reports_starting_not_failed(self) -> None:
        # Regression for the 2026-10-05 run08 log: the first slam_status sample
        # of every run was SLAM_FAILED simply because _run_ros had not finished
        # its rclpy setup yet, which made a healthy startup look like a dead
        # bridge.
        bridge = SlamBridge(load_v2_config().d500_mount)
        self.assertEqual(bridge.state(0.0), "SLAM_FAILED")
        with bridge._lock:
            bridge._started = True
        self.assertEqual(bridge.state(0.0), "SLAM_STARTING")
        with bridge._lock:
            bridge._failed = True
        self.assertEqual(bridge.state(0.0), "SLAM_FAILED")

    def test_watchdog_period_must_be_positive(self) -> None:
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                SlamBridge(load_v2_config().d500_mount, tf_starve_force_s=bad)

    # ------------------------------------------------------------------
    # Scan window release
    # ------------------------------------------------------------------

    def test_scan_newer_than_the_newest_tf_within_one_tf_period_is_published(self) -> None:
        # Regression for the 2026-10-04 window race: a scan assembled a few
        # milliseconds after the newest T265 TF stamp was rejected, which is
        # most scans whenever the lidar and the T265 are not phase locked.
        bridge = SlamBridge(load_v2_config().d500_mount)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=0.0)
        bridge.process_iteration(t265=(Pose2D(0.0, 0.0, 0.0, 1.0), _good_t265()),
                                 pending_scan=None, sink=sink, state=state, now_mono=1.0)
        measurement_s = 1.0 + bridge.tf_period_s - 0.002
        bridge.process_iteration(t265=None, pending_scan=(_scan(), measurement_s),
                                 sink=sink, state=state, now_mono=measurement_s)

        metrics = bridge.metrics()
        self.assertEqual(len(sink.tfs), 1)
        self.assertEqual(len(sink.scans), 1)
        self.assertEqual(metrics["slam.scan_publish_count"], 1)
        self.assertEqual(metrics["slam.scan_drop_no_tf_window_count"], 0)

    def test_a_batch_of_scans_after_the_newest_tf_is_never_counted_as_no_window(self) -> None:
        # TF at 50 Hz, scans at 5 Hz, every scan assembled 4 ms after the newest
        # TF sample.  With the one-TF-period tolerance this must publish them all
        # and leave the no-window counter at zero.
        bridge = SlamBridge(load_v2_config().d500_mount)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=0.0)
        for index in range(10):
            base = 10.0 + index * 0.2
            for tf_index in range(4):
                tf_stamp = base + tf_index * 0.05
                bridge.process_iteration(t265=(Pose2D(0.0, 0.0, 0.0, tf_stamp), _good_t265()),
                                         pending_scan=None, sink=sink, state=state, now_mono=tf_stamp)
            measurement_s = base + 0.15 + 0.004
            bridge.process_iteration(t265=None, pending_scan=(_scan(), measurement_s),
                                     sink=sink, state=state, now_mono=measurement_s)

        metrics = bridge.metrics()
        self.assertEqual(metrics["slam.scan_publish_count"], 10)
        self.assertEqual(len(sink.scans), 10)
        self.assertEqual(metrics["slam.scan_drop_no_tf_window_count"], 0)
        self.assertEqual(metrics["slam.scan_drop_count"], 0)

    def test_scans_survive_a_t265_read_at_its_native_nineteen_hertz(self) -> None:
        # Regression for the 2026-10-05 run06: the T265 stream was consumed at
        # its native 19 Hz, so the newest TF stamp legitimately trailed a fresh
        # scan by ~52 ms and the fixed 20 ms tolerance threw away 155 of 292
        # scans (53 %).  That then starved slam_toolbox into >0.5 s anchor gaps
        # and made the fused state flap out of ok.
        bridge = SlamBridge(load_v2_config().d500_mount)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=0.0)
        for index in range(40):
            tf_stamp = 10.0 + index * 0.052
            bridge.process_iteration(t265=(Pose2D(0.0, 0.0, 0.0, tf_stamp), _good_t265()),
                                     pending_scan=None, sink=sink, state=state, now_mono=tf_stamp)
            # Assembled 40 ms after the newest TF: inside one T265 period, well
            # outside one 50 Hz TF period.
            measurement_s = tf_stamp + 0.040
            bridge.process_iteration(t265=None, pending_scan=(_scan(), measurement_s),
                                     sink=sink, state=state, now_mono=measurement_s)

        metrics = bridge.metrics()
        self.assertEqual(metrics["slam.scan_drop_no_tf_window_count"], 0)
        # 40 samples * 0.052 s = 2.08 s, rate limited to 5 Hz -> about 10 published.
        self.assertGreaterEqual(metrics["slam.scan_publish_count"], 9)

    def test_scan_before_the_first_tf_is_counted_as_no_window_not_silently_held(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=5.0)
        bridge.process_iteration(t265=None, pending_scan=(_scan(), 5.0),
                                 sink=sink, state=state, now_mono=5.0)
        metrics = bridge.metrics()
        self.assertEqual(sink.scans, [])
        self.assertEqual(metrics["slam.scan_drop_no_tf_window_count"], 1)
        self.assertIsNone(metrics["slam.first_tf_stamp_s"])
        self.assertIsNone(bridge._pending_scan)

    def test_stale_scan_inside_the_window_is_dropped_for_age(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=0.0)
        bridge.process_iteration(t265=(Pose2D(0.0, 0.0, 0.0, 1.0), _good_t265()),
                                 pending_scan=None, sink=sink, state=state, now_mono=1.0)
        bridge.process_iteration(t265=None, pending_scan=(_scan(), 1.001),
                                 sink=sink, state=state, now_mono=1.6)
        metrics = bridge.metrics()
        self.assertEqual(sink.scans, [])
        self.assertEqual(metrics["slam.scan_drop_count"], 1)
        self.assertEqual(metrics["slam.scan_drop_no_tf_window_count"], 0)
        self.assertAlmostEqual(metrics["slam.scan_age_ms"], 599.0)

    # ------------------------------------------------------------------
    # TF starvation watchdog
    # ------------------------------------------------------------------

    def test_watchdog_holds_the_last_known_pose_after_three_seconds_without_a_tf(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount, tf_starve_force_s=3.0)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=100.0)
        # The bridge knows a pose but no TF was ever published: this is exactly
        # the latched-lost case where slam.first_tf_stamp_s stays null.
        bridge.push_t265(Pose2D(1.0, 2.0, 0.3, 100.0), _good_t265())

        bridge.process_iteration(t265=None, pending_scan=None, sink=sink, state=state, now_mono=102.9)
        self.assertEqual(sink.tfs, [])
        self.assertIsNone(bridge.metrics()["slam.first_tf_stamp_s"])
        self.assertEqual(bridge.metrics()["slam.tf_starved_force_count"], 0)

        bridge.process_iteration(t265=None, pending_scan=None, sink=sink, state=state, now_mono=103.1)

        self.assertEqual(len(sink.tfs), 1)
        held_pose, stamp_s = sink.tfs[0]
        self.assertEqual(held_pose, Pose2D(1.0, 2.0, 0.3, 100.0))
        # Restamped at the current time: slam_toolbox looks the TF up at the
        # scan stamp, so replaying the stale stamp would not open the window.
        self.assertEqual(stamp_s, 103.1)
        metrics = bridge.metrics()
        self.assertEqual(metrics["slam.tf_starved_force_count"], 1)
        self.assertEqual(metrics["slam.first_tf_stamp_s"], 100.0)
        self.assertEqual(metrics["slam.last_tf_stamp_s"], 103.1)

    def test_forced_tf_reopens_the_scan_window_after_starvation(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount, tf_starve_force_s=3.0)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=100.0)
        bridge.push_t265(Pose2D(1.0, 2.0, 0.3, 100.0), _good_t265())
        bridge.process_iteration(t265=None, pending_scan=None, sink=sink, state=state, now_mono=103.1)

        bridge.process_iteration(t265=None, pending_scan=(_scan(), 103.05),
                                 sink=sink, state=state, now_mono=103.1)

        metrics = bridge.metrics()
        self.assertEqual(len(sink.scans), 1)
        self.assertEqual(metrics["slam.scan_publish_count"], 1)
        self.assertEqual(metrics["slam.scan_drop_no_tf_window_count"], 0)

    def test_live_poses_publish_again_after_a_forced_tf(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount, tf_starve_force_s=3.0)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=100.0)
        bridge.push_t265(Pose2D(1.0, 2.0, 0.3, 100.0), _good_t265())
        bridge.process_iteration(t265=None, pending_scan=None, sink=sink, state=state, now_mono=103.1)
        bridge.process_iteration(t265=(Pose2D(1.1, 2.0, 0.3, 103.2), _good_t265()),
                                 pending_scan=None, sink=sink, state=state, now_mono=103.2)

        self.assertEqual([stamp for _pose, stamp in sink.tfs], [103.1, 103.2])
        self.assertEqual(bridge.metrics()["slam.tf_starved_force_count"], 1)

    def test_watchdog_stays_silent_without_any_known_pose(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount, tf_starve_force_s=3.0)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=100.0)
        bridge.process_iteration(t265=None, pending_scan=None, sink=sink, state=state, now_mono=1000.0)
        self.assertEqual(sink.tfs, [])
        self.assertEqual(bridge.metrics()["slam.tf_starved_force_count"], 0)

    def test_watchdog_does_not_fire_while_tf_keeps_flowing(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount, tf_starve_force_s=3.0)
        sink = _RecordingSink()
        state = bridge.new_window_state(now_mono=0.0)
        for index in range(80):
            stamp = 1.0 + index * 0.05
            bridge.process_iteration(t265=(Pose2D(0.0, 0.0, 0.0, stamp), _good_t265()),
                                     pending_scan=None, sink=sink, state=state, now_mono=stamp)
        self.assertEqual(bridge.metrics()["slam.tf_starved_force_count"], 0)
        self.assertEqual([stamp for _pose, stamp in sink.tfs], [1.0 + i * 0.05 for i in range(80)])


if __name__ == "__main__":
    unittest.main()
