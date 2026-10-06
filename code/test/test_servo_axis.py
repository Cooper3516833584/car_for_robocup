"""Hardware-free tests for the single-axis PWM servo component.

Never opens a PWM channel: the axis is driven through ``FakeServoAxis`` and the
factory is only built in its fake mode.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import sys
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.servo_axis import (
    MG90S_TRAVEL,
    FakeServoAxis,
    ServoAxis,
    ServoError,
    ServoNotStartedError,
    ServoRangeError,
    ServoTravel,
    angle_to_pulse_us,
    pulse_to_angle_deg,
    travel_from_endpoints,
)
from config.v2_factory import build_servo
from config.v2_loader import load_v2_config
from config.v2_models import ConfigV2Error, ServoConfig

EXAMPLE_PROFILE = Path(__file__).resolve().parents[2] / "configs" / "robocup_diffdrive.example.toml"


class _Clock:
    """Controllable monotonic clock so rate limiting is deterministic."""

    def __init__(self) -> None:
        self.now = 1000.0
        self.slept: list[float] = []

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.slept.append(seconds)
        self.now += seconds


class ServoTravelTests(unittest.TestCase):
    def test_mg90s_angle_mapping_centres_zero_on_the_mid_pulse(self) -> None:
        # 0 degrees is the mid pulse by project convention, not an endpoint.
        self.assertEqual(angle_to_pulse_us(MG90S_TRAVEL, 0.0), 1500)
        self.assertEqual(angle_to_pulse_us(MG90S_TRAVEL, 90.0), 2500)
        self.assertEqual(angle_to_pulse_us(MG90S_TRAVEL, -90.0), 500)
        self.assertEqual(angle_to_pulse_us(MG90S_TRAVEL, 45.0), 2000)
        self.assertEqual(angle_to_pulse_us(MG90S_TRAVEL, -45.0), 1000)

    def test_pulse_to_angle_is_the_inverse(self) -> None:
        # Pulses are whole microseconds, so the round trip is lossless only at
        # the pulse level; the angle error must stay inside one step.
        step_deg = 1.0 / MG90S_TRAVEL.us_per_deg
        for angle in (-90.0, -30.0, 0.0, 17.5, 90.0):
            pulse = angle_to_pulse_us(MG90S_TRAVEL, angle)
            recovered = pulse_to_angle_deg(MG90S_TRAVEL, pulse)
            self.assertEqual(angle_to_pulse_us(MG90S_TRAVEL, recovered), pulse)
            self.assertLessEqual(abs(recovered - angle), step_deg)

    def test_angle_outside_travel_fails_closed(self) -> None:
        for angle in (-90.0001, 90.5, 360.0, float("nan"), float("inf")):
            with self.assertRaises(ServoRangeError):
                angle_to_pulse_us(MG90S_TRAVEL, angle)

    def test_pulse_outside_travel_fails_closed(self) -> None:
        for pulse in (499, 2501, 0, float("nan")):
            with self.assertRaises(ServoRangeError):
                pulse_to_angle_deg(MG90S_TRAVEL, pulse)

    def test_invalid_calibrations_are_rejected(self) -> None:
        with self.assertRaises(ServoRangeError):
            ServoTravel(half_range_deg=0.0)
        with self.assertRaises(ServoRangeError):
            ServoTravel(pulse_min_us=2500, pulse_max_us=500)
        with self.assertRaises(ServoRangeError):
            ServoTravel(pulse_min_us=-1, pulse_max_us=2500)
        with self.assertRaises(ServoRangeError):
            ServoTravel(pulse_min_us=500.0, pulse_max_us=2500)  # type: ignore[arg-type]

    def test_a_pulse_reaching_the_whole_frame_is_rejected(self) -> None:
        # Equal to the frame period, so the driver would emit no low period at
        # all and the servo would never see a valid frame.
        with self.assertRaises(ServoRangeError):
            ServoTravel(pulse_min_us=2000, pulse_max_us=20_000, period_us=20_000)

    def test_an_unusually_wide_but_legal_pulse_range_is_accepted(self) -> None:
        # The component enforces signal legality, not the MG90S's mechanical
        # range: a wide range is a calibration choice, not an error.
        travel = ServoTravel(pulse_min_us=500, pulse_max_us=3000, period_us=20_000)

        self.assertEqual(travel.pulse_mid_us, 1750.0)
        self.assertEqual(angle_to_pulse_us(travel, 0.0), 1750)

    def test_asymmetric_endpoints_recentre_zero_on_the_mid_pulse(self) -> None:
        # A servo whose usable range is -20..+70 degrees still puts 0 degrees on
        # the mid pulse; only the pulse span per degree changes.
        travel = travel_from_endpoints(
            pulse_min_us=1000, pulse_max_us=2000, min_angle_deg=-20.0, max_angle_deg=70.0
        )
        self.assertEqual(travel.pulse_mid_us, 1500.0)
        self.assertAlmostEqual(travel.half_range_deg, 45.0)
        self.assertEqual(angle_to_pulse_us(travel, 0.0), 1500)
        self.assertEqual(angle_to_pulse_us(travel, 45.0), 2000)
        self.assertEqual(angle_to_pulse_us(travel, -45.0), 1000)

    def test_asymmetric_endpoints_require_an_increasing_pair(self) -> None:
        with self.assertRaises(ServoRangeError):
            travel_from_endpoints(
                pulse_min_us=1000, pulse_max_us=2000, min_angle_deg=70.0, max_angle_deg=20.0
            )


class ServoAxisTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = _Clock()
        self.axis = FakeServoAxis(
            MG90S_TRAVEL, settle_s=0.25, clock=self.clock, sleep=self.clock.sleep
        )

    def test_construction_does_not_touch_hardware(self) -> None:
        self.assertFalse(self.axis.is_running)
        self.assertEqual(self.axis.commands, [])
        self.assertIsNone(self.axis.angle_deg)

    def test_start_homes_to_zero_degrees(self) -> None:
        self.axis.start()

        self.assertTrue(self.axis.is_running)
        self.assertEqual(self.axis.commands, [1500])
        self.assertEqual(self.axis.angle_deg, 0.0)
        self.assertEqual(self.axis.pulse_us, 1500)

    def test_start_can_enable_without_moving(self) -> None:
        self.axis.start(home=False)

        self.assertTrue(self.axis.is_running)
        self.assertEqual(self.axis.commands, [])
        self.assertIsNone(self.axis.angle_deg)

    def test_start_is_idempotent(self) -> None:
        self.axis.start()
        self.axis.start()

        self.assertEqual(self.axis.commands, [1500])

    def test_home_angle_is_configurable(self) -> None:
        axis = FakeServoAxis(MG90S_TRAVEL, home_angle_deg=-30.0, clock=self.clock, sleep=self.clock.sleep)
        axis.start()

        self.assertEqual(axis.commands, [angle_to_pulse_us(MG90S_TRAVEL, -30.0)])
        self.assertEqual(axis.angle_deg, -30.0)

    def test_start_without_homing_writes_the_first_command_even_at_zero(self) -> None:
        self.axis.start(home=False)
        self.axis.set_angle(0.0)

        self.assertEqual(self.axis.commands, [1500])

    def test_commands_before_start_are_refused(self) -> None:
        with self.assertRaises(ServoNotStartedError):
            self.axis.set_angle(10.0)
        with self.assertRaises(ServoNotStartedError):
            self.axis.nudge(10.0)
        with self.assertRaises(ServoNotStartedError):
            self.axis.apply_angle(10.0)

    def test_set_angle_writes_the_calibrated_pulse_for_any_angle_in_range(self) -> None:
        self.axis.start()
        for angle in (-90.0, -45.0, -1.0, 0.0, 1.0, 45.0, 90.0):
            self.clock.now += 1.0  # stay clear of the command rate limit
            pulse = self.axis.set_angle(angle)
            self.assertEqual(pulse, angle_to_pulse_us(MG90S_TRAVEL, angle))
        self.assertEqual(self.axis.commands[0], 1500)
        self.assertEqual(self.axis.commands[-1], 2500)
        self.assertEqual(len(self.axis.commands), 8)

    def test_set_angle_outside_travel_is_refused_without_writing(self) -> None:
        self.axis.start()
        written = list(self.axis.commands)

        with self.assertRaises(ServoRangeError):
            self.axis.set_angle(90.5)

        self.assertEqual(self.axis.commands, written)
        self.assertEqual(self.axis.angle_deg, 0.0)

    def test_repeated_angle_does_not_rewrite_the_same_pulse(self) -> None:
        self.axis.start()
        self.clock.now += 1.0
        self.axis.set_angle(30.0)
        count = len(self.axis.commands)

        self.clock.now += 1.0
        self.axis.set_angle(30.0)

        self.assertEqual(len(self.axis.commands), count)

    def test_set_angle_is_rate_limited(self) -> None:
        self.axis.start()
        self.clock.now += 1.0
        self.axis.set_angle(10.0)

        self.clock.now += 0.005  # below the 20 ms minimum interval
        with self.assertRaises(ServoError):
            self.axis.set_angle(20.0)

        self.clock.now += 0.02
        self.assertEqual(self.axis.set_angle(20.0), angle_to_pulse_us(MG90S_TRAVEL, 20.0))

    def test_settle_waits_for_the_configured_duration(self) -> None:
        self.axis.start()
        self.clock.now += 1.0
        self.axis.set_angle(10.0, settle=True)

        self.assertEqual(self.clock.slept, [0.25])

    def test_nudge_moves_relative_to_the_last_angle(self) -> None:
        self.axis.start()
        self.clock.now += 1.0
        self.axis.nudge(15.0)
        self.assertEqual(self.axis.angle_deg, 15.0)

        self.clock.now += 1.0
        self.axis.nudge(-5.0)

        self.assertEqual(self.axis.angle_deg, 10.0)
        self.assertEqual(self.axis.pulse_us, angle_to_pulse_us(MG90S_TRAVEL, 10.0))

    def test_nudge_past_the_limit_is_refused(self) -> None:
        self.axis.start()
        self.clock.now += 1.0
        self.axis.set_angle(85.0)

        self.clock.now += 1.0
        with self.assertRaises(ServoRangeError):
            self.axis.nudge(10.0)

    def test_release_stops_the_pulse_train(self) -> None:
        self.axis.start()
        self.axis.release()

        self.assertFalse(self.axis.is_running)
        self.assertEqual(self.axis.disabled_count, 1)
        # An explicit release is an opt-out, so it must be possible at any time.
        self.axis.release()
        self.assertEqual(self.axis.disabled_count, 1)

    def test_config_has_no_separate_release_flag(self) -> None:
        # Holding is the component's default, so a second flag that only encoded
        # the same choice was removed rather than left as dead config.
        self.assertFalse(hasattr(ServoConfig(), "release_on_close"))

    def test_close_holds_the_axis_by_default(self) -> None:
        # Holding is the default: releasing a loaded axis silently loses the
        # angle the caller just commanded.
        self.axis.start()
        self.axis.close()

        self.assertTrue(self.axis.closed)
        self.assertEqual(self.axis.disabled_count, 0)

    def test_close_can_release_when_asked(self) -> None:
        self.axis.start()
        self.axis.close(hold=False)

        self.assertTrue(self.axis.closed)
        self.assertEqual(self.axis.disabled_count, 1)

    def test_hold_requires_a_running_axis(self) -> None:
        with self.assertRaises(ServoNotStartedError):
            self.axis.hold()

    def test_hold_keeps_the_axis_enabled(self) -> None:
        self.axis.start()
        self.axis.hold()

        self.assertTrue(self.axis.is_running)
        self.assertEqual(self.axis.disabled_count, 0)

    def test_context_manager_homes_and_keeps_holding(self) -> None:
        with FakeServoAxis(MG90S_TRAVEL, clock=self.clock, sleep=self.clock.sleep) as axis:
            self.assertTrue(axis.is_running)
            self.assertEqual(axis.pulse_us, 1500)
        # Leaving the block must not free a loaded axis.
        self.assertEqual(axis.disabled_count, 0)
        self.assertTrue(axis.closed)

    def test_axis_requires_a_pwm_output(self) -> None:
        with self.assertRaises(ValueError):
            ServoAxis(MG90S_TRAVEL)
        with self.assertRaises(ValueError):
            ServoAxis(MG90S_TRAVEL, pwm=None)

    def test_fake_axis_refuses_an_injected_pwm_output(self) -> None:
        with self.assertRaises(ValueError):
            FakeServoAxis(MG90S_TRAVEL, pwm=None)  # type: ignore[arg-type]

    def test_home_angle_outside_travel_is_rejected_at_construction(self) -> None:
        with self.assertRaises(ServoRangeError):
            FakeServoAxis(MG90S_TRAVEL, home_angle_deg=91.0)

    def test_float_angle_in_the_middle_of_the_span_is_accepted(self) -> None:
        self.axis.start()
        self.clock.now += 1.0
        pulse = self.axis.set_angle(22.5)

        self.assertEqual(pulse, 1750)
        step_deg = 1.0 / MG90S_TRAVEL.us_per_deg
        self.assertLessEqual(abs(self.axis.angle_deg_for(pulse) - 22.5), step_deg)


class _RealisticClock:
    """Clock whose value only advances by what was actually slept.

    Unlike :class:`_Clock` the caller never moves it by hand, which is what a
    real ``time.monotonic`` does: this is what catches a sweep that books delays
    in advance and ends up reporting a command time in the future.
    """

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class ServoSweepTimingTests(unittest.TestCase):
    """Timing bookkeeping must match a real monotonic clock, not a plan."""

    def setUp(self) -> None:
        self.clock = _RealisticClock()
        self.axis = FakeServoAxis(
            MG90S_TRAVEL, clock=self.clock, sleep=self.clock.sleep
        )
        self.axis.start()
        self.clock.now += 1.0
        self.axis.set_angle(-90.0)
        self.clock.now += 1.0

    def test_sweep_does_not_book_time_in_the_future(self) -> None:
        # Regression: the sweep used to advance its own notion of "now" by the
        # planned step delays, so _last_command_at ended up ahead of the real
        # clock and every following command was rejected as rate limited.
        self.axis.sweep_to(90.0, rate_deg_s=30.0)

        self.assertLessEqual(self.axis._last_command_at, self.clock.now)

    def test_sweep_takes_the_expected_wall_time(self) -> None:
        started = self.clock.now
        self.axis.sweep_to(90.0, rate_deg_s=30.0)

        self.assertAlmostEqual(self.clock.now - started, 6.0, places=6)

    def test_two_sweeps_can_run_back_to_back(self) -> None:
        # The real operator sequence: park at -90, cross to +90 at 30 deg/s,
        # then come back to -90 at 30 deg/s.
        clock = _RealisticClock()
        axis = FakeServoAxis(
            MG90S_TRAVEL, home_angle_deg=-90.0, clock=clock, sleep=clock.sleep
        )
        axis.start(home=False, sweep_start=True)
        axis.sweep_to(90.0, rate_deg_s=30.0)
        clock.now += 0.5
        pulse = axis.return_to_start(rate_deg_s=30.0)

        self.assertEqual(pulse, 500)
        self.assertEqual(axis.angle_deg, -90.0)

    def test_sweep_then_immediate_set_angle_is_rejected_then_accepted(self) -> None:
        self.axis.sweep_to(90.0, rate_deg_s=30.0)

        with self.assertRaises(ServoError):
            self.axis.set_angle(0.0)

        self.clock.now += 0.021
        self.assertEqual(self.axis.set_angle(0.0), angle_to_pulse_us(MG90S_TRAVEL, 0.0))

    def test_settle_also_ends_before_the_recorded_command_time(self) -> None:
        self.axis.sweep_to(90.0, rate_deg_s=30.0, settle=True)

        self.assertLessEqual(self.axis._last_command_at, self.clock.now)
    """Rate-controlled motion: the rate must hold regardless of step size."""

    def setUp(self) -> None:
        self.clock = _Clock()
        self.axis = FakeServoAxis(
            MG90S_TRAVEL, settle_s=0.25, clock=self.clock, sleep=self.clock.sleep
        )
        self.axis.start()
        self.clock.now += 1.0
        self.axis.set_angle(-90.0)
        self.clock.now += 1.0

    def test_sweep_crosses_at_the_requested_rate(self) -> None:
        # -90 -> +90 is 180 deg, so 30 deg/s must take 6 s of wall time.
        started = self.clock.now
        pulse = self.axis.sweep_to(90.0, rate_deg_s=30.0)

        self.assertEqual(pulse, 2500)
        self.assertEqual(self.axis.angle_deg, 90.0)
        self.assertAlmostEqual(self.clock.now - started, 6.0, places=9)

    def test_step_size_changes_command_count_but_not_the_rate(self) -> None:
        coarse = FakeServoAxis(MG90S_TRAVEL, clock=self.clock, sleep=self.clock.sleep)
        coarse.start()
        self.clock.now += 1.0
        coarse.set_angle(-90.0)
        self.clock.now += 1.0
        started = self.clock.now
        coarse.sweep_to(90.0, rate_deg_s=30.0, step_deg=5.0)

        self.assertEqual(len(coarse.commands), 2 + 36)  # home + -90 setup + 180/5 steps
        self.assertAlmostEqual(self.clock.now - started, 6.0, places=9)

    def test_sweep_ends_exactly_on_the_requested_angle(self) -> None:
        # A step size that does not divide the span must still land on target.
        self.axis.sweep_to(47.0, rate_deg_s=100.0, step_deg=7.0)

        self.assertEqual(self.axis.angle_deg, 47.0)
        self.assertEqual(self.axis.pulse_us, angle_to_pulse_us(MG90S_TRAVEL, 47.0))

    def test_sweep_is_monotonic_towards_the_target(self) -> None:
        self.axis.sweep_to(90.0, rate_deg_s=200.0, step_deg=10.0)
        sent = self.axis.commands[2:]  # drop home and the -90 setup command

        self.assertEqual(sent, sorted(sent))
        self.assertEqual(sent[0], angle_to_pulse_us(MG90S_TRAVEL, -80.0))

    def test_sweep_to_the_current_angle_writes_nothing(self) -> None:
        before = len(self.axis.commands)
        started = self.clock.now
        pulse = self.axis.sweep_to(-90.0, rate_deg_s=30.0)

        self.assertEqual(len(self.axis.commands), before)
        self.assertEqual(self.clock.now, started)
        self.assertEqual(pulse, 500)

    def test_sweep_returns_the_target_pulse_when_nothing_moved(self) -> None:
        # Regression: an empty plan must not fall back to a stale pulse value.
        self.assertEqual(self.axis.sweep_to(-90.0, rate_deg_s=30.0), 500)

    def test_sweep_by_moves_relative_to_the_current_angle(self) -> None:
        self.axis.sweep_by(45.0, rate_deg_s=90.0, step_deg=5.0)

        self.assertEqual(self.axis.angle_deg, -45.0)
        self.assertEqual(self.axis.pulse_us, angle_to_pulse_us(MG90S_TRAVEL, -45.0))

    def test_return_to_start_uses_the_recorded_start_angle(self) -> None:
        self.axis.sweep_to(90.0, rate_deg_s=180.0, step_deg=10.0)
        started = self.clock.now
        pulse = self.axis.return_to_start(rate_deg_s=30.0)

        self.assertEqual(self.axis.angle_deg, 0.0)  # start() homes to 0
        self.assertEqual(pulse, 1500)
        self.assertAlmostEqual(self.clock.now - started, 3.0, places=9)

    def test_return_to_start_without_a_recorded_angle_is_refused(self) -> None:
        blind = FakeServoAxis(MG90S_TRAVEL, clock=self.clock, sleep=self.clock.sleep)
        blind.start(home=False)

        self.assertIsNone(blind.start_angle_deg)
        with self.assertRaises(ServoNotStartedError):
            blind.return_to_start(rate_deg_s=30.0)

    def test_sweep_start_records_the_origin_without_homing_to_zero(self) -> None:
        axis = FakeServoAxis(
            MG90S_TRAVEL, home_angle_deg=-90.0, clock=self.clock, sleep=self.clock.sleep
        )
        axis.start(home=False, sweep_start=True)

        # The origin is recorded but nothing is written, so the caller's own
        # first command is not blocked by the rate limiter.
        self.assertEqual(axis.start_angle_deg, -90.0)
        self.assertEqual(axis.commands, [])
        self.assertIsNone(axis.pulse_us)

        axis.sweep_to(90.0, rate_deg_s=30.0)
        started = self.clock.now
        axis.return_to_start(rate_deg_s=30.0)

        self.assertEqual(axis.angle_deg, -90.0)
        self.assertAlmostEqual(self.clock.now - started, 6.0, places=9)

    def test_sweep_start_then_immediate_set_angle_is_accepted(self) -> None:
        # Regression: start(sweep_start=True) used to write a pulse, which made
        # the operator CLI's very first move fail the command rate limiter.
        axis = FakeServoAxis(MG90S_TRAVEL, clock=self.clock, sleep=self.clock.sleep)
        axis.start(home=False, sweep_start=True)

        self.assertEqual(axis.set_angle(-90.0), 500)
        self.assertEqual(axis.angle_deg, -90.0)

    def test_step_never_shorter_than_one_pwm_frame(self) -> None:
        # 1000 deg/s with a 1 deg step would be 1 ms per step; the frame period
        # is the floor, so the sweep runs slower than requested rather than
        # hammering the channel faster than it can refresh.
        started = self.clock.now
        self.axis.sweep_to(10.0, rate_deg_s=1000.0, step_deg=1.0)

        # 100 steps of one 20 ms frame each.
        self.assertAlmostEqual(self.clock.now - started, 2.0, places=9)

    def test_sweep_rejects_non_positive_rate_or_step(self) -> None:
        for rate in (0.0, -30.0, float("nan"), float("inf")):
            with self.assertRaises(ServoError):
                self.axis.sweep_to(0.0, rate_deg_s=rate)
        for step in (0.0, -1.0, float("nan")):
            with self.assertRaises(ServoError):
                self.axis.sweep_to(0.0, rate_deg_s=30.0, step_deg=step)

    def test_sweep_outside_travel_is_refused_before_moving(self) -> None:
        before = len(self.axis.commands)

        with self.assertRaises(ServoRangeError):
            self.axis.sweep_to(120.0, rate_deg_s=30.0)

        self.assertEqual(len(self.axis.commands), before)
        self.assertEqual(self.axis.angle_deg, -90.0)

    def test_sweep_before_start_is_refused(self) -> None:
        idle = FakeServoAxis(MG90S_TRAVEL, clock=self.clock, sleep=self.clock.sleep)
        with self.assertRaises(ServoNotStartedError):
            idle.sweep_to(0.0, rate_deg_s=30.0)

    def test_sweep_then_set_angle_is_not_blocked_by_the_rate_limiter(self) -> None:
        # The sweep paces itself, and the clock it advanced is what the ordinary
        # command paths consult, so a move issued just after it is accepted once
        # the minimum interval has elapsed (the limiter compares strictly).
        self.axis.sweep_to(0.0, rate_deg_s=90.0, step_deg=10.0)
        self.clock.now += 0.021

        self.assertEqual(self.axis.set_angle(-80.0), angle_to_pulse_us(MG90S_TRAVEL, -80.0))

    def test_rate_limited_move_is_still_rejected_right_after_a_sweep(self) -> None:
        # A sweep paces itself and deliberately bypasses the limiter, but the
        # limiter still applies to the next ordinary command, which is issued in
        # the same instant the sweep ended.
        self.axis.sweep_to(0.0, rate_deg_s=90.0, step_deg=10.0)

        with self.assertRaises(ServoError):
            self.axis.set_angle(10.0)

        self.clock.now += 0.021
        self.assertEqual(self.axis.set_angle(10.0), angle_to_pulse_us(MG90S_TRAVEL, 10.0))


class ServoConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        # Keep scratch profiles inside the checkout: the DSH file sandbox does
        # not always grant write access to the system temp directory.
        self._scratch = Path(__file__).resolve().parent / f"_servo_tmp_{uuid.uuid4().hex}"
        self._scratch.mkdir()
        self.addCleanup(shutil.rmtree, self._scratch, ignore_errors=True)

    def _write_profile(self, text: str) -> Path:
        path = self._scratch / "profile.toml"
        path.write_text(text, encoding="utf-8")
        return path

    def test_config_defaults_to_disabled_with_a_device_tree_selector(self) -> None:
        config = ServoConfig()

        self.assertFalse(config.enabled)
        # Only the device-tree node name: a numeric fallback could bind an
        # unrelated controller (on the ROCK 5A, pwmchip0 is D500's fd8b0000.pwm).
        self.assertEqual(config.chip_device_match, ("febd0030.pwm",))
        self.assertEqual(config.pulse_min_us, 500)
        self.assertEqual(config.pulse_max_us, 2500)
        self.assertEqual(config.period_us, 20_000)

    def test_example_profile_carries_an_optional_disabled_servo_table(self) -> None:
        config = load_v2_config(EXAMPLE_PROFILE)

        self.assertFalse(config.servo.enabled)
        self.assertEqual(config.servo.channel, 0)

    def test_disabled_servo_builds_nothing(self) -> None:
        self.assertIsNone(build_servo(load_v2_config(EXAMPLE_PROFILE)))

    def test_enabled_servo_builds_the_in_memory_axis_in_fake_mode(self) -> None:
        config = load_v2_config(EXAMPLE_PROFILE)
        object.__setattr__(config.servo, "enabled", True)

        axis = build_servo(config, fake=True)

        self.assertIsInstance(axis, FakeServoAxis)
        axis.start()
        self.assertEqual(axis.pulse_us, 1500)

    def test_enabled_servo_without_a_selector_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "chip_device_match"):
            ServoConfig(enabled=True, chip_device_match=())

    def test_pulse_bounds_must_fit_the_frame(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "pulse_max_us"):
            ServoConfig(pulse_min_us=500, pulse_max_us=25_000, period_us=20_000)

    def test_home_angle_must_lie_inside_the_travel(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "home_angle_deg"):
            ServoConfig(home_angle_deg=120.0)

    def test_profile_without_a_servo_table_still_loads_disabled(self) -> None:
        document = EXAMPLE_PROFILE.read_text(encoding="utf-8")
        start = document.index("# Single-axis PWM hobby servo")
        end = document.index("[sensors.t265.mount]")
        trimmed = document[:start] + document[end:]

        config = load_v2_config(self._write_profile(trimmed))

        self.assertFalse(config.servo.enabled)

    def test_unknown_servo_key_is_rejected(self) -> None:
        document = EXAMPLE_PROFILE.read_text(encoding="utf-8")
        altered = document.replace("channel = 0", "channel = 0\nnot_a_key = 1", 1)

        with self.assertRaisesRegex(ConfigV2Error, "devices.servo"):
            load_v2_config(self._write_profile(altered))


if __name__ == "__main__":
    unittest.main()
