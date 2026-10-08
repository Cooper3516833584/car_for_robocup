"""Explicit filming mode: bounded command/time motion without position sensors.

This mode estimates travel from limited commands; it never reads or invents a
T265/SLAM pose and never changes the normal localization runtime's policies.
"""

import math
import sys
import time

from center_target_route import EntryLatch, PERIOD_S
from components.payload_task import drop_payload, payload_channel, prepare_payload
from core.types import Twist2D

PATROL_STAGES = (("forward_280cm", 2.80), ("left_90deg_1", math.pi / 2),
                 ("forward_420cm", 4.20), ("left_90deg_2", math.pi / 2),
                 ("forward_250cm", 2.50))


class PayloadDemoSession:
    """Own only the existing drive/relay; no localization workers or mission step."""

    def __init__(self, drive, relay, *, event_logger=None, verify_relay=True,
                 clock=time.monotonic):
        self.drive, self.relay = drive, relay
        self.event_logger, self.verify_relay = event_logger, verify_relay
        self.clock, self.started_at = clock, clock()

    def record_event(self, event, **data):
        if self.event_logger is not None:
            self.event_logger.emit({"t": self.clock() - self.started_at,
                                    "type": event, **data}, priority=True)

    def close(self):
        try:
            self.drive.stop()
        finally:
            try:
                if self.relay is not None and self.relay.connected:
                    try:
                        if self.relay.all_off(verify=self.verify_relay) is False:
                            raise RuntimeError("demo relay all_off was not confirmed")
                    finally:
                        self.relay.close()
            finally:
                self.drive.close()


def run_payload_demo(session, vision, settings, *, abort=lambda: False,
                     max_seconds=300, speed_m_s=0.15, turn_rad_s=0.40,
                     clock=time.monotonic, sleep=time.sleep):
    """Patrol, slow on visibility, execute real release at horizontal centre.

    Distances and angles are command/time estimates, not physical measurements.
    STOP, deadline, stale camera, relay failures and drive watchdog remain live.
    """
    drive, relay = session.drive, session.relay
    for name, value, limit in (("demo speed", speed_m_s, drive.max_linear_speed_m_s),
                               ("demo turn speed", turn_rad_s, drive.max_angular_speed_rad_s)):
        if not math.isfinite(value) or not 0 < value <= limit:
            raise ValueError(f"{name} must be positive and within the configured drive limit")
    if not math.isfinite(max_seconds) or not 1 <= max_seconds <= 600:
        raise ValueError("demo deadline must be between 1 and 600 seconds")
    deadline = clock() + max_seconds
    latch, last_frame = EntryLatch(), None
    drops = 0

    def guard(*, check_vision=True):
        if abort():
            raise RuntimeError("payload demo interrupted")
        if clock() >= deadline:
            raise RuntimeError("payload demo time limit expired")
        if check_vision:
            vision.observe(clock())

    def tick(v, omega, remaining):
        guard()
        drive.command(Twist2D(v, omega))
        limited = drive.last_limited_twist
        rate = abs(limited.linear_x_m_s if v else limited.angular_z_rad_s)
        interval = min(PERIOD_S, remaining / rate) if rate else PERIOD_S
        before = clock()
        sleep(interval)
        # Long scheduling gaps cannot count as travel: watchdog may have stopped.
        dt = min(max(0.0, clock() - before), interval)
        guard()
        return rate * dt

    def hold(seconds):
        drive.stop()
        until = clock() + seconds
        while clock() < until:
            guard()
            sleep(min(PERIOD_S, until - clock()))
        guard()

    def move(label, amount, *, reverse=False, turn=False, right=False, speed=None):
        remaining = abs(amount)
        session.record_event("demo_stage_start", stage=label, nominal_amount=amount)
        print(f"[demo] START {label}", flush=True)
        while remaining > 1e-6:
            rate = turn_rad_s if turn else (speed or speed_m_s)
            travelled = tick(0.0 if turn else (-rate if reverse else rate),
                             (-rate if right else rate) if turn else 0.0, remaining)
            remaining = max(0.0, remaining - travelled)
        drive.stop()
        session.record_event("demo_stage_done", stage=label)
        print(f"[demo] DONE {label}", flush=True)

    def detour():
        move("advance_7cm", settings.advance_m,
             speed=min(speed_m_s, settings.patrol_slow_speed_m_s))
        move("payload_left_90deg", math.pi / 2, turn=True)
        hold(1.0)
        move("forward_47cm", settings.approach_m)
        drive.stop()
        guard()
        channel = payload_channel(settings.payload_slot)
        session.record_event("payload_release_start", slot=settings.payload_slot, channel=channel)
        print(f"[demo] RELEASE slot={settings.payload_slot}, CH{channel} OFF", flush=True)
        if not drop_payload(relay, settings.payload_slot, hold_s=settings.release_hold_s,
                            verify=settings.verify_relay, sleep=hold):
            # drop_payload handles relay errors; recheck cancellation if its hold failed.
            guard()
            raise RuntimeError("demo payload release failed")
        session.record_event("payload_release_done", slot=settings.payload_slot, channel=channel)
        move("reverse_47cm", settings.approach_m, reverse=True)
        move("payload_right_90deg", math.pi / 2, turn=True, right=True)
        hold(1.0)

    try:
        if relay is None or relay.channel_count < payload_channel(settings.payload_slot):
            raise RuntimeError("demo payload relay missing")
        session.record_event("demo_position_checks_disabled", motion_reference="limited_command_time",
                             speed_m_s=speed_m_s, turn_rad_s=turn_rad_s)
        vision_deadline = min(deadline, clock() + 60)
        while not vision.ready:
            guard(check_vision=False)
            if clock() >= vision_deadline:
                raise RuntimeError("demo YOLO startup timed out")
            sleep(PERIOD_S)
        guard()
        if not relay.connected:
            relay.open()
        # The operator may already have attached a load to the energized magnet.
        # Keep that contact ON while disabling only unused channels.
        selected = payload_channel(settings.payload_slot)
        for channel in range(1, relay.channel_count + 1):
            if channel != selected and relay.turn_off(channel, verify=settings.verify_relay) is False:
                raise RuntimeError("demo unused relay deactivation was not confirmed")
        if not prepare_payload(relay, settings.payload_slot, verify=settings.verify_relay):
            raise RuntimeError("demo payload holding state was not confirmed")
        guard()
        drive.start()
        for label, amount in PATROL_STAGES:
            if label.startswith("left"):
                move(label, amount, turn=True)
                hold(1.0)
                continue
            session.record_event("demo_stage_start", stage=label, nominal_amount=amount)
            print(f"[demo] START {label}", flush=True)
            remaining = amount
            while remaining > 1e-6:
                guard()
                frame, visible, centered = vision.observe_drop(clock())
                entry = None
                if frame != last_frame:
                    last_frame = frame
                    entry = latch.update(visible, trigger=centered is not None)
                if entry is not None:
                    session.record_event("test_route_payload_target", label=label, box=centered)
                    detour()
                    drops += 1
                    remaining = max(0.0, remaining - settings.advance_m)
                    latch.require_clear()
                    last_frame = vision.observe_drop(clock())[0]
                    session.record_event("test_route_resume", label=label, remaining_nominal_m=remaining)
                    continue
                speed = min(speed_m_s, settings.patrol_slow_speed_m_s) if visible else speed_m_s
                remaining = max(0.0, remaining - tick(speed, 0.0, remaining))
            drive.stop()
            session.record_event("demo_stage_done", stage=label)
            print(f"[demo] DONE {label}", flush=True)
        session.record_event("test_route_finished", drop_count=drops, position_checks=False)
        print(f"[demo] FINISHED; drops={drops}", flush=True)
        return drops
    except BaseException as exc:
        session.record_event("demo_stopped", reason=str(exc))
        raise
    finally:
        drive.stop()
        if relay is not None and relay.connected:
            primary_error = sys.exc_info()[0] is not None
            try:
                if relay.all_off(verify=settings.verify_relay) is False:
                    raise RuntimeError("demo final relay all_off was not confirmed")
            except Exception as exc:
                session.record_event("demo_cleanup_failed", reason=str(exc))
                if not primary_error:
                    raise
