"""Safe production facade for differential-drive motion commands."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable

from core.types import Twist2D, WheelSpeeds

from .c10b_diff_backend import DriveBackend, UnsupportedFirmwareMotion
from .differential_kinematics import (
    DifferentialGeometry,
    DifferentialKinematics,
    scale_wheels_to_limit,
)
from .hardware_control_lock import HardwareControlLock


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive(name: str, value: float) -> float:
    result = _finite(name, value)
    if result <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return result


def _rate_limit(previous: float, target: float, acceleration: float, dt: float) -> float:
    change = acceleration * dt
    return min(previous + change, max(previous - change, target))


class DifferentialDrive:
    """Clamp, rate-limit, convert and safely send canonical ``Twist2D`` input."""

    def __init__(
        self,
        backend: DriveBackend,
        *,
        geometry: DifferentialGeometry,
        max_wheel_speed_m_s: float,
        max_linear_speed_m_s: float,
        max_angular_speed_rad_s: float,
        max_linear_accel_m_s2: float,
        max_angular_accel_rad_s2: float,
        command_timeout_s: float,
        clock: Callable[[], float] = time.monotonic,
        hardware_lock_path: str | None = None,
        max_accel_dt_s: float = 0.5,
        watchdog_period_s: float = 0.02,
    ) -> None:
        self.backend = backend
        self.kinematics = DifferentialKinematics(geometry)
        self.max_wheel_speed_m_s = _positive("max_wheel_speed_m_s", max_wheel_speed_m_s)
        self.max_linear_speed_m_s = _positive("max_linear_speed_m_s", max_linear_speed_m_s)
        self.max_angular_speed_rad_s = _positive("max_angular_speed_rad_s", max_angular_speed_rad_s)
        self.max_linear_accel_m_s2 = _positive("max_linear_accel_m_s2", max_linear_accel_m_s2)
        self.max_angular_accel_rad_s2 = _positive("max_angular_accel_rad_s2", max_angular_accel_rad_s2)
        self.command_timeout_s = _positive("command_timeout_s", command_timeout_s)
        self.max_accel_dt_s = _positive("max_accel_dt_s", max_accel_dt_s)
        self.watchdog_period_s = _positive("watchdog_period_s", watchdog_period_s)
        self._clock = clock
        self._hardware_lock = HardwareControlLock(hardware_lock_path)
        self._lock = threading.RLock()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._started = False
        self._last_twist = Twist2D(0.0, 0.0)
        self._last_update_s: float | None = None
        self._last_command_s: float | None = None
        self._watchdog_stop_count = 0

    @property
    def is_running(self) -> bool:
        return self._started

    @property
    def watchdog_stop_count(self) -> int:
        with self._lock:
            return self._watchdog_stop_count

    @property
    def last_limited_twist(self) -> Twist2D:
        with self._lock:
            return self._last_twist

    def start(self) -> "DifferentialDrive":
        with self._lock:
            if self._started:
                raise RuntimeError("DifferentialDrive is already running")
            self._hardware_lock.acquire()
            try:
                self.backend.start()
            except BaseException:
                self._hardware_lock.release()
                raise
            now = _finite("clock", self._clock())
            self._last_twist = Twist2D(0.0, 0.0)
            self._last_update_s = now
            self._last_command_s = None
            self._watchdog_stop.clear()
            self._started = True
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_loop,
                name="differential-drive-watchdog",
                daemon=True,
            )
            self._watchdog_thread.start()
        return self

    def __enter__(self) -> "DifferentialDrive":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def command(self, twist: Twist2D, *, now_s: float | None = None) -> None:
        linear = _finite("linear_x_m_s", twist.linear_x_m_s)
        angular = _finite("angular_z_rad_s", twist.angular_z_rad_s)
        now = _finite("now_s", self._clock() if now_s is None else now_s)
        with self._lock:
            self._require_started()
            if self._last_update_s is not None and now < self._last_update_s:
                raise ValueError("now_s must not move backwards")
            if self._last_command_s is not None and now - self._last_command_s >= self.command_timeout_s:
                self._stop_locked()
                self._last_update_s = now

            dt = 0.0 if self._last_update_s is None else min(now - self._last_update_s, self.max_accel_dt_s)
            target = Twist2D(
                max(-self.max_linear_speed_m_s, min(linear, self.max_linear_speed_m_s)),
                max(-self.max_angular_speed_rad_s, min(angular, self.max_angular_speed_rad_s)),
            )
            limited = Twist2D(
                _rate_limit(self._last_twist.linear_x_m_s, target.linear_x_m_s, self.max_linear_accel_m_s2, dt),
                _rate_limit(self._last_twist.angular_z_rad_s, target.angular_z_rad_s, self.max_angular_accel_rad_s2, dt),
            )
            wheels = scale_wheels_to_limit(
                self.kinematics.twist_to_wheels(limited), self.max_wheel_speed_m_s
            )
            actual = self.kinematics.wheels_to_twist(wheels)
            try:
                self.backend.command_wheel_speeds(wheels)
            except BaseException:
                try:
                    self.backend.stop()
                except BaseException:
                    pass
                self._reset_motion_state(now)
                raise
            self._last_twist = actual
            self._last_update_s = now
            self._last_command_s = now

    def forward(self, speed_m_s: float, *, now_s: float | None = None) -> None:
        self.command(Twist2D(float(speed_m_s), 0.0), now_s=now_s)

    def rotate(self, omega_rad_s: float, *, now_s: float | None = None) -> None:
        self.command(Twist2D(0.0, float(omega_rad_s)), now_s=now_s)

    def check_watchdog(self, *, now_s: float | None = None) -> bool:
        """Run one deterministic watchdog check; return whether it stopped motion."""

        now = _finite("now_s", self._clock() if now_s is None else now_s)
        with self._lock:
            if not self._started or self._last_command_s is None:
                return False
            if now < self._last_command_s:
                raise ValueError("now_s must not move backwards")
            if now - self._last_command_s < self.command_timeout_s:
                return False
            self._stop_locked()
            self._last_update_s = now
            self._watchdog_stop_count += 1
            return True

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._stop_locked()

    def close(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._watchdog_stop.set()
            watchdog_thread = self._watchdog_thread
        if watchdog_thread is not None and watchdog_thread is not threading.current_thread():
            watchdog_thread.join(timeout=max(0.1, self.watchdog_period_s * 4.0))
        with self._lock:
            try:
                self.backend.stop()
            finally:
                try:
                    self.backend.close()
                finally:
                    self._reset_motion_state(None)
                    self._started = False
                    self._watchdog_thread = None
                    self._hardware_lock.release()

    def _stop_locked(self) -> None:
        try:
            self.backend.stop()
        finally:
            self._reset_motion_state(self._last_update_s)

    def _reset_motion_state(self, timestamp_s: float | None) -> None:
        self._last_twist = Twist2D(0.0, 0.0)
        self._last_update_s = timestamp_s
        self._last_command_s = None

    def _watchdog_loop(self) -> None:
        while not self._watchdog_stop.wait(self.watchdog_period_s):
            try:
                self.check_watchdog()
            except BaseException:
                # Watchdog errors never leave its worker silently dead; the
                # lower-level RearMotorDriver has its own timeout as backup.
                continue

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("DifferentialDrive is not running")
