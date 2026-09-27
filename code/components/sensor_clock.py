"""Small fixed-offset mappings from device clocks to host monotonic time."""

from __future__ import annotations

import math


class DeviceClockMapper:
    """Map device milliseconds into the host monotonic clock domain.

    The offset is captured once and held steady, avoiding callback jitter being
    copied into every measurement timestamp. An optional modulus handles
    wrapping device counters such as the D500's uint16 millisecond clock.
    """

    def __init__(self, *, modulus_ms: int | None = None) -> None:
        if modulus_ms is not None and modulus_ms <= 1:
            raise ValueError("modulus_ms must be greater than one")
        self.modulus_ms = modulus_ms
        self.reset()

    def reset(self) -> None:
        self._offset_s: float | None = None
        self._last_device_ms: float | None = None
        self._unwrapped_device_ms: float | None = None
        self._last_measurement_s: float | None = None

    def map_milliseconds(self, device_timestamp_ms: float | None, received_monotonic_s: float) -> float:
        received = float(received_monotonic_s)
        if not math.isfinite(received):
            raise ValueError("received_monotonic_s must be finite")
        try:
            device_ms = float(device_timestamp_ms)
        except (TypeError, ValueError):
            return received
        if not math.isfinite(device_ms) or device_ms < 0.0:
            return received

        reset_clock = self._offset_s is None
        delta_ms = 0.0
        if self._last_device_ms is not None:
            delta_ms = device_ms - self._last_device_ms
            if self.modulus_ms is None:
                reset_clock = reset_clock or delta_ms < -1.0
            elif delta_ms < -self.modulus_ms / 2.0:
                delta_ms += self.modulus_ms
            elif delta_ms < 0.0 or delta_ms > self.modulus_ms / 2.0:
                reset_clock = True

        if reset_clock:
            self._unwrapped_device_ms = device_ms
            self._offset_s = received - device_ms * 1e-3
            measurement_s = received
        else:
            assert self._unwrapped_device_ms is not None
            self._unwrapped_device_ms += delta_ms
            measurement_s = self._unwrapped_device_ms * 1e-3 + self._offset_s
            if self._last_measurement_s is not None and measurement_s < self._last_measurement_s:
                # Preserve monotonic output across sub-millisecond jitter.
                self._offset_s += self._last_measurement_s - measurement_s
                measurement_s = self._last_measurement_s

        self._last_device_ms = device_ms
        self._last_measurement_s = measurement_s
        return measurement_s
