"""Shared health states for sensors and runtime components."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class HealthState(Enum):
    OK = "ok"
    DEGRADED = "degraded"
    LOST = "lost"


@dataclass(frozen=True, slots=True)
class SensorHealth:
    """Health snapshot using monotonic seconds for its update timestamp."""

    component: str
    state: HealthState
    updated_at_s: float
    message: str = ""
