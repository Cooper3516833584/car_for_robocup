"""Shared, hardware-lazy CPU inference settings measured on the ROCK 5A."""

import logging
import os
from pathlib import Path

IMGSZ = 320
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30


def select_fastest_cpus(policies, allowed):
    available = [(frequency, set(cpus) & set(allowed)) for frequency, cpus in policies]
    available = [(frequency, cpus) for frequency, cpus in available if cpus]
    if not available:
        return set()
    fastest = max(frequency for frequency, _ in available)
    return set().union(*(cpus for frequency, cpus in available if frequency == fastest))


def pin_vision_worker():
    """Pin only the calling Linux thread, respecting its allowed CPU set."""
    try:
        policies = []
        for policy in Path("/sys/devices/system/cpu/cpufreq").glob("policy*"):
            policies.append((int((policy / "cpuinfo_max_freq").read_text()),
                             {int(cpu) for cpu in (policy / "related_cpus").read_text().split()}))
        cpus = select_fastest_cpus(policies, os.sched_getaffinity(0))
        if cpus:
            os.sched_setaffinity(0, cpus)
        return sorted(cpus)
    except (OSError, ValueError, AttributeError):
        return []


class CpuDetector:
    """Restore caller affinity; reapply one thread after Ultralytics setup.

    First inference includes cold backend setup and must happen while stopped.
    Torch thread count is process-wide; affinity is scoped to this native thread.
    """

    def __init__(self, detector):
        self._detector = detector

    @property
    def names(self):
        return self._detector.names

    def predict(self, *args, **kwargs):
        import torch

        try:
            original = os.sched_getaffinity(0)
        except (OSError, AttributeError):
            original = None
        try:
            pin_vision_worker()
            torch.set_num_threads(1)
            kwargs.setdefault("device", "cpu")
            kwargs.setdefault("imgsz", IMGSZ)
            return self._detector.predict(*args, **kwargs)
        finally:
            # Backend initialization overrides this to seven on the deployed CPU.
            # Apply even on exceptions, and restore control-thread affinity.
            try:
                torch.set_num_threads(1)
            finally:
                if original is not None:
                    try:
                        os.sched_setaffinity(0, original)
                    except OSError:
                        logging.getLogger(__name__).exception("cannot restore CPU affinity")
