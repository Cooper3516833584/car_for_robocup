"""Bounded, non-blocking JSONL event logging for the differential runtime."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any


def _json_value(value: Any) -> Any:
    if hasattr(value, "value"):
        return value.value
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class JsonlEventLogger:
    """Write JSON events on a worker thread; producers never wait for disk."""

    _SENTINEL = object()

    def __init__(self, path: str | Path, *, capacity: int = 4096) -> None:
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._high: queue.Queue[object] = queue.Queue(maxsize=max(8, capacity // 8))
        self._normal: queue.Queue[object] = queue.Queue(maxsize=capacity)
        self._closed = threading.Event()
        self._dropped_normal = 0
        self._dropped_high = 0
        self._write_error: str | None = None
        self._thread = threading.Thread(target=self._worker, name="robocup-jsonl-log", daemon=True)
        self._thread.start()

    @property
    def dropped_events(self) -> int:
        return self._dropped_normal + self._dropped_high

    @property
    def write_error(self) -> str | None:
        return self._write_error

    def emit(self, event: dict[str, Any], *, priority: bool = False) -> bool:
        if self._closed.is_set():
            return False
        item = _json_value(event)
        target = self._high if priority else self._normal
        try:
            target.put_nowait(item)
            return True
        except queue.Full:
            if priority:
                self._dropped_high += 1
            else:
                self._dropped_normal += 1
            return False

    def close(self, *, timeout_s: float = 2.0) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        deadline = time.monotonic() + max(0.0, timeout_s)
        while self._thread.is_alive() and time.monotonic() < deadline:
            try:
                self._high.put_nowait(self._SENTINEL)
                break
            except queue.Full:
                time.sleep(0.005)
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))

    def _worker(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("w", encoding="utf-8", newline="\n") as stream:
                while True:
                    item = None
                    try:
                        item = self._high.get_nowait()
                    except queue.Empty:
                        try:
                            item = self._normal.get(timeout=0.05)
                        except queue.Empty:
                            if self._closed.is_set():
                                if self._high.empty() and self._normal.empty():
                                    break
                            continue
                    if item is self._SENTINEL:
                        if self._high.empty() and self._normal.empty():
                            break
                        continue
                    stream.write(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n")
                    stream.flush()
        except Exception as exc:  # diagnostics must not disrupt actuation or cleanup
            self._write_error = f"{type(exc).__name__}: {exc}"
