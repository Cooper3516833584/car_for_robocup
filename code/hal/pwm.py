"""Hardware abstraction layer: PWM output.

Only the abstract :class:`PWMOutput` interface and the Linux sysfs
implementation live here.  A component never knows about a specific board's
device-tree names; every value (sysfs root, chip selector, channel, period,
polarity) comes from the TOML profile via this layer.
"""

from __future__ import annotations

from collections.abc import Sequence
import os
import re
import time
from pathlib import Path
from typing import Protocol, runtime_checkable


@runtime_checkable
class PWMOutput(Protocol):
    """Minimal PWM interface consumed by the PWM servo component."""

    def start(self) -> None: ...

    def set_pulse_us(self, pulse_us: int) -> None: ...

    def disable(self) -> None: ...

    def close(self) -> None: ...


_CHIP_NAME = re.compile(r"^pwmchip(\d+)$")
_NODE_NAME = re.compile(r"^[0-9a-fA-F]+\.[A-Za-z0-9_.-]+$")


def _resolved_parts(chip: Path) -> list[str]:
    """Path components of ``chip`` with its ``device`` symlink expanded by hand.

    See :func:`_device_node_name` for why :meth:`Path.resolve` is not used: on
    the board's Python 3.11 it does not reliably traverse a symlink nested under
    a path whose last component is already a symlink.
    """

    device = chip / "device"
    try:
        target = os.readlink(device)
    except OSError:
        combined = str(chip)
    else:
        combined = str(Path(target) if os.path.isabs(target) else chip.parent / target)
    return [part for part in combined.replace("\\", "/").split("/") if part]


def resolve_chip(root: Path, match: str | Sequence[str]) -> Path:
    """Return the ``pwmchip*`` entry selected by ``match``.

    ``match`` accepts any of three stable selectors, tried in this order:

    ``pwmchipN``
        The literal sysfs name; the only selector that survives a device-tree
        address change, but it depends on probe order.
    ``NNNN.NNNN.pwm`` (for example ``febd0030.pwm``)
        A device-tree node name; resolved through the ``device`` symlink, so it
        is immune to probe order and to the truncated sysfs *class* names.
    any other string
        A substring of the resolved device path, for boards whose node names
        are not known in advance.

    Surviving probe-order changes matters here: two PWM controllers can expose
    nodes that collapse onto the same sysfs class name, so only the device-tree
    address identifies the pin unambiguously.
    """

    matches = (match,) if isinstance(match, str) else tuple(match)
    chips = sorted(root.glob("pwmchip*"), key=lambda chip: _chip_sort_key(chip.name))
    for candidate in matches:
        for chip in chips:
            if candidate == chip.name:
                return chip
        for chip in chips:
            if _NODE_NAME.match(candidate) and _resolved_parts(chip)[-1:] == [candidate]:
                return chip
    for candidate in matches:
        # Only a non-node selector may fall back to a substring search. A node
        # name that did not match exactly must stay unmatched: a loose substring
        # test would otherwise accept any controller whose device path merely
        # contains the name, which is how the servo pin could silently bind the
        # wrong PWM channel instead of reporting that the overlay is missing.
        if _NODE_NAME.match(candidate):
            continue
        for chip in chips:
            if any(candidate in part for part in _resolved_parts(chip)):
                return chip
    raise PWMBackendError(
        f"cannot find a PWM chip matching {matches!r} under {root}; the pin has no "
        "exported PWM channel - check that the device-tree/pinmux overlay for your "
        "board is enabled and that the board has rebooted since"
    )


def _chip_sort_key(name: str) -> tuple[int, str]:
    found = _CHIP_NAME.match(name)
    return (int(found.group(1)) if found else 1 << 30, name)


class PWMBackendError(RuntimeError):
    """The configured PWM output cannot be opened or controlled."""


def _write(path: Path, value: int | str) -> None:
    path.write_text(f"{value}\n", encoding="ascii")


class LinuxSysfsPWMOutput:
    """Linux sysfs ``/sys/class/pwm`` PWM output.

    The chip is selected by :func:`resolve_chip` from ``chip_device_match`` (a
    single selector or a tuple of fallbacks), so a caller names a pin's
    device-tree node rather than a probe-order-dependent ``pwmchipN``.
    ``pwm_chip`` overrides the search with an explicit ``/sys/class/pwm/pwmchipN``
    path, and ``pwm_path`` skips chip selection entirely for an already-known
    ``.../pwmchipN/pwmM`` channel directory.
    """

    def __init__(
        self,
        *,
        sysfs_root: str | Path = "/sys/class/pwm",
        chip_device_match: str | Sequence[str] = "fd8b0000.pwm",
        channel: int = 0,
        period_ns: int = 20_000_000,
        polarity: str = "normal",
        pwm_chip: str | Path | None = None,
        pwm_path: str | Path | None = None,
    ) -> None:
        if channel < 0:
            raise ValueError("pwm channel cannot be negative")
        if period_ns <= 0:
            raise ValueError("pwm period_ns must be positive")
        if polarity not in ("normal", "inversed"):
            raise ValueError("pwm polarity must be 'normal' or 'inversed'")
        self._root = Path(sysfs_root)
        self._chip_match: tuple[str, ...] = (
            (chip_device_match,) if isinstance(chip_device_match, str)
            else tuple(chip_device_match)
        )
        if not self._chip_match or not all(isinstance(item, str) and item for item in self._chip_match):
            raise ValueError("chip_device_match must name at least one non-empty selector")
        self._channel = channel
        self._period_ns = period_ns
        self._polarity = polarity
        self._configured_chip = Path(pwm_chip) if pwm_chip is not None else None
        self._explicit_pwm = Path(pwm_path) if pwm_path is not None else None
        self._pwm: Path | None = None

    @property
    def is_running(self) -> bool:
        return self._pwm is not None

    def _resolve_chip(self) -> Path:
        if self._configured_chip is not None:
            return self._configured_chip
        return resolve_chip(self._root, self._chip_match)

    def start(self) -> "LinuxSysfsPWMOutput":
        if self._pwm is not None:
            raise PWMBackendError("PWM output is already running")
        if self._explicit_pwm is not None:
            pwm = self._explicit_pwm
            chip = pwm.parent
        else:
            chip = self._resolve_chip()
            pwm = chip / f"pwm{self._channel}"
        if not pwm.exists():
            _write(chip / "export", self._channel)
            for _ in range(20):
                if pwm.exists():
                    break
                time.sleep(0.05)
        if not pwm.exists():
            raise PWMBackendError(
                f"PWM export did not create {pwm}; check permissions"
            )
        enable = pwm / "enable"
        try:
            if enable.read_text(encoding="ascii").strip() == "1":
                _write(enable, 0)
            _write(pwm / "period", self._period_ns)
            _write(pwm / "polarity", self._polarity)
        except OSError as exc:
            raise PWMBackendError(f"cannot configure {pwm}: {exc}") from exc
        self._pwm = pwm
        return self

    def set_pulse_us(self, pulse_us: int) -> None:
        if self._pwm is None:
            raise PWMBackendError("PWM output is not running")
        pulse = int(pulse_us)
        if pulse < 0:
            raise ValueError("pulse_us cannot be negative")
        try:
            _write(self._pwm / "duty_cycle", pulse * 1000)
            _write(self._pwm / "enable", 1)
        except OSError as exc:
            raise PWMBackendError(
                f"cannot write PWM duty cycle for {self._pwm}: {exc}"
            ) from exc

    def disable(self) -> None:
        if self._pwm is None:
            raise PWMBackendError("PWM output is not running")
        try:
            _write(self._pwm / "enable", 0)
        except OSError as exc:
            raise PWMBackendError(
                f"cannot disable PWM output {self._pwm}: {exc}"
            ) from exc

    def close(self) -> None:
        self._pwm = None
