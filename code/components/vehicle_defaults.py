"""Protocol defaults shared with the validated C10B low-level sender.

These are installed firmware constraints, not chassis geometry. The differential
composition root supplies measured geometry and explicit protocol settings.
"""

from __future__ import annotations

from typing import Final

# C10B firmware-compiled track used by the serial protocol:
# Vz = (right - left) / firmware_track.  Deliberately different from the
# physical track width; the two must never be merged.
DEFAULT_FIRMWARE_TRACK_WIDTH_MM: Final[float] = 164.0
DEFAULT_MIN_TURN_RADIUS_MM: Final[float] = 350.0
