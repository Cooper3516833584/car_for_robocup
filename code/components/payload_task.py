"""A single LCUS release pulse; the runtime owns the relay lifecycle."""

from __future__ import annotations

import logging
import math
import time

LOG = logging.getLogger(__name__)


def drop_payload(relay, slot=1, *, slot_to_relay=None, active_on=True,
                 hold_s=0.5, verify=False, sleep=time.sleep) -> bool:
    """Release slot 1/2/3, then restore inactive even on interruption."""
    mapping = {1: 1, 2: 2, 3: 3} if slot_to_relay is None else slot_to_relay
    if relay is None or isinstance(slot, bool) or slot not in (1, 2, 3):
        LOG.error("payload relay unavailable or invalid slot: %r", slot)
        return False
    channel = None
    ok = False
    try:
        if not math.isfinite(hold_s) or hold_s < 0:
            raise ValueError("hold_s must be finite and non-negative")
        channel = mapping[slot]
        active = relay.turn_on if active_on else relay.turn_off
        if active(channel, verify=verify) is False:
            LOG.error("payload slot %s activation was not confirmed", slot)
        else:
            sleep(hold_s)
            ok = True
    except Exception:
        LOG.exception("payload slot %s release failed", slot)
    finally:
        if channel is not None:
            inactive = relay.turn_off if active_on else relay.turn_on
            try:
                if inactive(channel, verify=verify) is False:
                    ok = False
                    # One best-effort recovery, never a mission failure gate.
                    inactive(channel, verify=verify)
                    LOG.error("payload slot %s inactive state was not confirmed", slot)
            except Exception:
                ok = False
                LOG.exception("could not restore payload slot %s", slot)
                try:
                    inactive(channel, verify=verify)
                except Exception:
                    LOG.exception("payload recovery also failed")
    return ok
