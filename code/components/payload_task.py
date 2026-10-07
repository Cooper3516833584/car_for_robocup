"""Electromagnet holding/release; the caller owns the relay lifecycle."""

from __future__ import annotations

import logging
import math
import time
from types import MappingProxyType

LOG = logging.getLogger(__name__)

# 操作者于 2026-10-07 确认：CH1 无电磁铁，CH2 右前、CH3 中间、CH4 左前。
# 三个逻辑投放选项不等于继电器路号；主程序和路线测试共用此映射。
PAYLOAD_SLOT_TO_RELAY = MappingProxyType({1: 2, 2: 3, 3: 4})
PAYLOAD_ACTIVE_ON = False  # 通电吸住，断电释放；释放后不重新吸合。


def payload_channel(slot, slot_to_relay=None) -> int:
    if isinstance(slot, bool) or not isinstance(slot, int) or slot not in (1, 2, 3):
        raise ValueError("payload slot must be 1, 2 or 3")
    mapping = PAYLOAD_SLOT_TO_RELAY if slot_to_relay is None else slot_to_relay
    channel = mapping[slot]
    if isinstance(channel, bool) or not isinstance(channel, int) or not 1 <= channel <= 8:
        raise ValueError("payload relay channel must be an integer in [1, 8]")
    return channel


def prepare_payload(relay, slot=1, *, slot_to_relay=None, active_on=PAYLOAD_ACTIVE_ON,
                    verify=True) -> bool:
    """Hold the selected payload before patrol; never energize unused CH1."""
    if relay is None:
        return False
    channel = None
    try:
        channel = payload_channel(slot, slot_to_relay)
        hold = relay.turn_off if active_on else relay.turn_on
        if hold(channel, verify=verify) is False:
            raise RuntimeError("payload holding state was not confirmed")
        return True
    except Exception:
        LOG.exception("could not prepare payload slot %s", slot)
        if channel is not None:
            try:
                relay.turn_off(channel, verify=verify)
            except Exception:
                LOG.exception("payload preparation recovery failed")
        return False


def drop_payload(relay, slot=1, *, slot_to_relay=None, active_on=PAYLOAD_ACTIVE_ON,
                 hold_s=0.5, verify=False, sleep=time.sleep) -> bool:
    """Release the selected magnet and leave it OFF, including on interruption.

    The default disconnects an energized holding magnet. Explicit active_on=True
    retains pulse support for other payload hardware, ending OFF as well.
    """
    if relay is None:
        LOG.error("payload relay unavailable or invalid slot: %r", slot)
        return False
    channel = None
    ok = False
    try:
        if not math.isfinite(hold_s) or hold_s < 0:
            raise ValueError("hold_s must be finite and non-negative")
        channel = payload_channel(slot, slot_to_relay)
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
            inactive = relay.turn_off
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
