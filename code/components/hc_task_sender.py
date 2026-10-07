"""One task write through the existing HC-15 driver, with no ACK or retry."""

from __future__ import annotations

import logging

from .serial_communication import HC15SerialDriver

LOG = logging.getLogger(__name__)
DEFAULT_TEMPLATE = "TASK,{red},{blue},{green}\n"


def build_task_message(counts, template=DEFAULT_TEMPLATE) -> bytes:
    return template.format(red=counts.red, blue=counts.blue,
                           green=counts.green).encode("ascii")


def send_task_once(counts, *, port="/dev/ttyS4", baudrate=115200,
                   bridge_envelope=False, connect_wait_s=2.0,
                   template=DEFAULT_TEMPLATE, driver_factory=None) -> bool:
    driver = None
    ok = False
    try:
        message = build_task_message(counts, template)
        factory = HC15SerialDriver if driver_factory is None else driver_factory
        driver = factory(on_bytes=lambda _data: None, port=port,
                         baudrate=baudrate, bridge_envelope=bridge_envelope)
        driver.start()
        if driver.wait_connected(connect_wait_s):
            driver.write(message)  # Exactly one application-level write.
            ok = True
            LOG.info("HC task sent once: %r", message)
        else:
            LOG.error("HC connection timed out; task was not sent")
    except Exception:
        LOG.exception("HC one-shot task send failed")
    finally:
        if driver is not None:
            try:
                driver.close()
            except Exception:
                ok = False
                LOG.exception("HC close failed")
    return ok
