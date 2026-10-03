"""Read complete D500 scans for the relative SLAM localization path."""

from __future__ import annotations

import time

from .radar_driver import D500SerialDriver, RadarScanAssembler


class ScanOnlyD500Source:
    """Feed verified, assembled scans to SLAM without scan-to-scan ICP."""

    def __init__(self, runtime, port: str, baudrate: int, logger=None) -> None:
        self.runtime = runtime
        self.logger = logger
        self.assembler = RadarScanAssembler()
        self.serial = D500SerialDriver(
            port=port, baudrate=baudrate, on_packet=self._on_packet,
        )

    def _on_packet(self, packet) -> None:
        bridge = self.runtime.slam_bridge
        if bridge is None:
            return
        for scan in self.assembler.feed(packet):
            received_s = time.monotonic()
            measurement_s = self.runtime._map_d500_timestamp(scan.timestamp_ms, received_s)
            bridge.push_d500_scan(scan, measurement_s)
            if self.logger is not None:
                self.logger.emit({
                    "type": "accuracy_d500_scan", "monotonic_s": measurement_s,
                    "point_count": len(scan.points),
                    "rotation_speed_deg_s": scan.rotation_speed_deg_s,
                })

    def start(self) -> "ScanOnlyD500Source":
        self.assembler.reset()
        self.serial.start()
        return self

    def close(self) -> None:
        self.serial.close()
