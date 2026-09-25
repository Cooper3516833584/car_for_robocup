"""Runtime readiness and constrained hardware-probe policies for v2 configs."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .v2_models import DifferentialRobotConfig


class RuntimeMode(Enum):
    DRY_RUN = "dry-run"
    REPLAY = "replay"
    HARDWARE_PROBE = "hardware-probe"
    HARDWARE_MISSION = "hardware-mission"


@dataclass(frozen=True, slots=True)
class RuntimeConstraints:
    autonomous_navigation_allowed: bool
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_wheel_speed_m_s: float


def validate_runtime_readiness(config: DifferentialRobotConfig, mode: RuntimeMode) -> list[str]:
    """Return blocking reasons for the requested runtime mode."""

    errors: list[str] = []
    if mode is not RuntimeMode.HARDWARE_MISSION:
        return errors
    if config.safety.require_measured_geometry_for_hardware_mission and not config.calibration.geometry_measured:
        errors.append("hardware mission requires measured drive geometry")
    if config.safety.require_measured_extrinsics_for_hardware_mission and not config.calibration.sensor_extrinsics_measured:
        errors.append("hardware mission requires measured sensor extrinsics")
    if config.safety.require_measured_map_for_hardware_mission and not config.competition_map.measured:
        errors.append("hardware mission requires a measured competition map")
    if config.safety.require_measured_footprint_for_hardware_mission and not config.footprint.measured:
        errors.append("hardware mission requires a measured robot footprint")
    if (
        config.safety.require_verified_c10b_diff_firmware_for_curved_motion
        and config.drive.protocol_mode == "differential_vx_vz"
        and not config.calibration.c10b_diff_firmware_verified
    ):
        errors.append(
            "differential_vx_vz requires verified C10B differential firmware; "
            "select ackermann_firmware_compat for the compatibility backend"
        )
    return errors


def runtime_constraints(config: DifferentialRobotConfig, mode: RuntimeMode) -> RuntimeConstraints:
    """Return mode-specific limits; hardware probes are low-speed only."""

    if mode is RuntimeMode.HARDWARE_PROBE:
        linear = min(config.drive.max_linear_speed_m_s, config.safety.hardware_probe_max_linear_speed_m_s)
        angular = min(config.drive.max_angular_speed_rad_s, config.safety.hardware_probe_max_angular_speed_rad_s)
        return RuntimeConstraints(False, linear, angular, min(config.drive.max_wheel_speed_m_s, linear))
    return RuntimeConstraints(
        mode is not RuntimeMode.HARDWARE_MISSION or not validate_runtime_readiness(config, mode),
        config.drive.max_linear_speed_m_s,
        config.drive.max_angular_speed_rad_s,
        config.drive.max_wheel_speed_m_s,
    )
