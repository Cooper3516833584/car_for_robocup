"""Shared, hardware-free speed policy for mission entry points."""

from dataclasses import replace
import math

WALL_CRUISE_SPEED_M_S = 0.40


def route_speed_config(config, scale, *, wall_mission=False):
    """Scale motion targets; geometry, acceleration and preview policy stay fixed."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("--speed-scale must be finite and positive")
    drive = config.drive
    if wall_mission:
        # Operator-requested wall profile: permit 40 cm/s through both chassis
        # and wheel limiting even with the board's older 20/30 cm/s TOML.
        drive = replace(drive,
                        max_linear_speed_m_s=max(drive.max_linear_speed_m_s, WALL_CRUISE_SPEED_M_S),
                        max_wheel_speed_m_s=max(drive.max_wheel_speed_m_s, WALL_CRUISE_SPEED_M_S))
    return replace(config,
                   drive=replace(drive,
                       max_linear_speed_m_s=drive.max_linear_speed_m_s * scale,
                       max_angular_speed_rad_s=drive.max_angular_speed_rad_s * scale,
                       max_wheel_speed_m_s=drive.max_wheel_speed_m_s * scale),
                   navigation=replace(config.navigation,
                       translation_speed_scale=config.navigation.translation_speed_scale * scale,
                       final_yaw_gain=config.navigation.final_yaw_gain * scale))
