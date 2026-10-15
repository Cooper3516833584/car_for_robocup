"""Shared, hardware-free speed policy for mission entry points."""

from dataclasses import replace
import math


def route_speed_config(config, scale):
    """Scale motion targets; geometry, acceleration and preview policy stay fixed."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("--speed-scale must be finite and positive")
    return replace(config,
                   drive=replace(config.drive,
                       max_linear_speed_m_s=config.drive.max_linear_speed_m_s * scale,
                       max_angular_speed_rad_s=config.drive.max_angular_speed_rad_s * scale,
                       max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s * scale),
                   navigation=replace(config.navigation,
                       translation_speed_scale=config.navigation.translation_speed_scale * scale,
                       final_yaw_gain=config.navigation.final_yaw_gain * scale))
