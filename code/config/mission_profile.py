"""Shared, hardware-free speed and payload policies for mission entry points."""

from dataclasses import replace
import math

from .v2_factory import configure_payload_relay


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


def payload_config(config, *, action, release_mode="simulate", relay_port=None, relay_channels=None):
    """Resolve payload output before camera, PWM, localization or drive opens."""
    if action == "beep":
        if config.relay.enabled or relay_port is not None or relay_channels is not None:
            raise ValueError("beep mode requires the payload relay to be disabled")
        return config
    relay = config.relay
    if release_mode == "simulate":
        if relay_port is not None:
            raise ValueError("--relay-port requires explicit --release-mode relay")
        return replace(config, relay=replace(relay, enabled=False, channel_count=4,
                                             disconnect_on_shutdown=True))
    if release_mode != "relay":
        raise ValueError("release mode must be simulate or relay")
    if relay_port is not None:
        if not relay_port.strip():
            raise ValueError("--relay-port must not be empty")
        relay = replace(relay, enabled=True, port=relay_port)
    if relay_channels is not None:
        relay = replace(relay, channel_count=relay_channels)
    if not relay.enabled:
        raise ValueError("drop mode requires an enabled relay in config or explicit --relay-port")
    if relay.channel_count < 4:
        raise ValueError("drop mode requires relay channels 2, 3 and 4")
    return configure_payload_relay(replace(config, relay=relay))
