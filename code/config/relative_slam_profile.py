"""Explicit, in-memory relative SLAM profile for accepted task localization."""

from __future__ import annotations

from dataclasses import replace

from .v2_models import DifferentialRobotConfig, LocalizationConfig, SlamLocalizationConfig


def accepted_relative_slam_profile(config: DifferentialRobotConfig) -> DifferentialRobotConfig:
    """Select the accepted 2/3 T265 + D500/SLAM path for relative task actions.

    This changes localization settings only. Drive and firmware readiness checks
    remain governed by the board profile.
    """
    if not config.t265.enabled or not config.d500.enabled:
        raise ValueError("relative SLAM task requires enabled T265 and D500")
    return replace(
        config,
        fusion=replace(config.fusion, t265_min_tracker_confidence=2),
        d500_localization=replace(
            config.d500_localization,
            enable_wall_absolute=False,
            require_global_for_hardware=False,
        ),
        localization=LocalizationConfig(
            backend="slam_toolbox",
            slam=SlamLocalizationConfig(
                enabled=True,
                require_field_anchor=False,
                hardware_mission_validated=True,
                relative_goals_only=True,
            ),
        ),
    )
