"""V2 component builders; hardware-specific imports stay inside the builder."""

from __future__ import annotations

from typing import Callable

from .v2_models import DifferentialRobotConfig


def build_differential_drive(
    config: DifferentialRobotConfig,
    *,
    fake: bool = False,
    hardware_lock_path: str | None = "/run/lock/car-hardware.lock",
    clock: Callable[[], float] | None = None,
):
    """Build the SI drive facade; fake construction never opens a device.

    The caller must validate ``HARDWARE_MISSION`` readiness before starting a
    real backend. Device opening happens only when the returned drive starts.
    """

    from components.differential_drive import DifferentialDrive
    from components.differential_kinematics import DifferentialGeometry

    drive_config = config.drive
    kwargs = {} if clock is None else {"clock": clock}
    if fake:
        from components.c10b_diff_backend import FakeDriveBackend

        backend = FakeDriveBackend(max_wheel_speed_m_s=drive_config.max_wheel_speed_m_s)
        lock_path = None
    else:
        from components.c10b_diff_backend import C10BDifferentialBackend
        from components.rear_motor import RearMotorDriver

        rear_driver = RearMotorDriver(
            device=config.c10b.port,
            max_wheel_speed_mm_s=drive_config.max_wheel_speed_m_s * 1000.0,
            track_width_mm=drive_config.firmware_track_width_m * 1000.0,
            min_turn_radius_mm=drive_config.firmware_min_turn_radius_m * 1000.0,
            allow_in_place_rotation=drive_config.allow_in_place_rotation,
            command_timeout_s=drive_config.command_timeout_s,
        )
        backend = C10BDifferentialBackend(
            rear_driver,
            protocol_mode=drive_config.protocol_mode,
            firmware_track_width_m=drive_config.firmware_track_width_m,
            firmware_min_turn_radius_m=drive_config.firmware_min_turn_radius_m,
            allow_in_place_rotation=drive_config.allow_in_place_rotation,
            max_wheel_speed_m_s=drive_config.max_wheel_speed_m_s,
        )
        lock_path = hardware_lock_path

    return DifferentialDrive(
        backend,
        geometry=DifferentialGeometry(config.geometry.drive_track_width_m),
        max_wheel_speed_m_s=drive_config.max_wheel_speed_m_s,
        max_linear_speed_m_s=drive_config.max_linear_speed_m_s,
        max_angular_speed_rad_s=drive_config.max_angular_speed_rad_s,
        max_linear_accel_m_s2=drive_config.max_linear_accel_m_s2,
        max_angular_accel_rad_s2=drive_config.max_angular_accel_rad_s2,
        command_timeout_s=drive_config.command_timeout_s,
        hardware_lock_path=lock_path,
        **kwargs,
    )


def build_t265_source(config: DifferentialRobotConfig, *, fake: bool = False, samples=()):
    """Build an optional T265 source without importing the SDK for fake mode."""

    if not config.t265.enabled:
        return None
    if fake:
        from components.t265_driver import FakeT265PoseSource

        return FakeT265PoseSource(samples)
    from components.t265_driver import RealSenseT265PoseSource

    return RealSenseT265PoseSource(config.t265.serial)


def build_pose_fusion(config: DifferentialRobotConfig):
    """Build the pure pose-fusion state object from validated v2 settings."""

    from components.pose_fusion import PoseFusion

    return PoseFusion(config.fusion)


def build_differential_navigator(config: DifferentialRobotConfig):
    """Build the pure planner/controller from validated v2 models."""

    from components.differential_navigation import DifferentialNavigator

    return DifferentialNavigator(config.geometry, config.drive, config.navigation)


def build_competition_world(config: DifferentialRobotConfig):
    """Build the configured static competition map without hardware side effects."""

    from components.competition_map import CompetitionMapSpec, build_competition_navigation_grid

    map_config = config.competition_map
    spec = CompetitionMapSpec(
        width_m=map_config.width_m,
        height_m=map_config.height_m,
        resolution_m=map_config.resolution_m,
        origin_x_m=map_config.origin_x_m,
        origin_y_m=map_config.origin_y_m,
        static_obstacles=map_config.static_obstacles,
        allowed_regions=map_config.allowed_regions,
    )
    return build_competition_navigation_grid(
        spec,
        config.footprint.robot_radius_m,
        config.footprint.safety_margin_m,
    )
