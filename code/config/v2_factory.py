"""V2 component builders; hardware-specific imports stay inside the builder."""

from __future__ import annotations

from typing import Callable
from dataclasses import replace

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


def build_payload_demo_session(config: DifferentialRobotConfig, *, event_logger=None,
                               fake=False, clock=None):
    """Explicit filming entry; validate static hardware settings, open no sensors."""
    from .v2_runtime import RuntimeMode, validate_runtime_readiness
    from payload_demo import PayloadDemoSession

    errors = validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION)
    if errors and not fake:
        raise ValueError("; ".join(errors))
    if not config.drive.allow_in_place_rotation:
        raise ValueError("payload demo requires verified in-place rotation")
    kwargs = {} if clock is None else {"clock": clock}
    return PayloadDemoSession(build_differential_drive(config, fake=fake, clock=clock),
                              build_relay(config, fake=fake), event_logger=event_logger,
                              verify_relay=config.relay.verify_writes, **kwargs)


def configure_payload_relay(config: DifferentialRobotConfig, *, port: str | None = None):
    """Explicit CLI enablement and cleanup policy; never opens hardware."""
    relay = config.relay
    if port is not None:
        if not port.strip():
            raise ValueError("payload relay port must not be empty")
        relay = replace(relay, enabled=True, port=port)
    if not relay.enabled:
        return config
    if relay.channel_count < 4:
        raise ValueError("payload magnets require relay channels 2, 3 and 4")
    return replace(config, relay=replace(relay, verify_writes=True, disconnect_on_shutdown=True))


def build_relay(config: DifferentialRobotConfig, *, fake: bool = False):
    """Build the optional LCUS payload relay; construction never opens the port.

    Returns ``None`` when ``[devices.relay] enabled = false``. The real driver
    opens its serial port only when the runtime starts, and the dry-run/replay
    fake never touches a device.
    """

    relay_config = config.relay
    if not relay_config.enabled:
        return None
    if fake:
        from components.relay_lcus import FakeLCUSRelay

        return FakeLCUSRelay(channel_count=relay_config.channel_count)
    from components.relay_lcus import LCUSRelay

    return LCUSRelay(
        port=relay_config.port,
        baudrate=relay_config.baudrate,
        channel_count=relay_config.channel_count,
        timeout=relay_config.read_timeout_s,
        query_timeout=relay_config.query_timeout_s,
    )


def build_servo(config: DifferentialRobotConfig, *, fake: bool = False):
    """Build the optional single-axis PWM servo; construction never opens a device.

    Returns ``None`` when ``[devices.servo] enabled = false``. Device opening is
    deferred to :meth:`components.servo_axis.ServoAxis.start`, so building the
    runtime cannot move a physical axis. ``fake=True`` returns the in-memory axis
    used by dry-run and replay.
    """

    servo_config = getattr(config, "servo", None)
    if servo_config is None or not servo_config.enabled:
        return None

    from components.servo_axis import FakeServoAxis, ServoAxis, ServoTravel

    travel = ServoTravel(
        half_range_deg=servo_config.travel_half_range_deg,
        pulse_min_us=servo_config.pulse_min_us,
        pulse_max_us=servo_config.pulse_max_us,
        period_us=servo_config.period_us,
    )
    common = {
        "travel": travel,
        "settle_s": servo_config.settle_s,
        "min_command_interval_s": servo_config.min_command_interval_s,
        "home_angle_deg": servo_config.home_angle_deg,
    }
    if fake:
        return FakeServoAxis(**common)
    from hal.pwm import LinuxSysfsPWMOutput

    kwargs = {"pwm_path": servo_config.pwm_path} if servo_config.pwm_path.strip() else {}
    output = LinuxSysfsPWMOutput(
        chip_device_match=tuple(servo_config.chip_device_match),
        channel=servo_config.channel,
        period_ns=servo_config.period_us * 1000,
        **kwargs,
    )
    return ServoAxis(pwm=output, **common)


def build_pose_fusion(config: DifferentialRobotConfig):
    """Build the pure pose-fusion state object from validated v2 settings."""

    from components.pose_fusion import PoseFusion

    return PoseFusion(
        config.fusion,
        backend=config.localization.backend,
        require_field_anchor=config.localization.slam.require_field_anchor,
    )


def build_differential_navigator(config: DifferentialRobotConfig):
    """Build the direct relative-goal controller from validated v2 models."""

    from components.differential_navigation import DifferentialNavigator

    return DifferentialNavigator(config.drive, config.navigation, track_width_m=config.geometry.drive_track_width_m)


def build_basic_motion_controller(config: DifferentialRobotConfig, navigator):
    """Build the non-blocking task action layer without opening hardware."""

    from components.basic_motion_controller import BasicMotionController

    return BasicMotionController(navigator, config.navigation, config.drive,
                                 track_width_m=config.geometry.drive_track_width_m)
