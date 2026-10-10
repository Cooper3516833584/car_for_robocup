"""Validated schema-v2 configuration models for the differential platform."""

from __future__ import annotations

from dataclasses import dataclass
import math


class ConfigV2Error(ValueError):
    """Invalid or unsupported differential-drive schema-v2 configuration."""


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ConfigV2Error(f"{name} must be finite")
    return result


def _positive(name: str, value: float) -> float:
    result = _finite(name, value)
    if result <= 0.0:
        raise ConfigV2Error(f"{name} must be greater than zero")
    return result


@dataclass(frozen=True, slots=True)
class CalibrationStatusConfig:
    c10b_diff_firmware_verified: bool


@dataclass(frozen=True, slots=True)
class DifferentialGeometryConfig:
    drive_track_width_m: float
    wheel_diameter_m: float
    body_length_m: float
    body_width_m: float
    drive_axle_to_body_center_x_m: float
    front_support_x_m: float
    rear_support_x_m: float

    def __post_init__(self) -> None:
        for name in ("drive_track_width_m", "wheel_diameter_m", "body_length_m", "body_width_m"):
            _positive(name, getattr(self, name))
        for name in ("drive_axle_to_body_center_x_m", "front_support_x_m", "rear_support_x_m"):
            _finite(name, getattr(self, name))


@dataclass(frozen=True, slots=True)
class DifferentialDriveConfig:
    kinematics: str
    protocol_mode: str
    max_wheel_speed_m_s: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_linear_accel_m_s2: float
    max_angular_accel_rad_s2: float
    command_timeout_s: float
    firmware_track_width_m: float
    firmware_min_turn_radius_m: float
    allow_in_place_rotation: bool

    def __post_init__(self) -> None:
        if self.kinematics != "differential":
            raise ConfigV2Error("vehicle.drive.kinematics must be 'differential'")
        if self.protocol_mode not in {"ackermann_firmware_compat", "differential_vx_vz"}:
            raise ConfigV2Error(
                "vehicle.drive.protocol_mode must be 'ackermann_firmware_compat' "
                "or 'differential_vx_vz'"
            )
        for name in (
            "max_wheel_speed_m_s", "max_linear_speed_m_s", "max_angular_speed_rad_s",
            "max_linear_accel_m_s2", "max_angular_accel_rad_s2", "command_timeout_s",
            "firmware_track_width_m", "firmware_min_turn_radius_m",
        ):
            _positive(name, getattr(self, name))


@dataclass(frozen=True, slots=True)
class SensorMount3DConfig:
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"):
            _finite(name, getattr(self, name))


@dataclass(frozen=True, slots=True)
class C10BConfig:
    port: str
    baudrate: int

    def __post_init__(self) -> None:
        if not self.port.strip():
            raise ConfigV2Error("devices.c10b.port must not be empty")
        if self.baudrate <= 0:
            raise ConfigV2Error("devices.c10b.baudrate must be positive")


@dataclass(frozen=True, slots=True)
class D500Config:
    enabled: bool
    port: str
    baudrate: int

    def __post_init__(self) -> None:
        if self.enabled and not self.port.strip():
            raise ConfigV2Error("devices.d500.port must not be empty when enabled")
        if self.baudrate <= 0:
            raise ConfigV2Error("devices.d500.baudrate must be positive")


@dataclass(frozen=True, slots=True)
class RelayConfig:
    """Optional LCUS USB relay (payload switch); disabled unless configured.

    Channel bounds mirror the LCUS protocol in ``components/relay_lcus.py``: one
    board drives 1..8 relay channels and always runs at 9600 8N1, so only the
    port and the installed board's channel count are deployment choices. The
    default is 4 because the board fitted to this robot is a 4-channel LCUS
    board; set 8 explicitly for an 8-channel board.
    """

    enabled: bool = False
    port: str = ""
    baudrate: int = 9600
    channel_count: int = 4
    read_timeout_s: float = 0.2
    query_timeout_s: float = 1.0
    verify_writes: bool = True
    disconnect_on_shutdown: bool = True

    def __post_init__(self) -> None:
        if self.enabled and not self.port.strip():
            raise ConfigV2Error("devices.relay.port must not be empty when enabled")
        if self.baudrate <= 0:
            raise ConfigV2Error("devices.relay.baudrate must be positive")
        if isinstance(self.channel_count, bool) or not isinstance(self.channel_count, int):
            raise ConfigV2Error("devices.relay.channel_count must be an integer")
        if not 1 <= self.channel_count <= 8:
            raise ConfigV2Error("devices.relay.channel_count must be in [1, 8]")
        _positive("devices.relay.read_timeout_s", self.read_timeout_s)
        _positive("devices.relay.query_timeout_s", self.query_timeout_s)


@dataclass(frozen=True, slots=True)
class ServoConfig:
    """Optional single-axis PWM hobby servo (payload pan/tilt axis).

    Disabled unless a profile opts in, so every profile written before the servo
    existed keeps loading unchanged. ``chip_device_match`` holds one or more
    selectors for :func:`hal.pwm.resolve_chip`; the default names the ROCK 5A
    ``PWM7_IR_M0`` device-tree node (physical Pin 28) rather than a
    probe-order-dependent ``pwmchipN``. ``0`` degrees is the mid pulse, matching
    ``components/servo_axis.py``.
    """

    enabled: bool = False
    # Only the device-tree node name is trusted by default. A ``pwmchipN``
    # fallback is deliberately absent: probe order and truncated sysfs class
    # names make it easy to bind an unrelated PWM controller that happens to
    # share the number, which would drive the wrong pin instead of failing.
    chip_device_match: tuple[str, ...] = ("febd0030.pwm",)
    channel: int = 0
    pwm_path: str = ""
    period_us: int = 20_000
    pulse_min_us: int = 500
    pulse_max_us: int = 2500
    travel_half_range_deg: float = 90.0
    home_angle_deg: float = 0.0
    settle_s: float = 0.35
    min_command_interval_s: float = 0.02

    def __post_init__(self) -> None:
        if isinstance(self.chip_device_match, str):
            selectors = (self.chip_device_match,)
        else:
            selectors = tuple(self.chip_device_match)
        if not all(isinstance(item, str) and item.strip() for item in selectors):
            raise ConfigV2Error("devices.servo.chip_device_match must hold non-empty strings")
        object.__setattr__(self, "chip_device_match", selectors)
        if isinstance(self.channel, bool) or not isinstance(self.channel, int) or self.channel < 0:
            raise ConfigV2Error("devices.servo.channel must be a non-negative integer")
        if not self.pwm_path.strip() and not selectors:
            raise ConfigV2Error(
                "devices.servo needs either pwm_path or chip_device_match when enabled"
            )
        if isinstance(self.period_us, bool) or not isinstance(self.period_us, int):
            raise ConfigV2Error("devices.servo.period_us must be an integer")
        for name in ("pulse_min_us", "pulse_max_us"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ConfigV2Error(f"devices.servo.{name} must be an integer number of microseconds")
        if not 0 <= self.pulse_min_us < self.pulse_max_us < self.period_us:
            raise ConfigV2Error(
                "devices.servo pulses must satisfy 0 <= pulse_min_us < pulse_max_us < period_us"
            )
        half_range = _positive("devices.servo.travel_half_range_deg", self.travel_half_range_deg)
        home = _finite("devices.servo.home_angle_deg", self.home_angle_deg)
        if not -half_range <= home <= half_range:
            raise ConfigV2Error(
                "devices.servo.home_angle_deg must lie inside travel_half_range_deg"
            )
        _positive("devices.servo.settle_s", self.settle_s)
        for name in ("min_command_interval_s",):
            value = _finite(f"devices.servo.{name}", getattr(self, name))
            if value < 0.0:
                raise ConfigV2Error(f"devices.servo.{name} must not be negative")


@dataclass(frozen=True, slots=True)
class D500LocalizationConfig:
    enable_icp: bool
    enable_wall_absolute: bool
    require_global_for_hardware: bool
    reference_measured: bool
    field_width_m: float
    field_height_m: float
    back_wall_x_m: float = 0.0
    right_wall_y_m: float = 0.0
    use_front_wall: bool = False
    use_left_wall: bool = False
    min_confidence: float = 0.65
    max_position_jump_m: float = 0.50
    max_yaw_jump_rad: float = 0.35

    def __post_init__(self) -> None:
        if not self.enable_icp:
            raise ConfigV2Error("sensors.d500.localization.enable_icp must remain true for D500 pose output")
        _positive("sensors.d500.localization.field_width_m", self.field_width_m)
        _positive("sensors.d500.localization.field_height_m", self.field_height_m)
        _finite("sensors.d500.localization.back_wall_x_m", self.back_wall_x_m)
        _finite("sensors.d500.localization.right_wall_y_m", self.right_wall_y_m)
        confidence = _finite("sensors.d500.localization.min_confidence", self.min_confidence)
        if not 0.0 <= confidence <= 1.0:
            raise ConfigV2Error("sensors.d500.localization.min_confidence must be in [0, 1]")
        _positive("sensors.d500.localization.max_position_jump_m", self.max_position_jump_m)
        _positive("sensors.d500.localization.max_yaw_jump_rad", self.max_yaw_jump_rad)


@dataclass(frozen=True, slots=True)
class T265Config:
    enabled: bool
    serial: str


@dataclass(frozen=True, slots=True)
class FusionConfig:
    t265_max_age_s: float
    d500_max_age_s: float
    t265_min_tracker_confidence: int

    def __post_init__(self) -> None:
        for name in ("t265_max_age_s", "d500_max_age_s"):
            _positive(name, getattr(self, name))
        if not 0 <= self.t265_min_tracker_confidence <= 3:
            raise ConfigV2Error("fusion.t265_min_tracker_confidence must be in [0, 3]")


@dataclass(frozen=True, slots=True)
class SlamLocalizationConfig:
    enabled: bool = False
    require_field_anchor: bool = True
    hardware_mission_validated: bool = False
    relative_goals_only: bool = False


@dataclass(frozen=True, slots=True)
class LocalizationConfig:
    backend: str = "legacy"
    slam: SlamLocalizationConfig = SlamLocalizationConfig()

    def __post_init__(self) -> None:
        if self.backend not in {"legacy", "slam_toolbox"}:
            raise ConfigV2Error("localization.backend must be legacy or slam_toolbox")
        if self.backend == "slam_toolbox" and not self.slam.enabled:
            raise ConfigV2Error("slam_toolbox backend requires localization.slam.enabled=true")
        if self.slam.relative_goals_only and (self.backend != "slam_toolbox" or self.slam.require_field_anchor):
            raise ConfigV2Error("relative_goals_only requires relative slam_toolbox localization")


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    position_tolerance_m: float
    yaw_tolerance_rad: float
    lookahead_m: float
    rotate_in_place_threshold_rad: float
    slowdown_distance_m: float
    path_yaw_gain: float = 1.5
    final_yaw_gain: float = 1.5
    degraded_speed_scale: float = 0.4
    translation_speed_scale: float = 1.0

    def __post_init__(self) -> None:
        for name in (
            "position_tolerance_m", "yaw_tolerance_rad", "lookahead_m",
            "rotate_in_place_threshold_rad", "slowdown_distance_m",
            "path_yaw_gain", "final_yaw_gain", "translation_speed_scale",
        ):
            _positive(name, getattr(self, name))
        scale = _finite("degraded_speed_scale", self.degraded_speed_scale)
        if not 0.0 < scale <= 1.0:
            raise ConfigV2Error("navigation.degraded_speed_scale must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    require_verified_c10b_diff_firmware_for_curved_motion: bool
    stop_if_pose_lost_s: float
    hardware_probe_max_linear_speed_m_s: float = 0.10
    hardware_probe_max_angular_speed_rad_s: float = 0.35
    require_measured_d500_reference_for_hardware_mission: bool = True

    def __post_init__(self) -> None:
        _positive("safety.stop_if_pose_lost_s", self.stop_if_pose_lost_s)
        _positive("safety.hardware_probe_max_linear_speed_m_s", self.hardware_probe_max_linear_speed_m_s)
        _positive("safety.hardware_probe_max_angular_speed_rad_s", self.hardware_probe_max_angular_speed_rad_s)


@dataclass(frozen=True, slots=True)
class DifferentialRobotConfig:
    schema_version: int
    robot_name: str
    calibration: CalibrationStatusConfig
    geometry: DifferentialGeometryConfig
    drive: DifferentialDriveConfig
    c10b: C10BConfig
    d500: D500Config
    d500_localization: D500LocalizationConfig
    d500_mount: SensorMount3DConfig
    t265: T265Config
    t265_mount: SensorMount3DConfig
    fusion: FusionConfig
    navigation: NavigationConfig
    safety: SafetyConfig
    relay: RelayConfig = RelayConfig()
    servo: ServoConfig = ServoConfig()
    localization: LocalizationConfig = LocalizationConfig()

    def __post_init__(self) -> None:
        if self.schema_version != 2:
            raise ConfigV2Error(
                f"unsupported schema_version {self.schema_version}; "
                "load legacy schema-v1 files with load_car_config()"
            )
        if not self.robot_name.strip():
            raise ConfigV2Error("robot_name must not be empty")
        if self.localization.slam.enabled and (not self.t265.enabled or not self.d500.enabled):
            raise ConfigV2Error("SLAM bridge requires enabled T265 and D500 sensors")
