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
    geometry_measured: bool
    sensor_extrinsics_measured: bool
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
class T265Config:
    enabled: bool
    serial: str


@dataclass(frozen=True, slots=True)
class FusionConfig:
    t265_max_age_s: float
    d500_max_age_s: float
    t265_min_tracker_confidence: int
    position_correction_gain: float
    yaw_correction_gain: float
    max_position_innovation_m: float
    max_yaw_innovation_rad: float
    max_single_position_correction_m: float
    max_single_yaw_correction_rad: float

    def __post_init__(self) -> None:
        for name in (
            "t265_max_age_s", "d500_max_age_s", "max_position_innovation_m",
            "max_yaw_innovation_rad", "max_single_position_correction_m",
            "max_single_yaw_correction_rad",
        ):
            _positive(name, getattr(self, name))
        if not 0 <= self.t265_min_tracker_confidence <= 3:
            raise ConfigV2Error("fusion.t265_min_tracker_confidence must be in [0, 3]")
        for name in ("position_correction_gain", "yaw_correction_gain"):
            value = _finite(name, getattr(self, name))
            if not 0.0 <= value <= 1.0:
                raise ConfigV2Error(f"fusion.{name} must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    position_tolerance_m: float
    yaw_tolerance_rad: float
    lookahead_m: float
    rotate_in_place_threshold_rad: float
    slowdown_distance_m: float
    safety_margin_m: float = 0.05
    path_yaw_gain: float = 1.5
    final_yaw_gain: float = 1.5
    degraded_speed_scale: float = 0.4

    def __post_init__(self) -> None:
        for name in (
            "position_tolerance_m", "yaw_tolerance_rad", "lookahead_m",
            "rotate_in_place_threshold_rad", "slowdown_distance_m", "safety_margin_m",
            "path_yaw_gain", "final_yaw_gain",
        ):
            _positive(name, getattr(self, name))
        scale = _finite("degraded_speed_scale", self.degraded_speed_scale)
        if not 0.0 < scale <= 1.0:
            raise ConfigV2Error("navigation.degraded_speed_scale must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    require_measured_geometry_for_hardware_mission: bool
    require_verified_c10b_diff_firmware_for_curved_motion: bool
    stop_if_pose_lost_s: float
    hardware_probe_max_linear_speed_m_s: float = 0.10
    hardware_probe_max_angular_speed_rad_s: float = 0.35
    require_measured_extrinsics_for_hardware_mission: bool = True

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
    d500_mount: SensorMount3DConfig
    t265: T265Config
    t265_mount: SensorMount3DConfig
    fusion: FusionConfig
    navigation: NavigationConfig
    safety: SafetyConfig

    def __post_init__(self) -> None:
        if self.schema_version != 2:
            raise ConfigV2Error(
                f"unsupported schema_version {self.schema_version}; "
                "load legacy schema-v1 files with load_car_config()"
            )
        if not self.robot_name.strip():
            raise ConfigV2Error("robot_name must not be empty")
