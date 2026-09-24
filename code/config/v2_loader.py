"""TOML loader for schema-v2 differential robot profiles."""

from __future__ import annotations

from dataclasses import fields
from pathlib import Path
from typing import Any, TypeVar

try:
    import tomllib
except ModuleNotFoundError:  # Keep v1 package imports usable on Python 3.10.
    import tomli as tomllib  # type: ignore[no-redef]

from .v2_models import (
    C10BConfig,
    CalibrationStatusConfig,
    ConfigV2Error,
    D500Config,
    DifferentialDriveConfig,
    DifferentialGeometryConfig,
    DifferentialRobotConfig,
    FusionConfig,
    NavigationConfig,
    SafetyConfig,
    SensorMount3DConfig,
    T265Config,
)

T = TypeVar("T")
DEFAULT_V2_CONFIG = Path(__file__).resolve().parents[2] / "configs" / "robocup_diffdrive.example.toml"


def _table(parent: dict[str, Any], key: str, path: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise ConfigV2Error(f"{path} must be a TOML table")
    return value


def _build(model: type[T], value: dict[str, Any], path: str) -> T:
    allowed = {field.name for field in fields(model)}
    unknown = set(value) - allowed
    if unknown:
        raise ConfigV2Error(f"unknown key(s) in {path}: {', '.join(sorted(unknown))}")
    try:
        return model(**value)
    except (TypeError, ValueError) as exc:
        raise ConfigV2Error(f"invalid {path}: {exc}") from exc


def load_v2_config(path: str | Path | None = None) -> DifferentialRobotConfig:
    """Load and validate schema v2; use ``load_car_config`` for schema v1."""

    config_path = DEFAULT_V2_CONFIG if path is None else Path(path)
    try:
        with config_path.open("rb") as stream:
            document = tomllib.load(stream)
    except OSError as exc:
        raise ConfigV2Error(f"cannot read config {config_path}: {exc}") from exc
    except tomllib.TOMLDecodeError as exc:
        raise ConfigV2Error(f"invalid TOML in {config_path}: {exc}") from exc

    version = document.get("schema_version")
    if version != 2:
        raise ConfigV2Error(
            f"unsupported schema_version {version!r}; load legacy schema-v1 files "
            "with load_car_config()"
        )
    root_allowed = {
        "schema_version", "robot_name", "calibration", "vehicle", "devices",
        "sensors", "fusion", "navigation", "safety",
    }
    unknown = set(document) - root_allowed
    if unknown:
        raise ConfigV2Error(f"unknown top-level key(s): {', '.join(sorted(unknown))}")

    vehicle = _table(document, "vehicle", "vehicle")
    devices = _table(document, "devices", "devices")
    sensors = _table(document, "sensors", "sensors")
    if set(vehicle) != {"geometry", "drive"}:
        raise ConfigV2Error("vehicle must contain [vehicle.geometry] and [vehicle.drive]")
    if set(devices) != {"c10b", "d500", "t265"}:
        raise ConfigV2Error("devices must contain [devices.c10b], [devices.d500], [devices.t265]")
    if set(sensors) != {"d500", "t265"}:
        raise ConfigV2Error("sensors must contain [sensors.d500] and [sensors.t265]")
    d500_sensor = _table(sensors, "d500", "sensors.d500")
    t265_sensor = _table(sensors, "t265", "sensors.t265")
    if set(d500_sensor) != {"mount"} or set(t265_sensor) != {"mount"}:
        raise ConfigV2Error("each sensor table must contain exactly one [mount] table")

    return DifferentialRobotConfig(
        schema_version=version,
        robot_name=document.get("robot_name", ""),
        calibration=_build(CalibrationStatusConfig, _table(document, "calibration", "calibration"), "calibration"),
        geometry=_build(DifferentialGeometryConfig, _table(vehicle, "geometry", "vehicle.geometry"), "vehicle.geometry"),
        drive=_build(DifferentialDriveConfig, _table(vehicle, "drive", "vehicle.drive"), "vehicle.drive"),
        c10b=_build(C10BConfig, _table(devices, "c10b", "devices.c10b"), "devices.c10b"),
        d500=_build(D500Config, _table(devices, "d500", "devices.d500"), "devices.d500"),
        d500_mount=_build(SensorMount3DConfig, _table(d500_sensor, "mount", "sensors.d500.mount"), "sensors.d500.mount"),
        t265=_build(T265Config, _table(devices, "t265", "devices.t265"), "devices.t265"),
        t265_mount=_build(SensorMount3DConfig, _table(t265_sensor, "mount", "sensors.t265.mount"), "sensors.t265.mount"),
        fusion=_build(FusionConfig, _table(document, "fusion", "fusion"), "fusion"),
        navigation=_build(NavigationConfig, _table(document, "navigation", "navigation"), "navigation"),
        safety=_build(SafetyConfig, _table(document, "safety", "safety"), "safety"),
    )
