"""Lazy config exports; importing schema-v2 modules stays independent of v1 hardware."""

from importlib import import_module as _import_module

_EXPORT_MODULES = {
    'AlarmGPIOConfig': 'models',
    'CameraDeviceConfig': 'models',
    'CameraLineConfig': 'models',
    'CameraPerspectiveConfig': 'models',
    'CarConfig': 'models',
    'DevicesConfig': 'models',
    'HC14DeviceConfig': 'models',
    'HardwareConfig': 'models',
    'MissionCommonConfig': 'models',
    'MissionControlConfig': 'models',
    'MissionsConfig': 'models',
    'MotorDeviceConfig': 'models',
    'ProfileConfig': 'models',
    'RadarDeviceConfig': 'models',
    'RadarMountConfig': 'models',
    'RuntimeStateConfig': 'models',
    'ScreenDeviceConfig': 'models',
    'SensorCameraConfig': 'models',
    'SensorRadarConfig': 'models',
    'SensorsConfig': 'models',
    'SteeringCalibrationConfig': 'models',
    'SteeringPWMConfig': 'models',
    'Task1Config': 'models',
    'Task2Config': 'models',
    'VehicleConfig': 'models',
    'VehicleDriveConfig': 'models',
    'VehicleGeometryConfig': 'models',
    'CAR_CONFIG_ENV_VAR': 'loader',
    'DEFAULT_CONFIG_FILENAME': 'loader',
    'ConfigError': 'loader',
    'load_car_config': 'loader',
    'resolve_config_path': 'loader',
    'RuntimeRadarCenterState': 'runtime_state',
    'load_runtime_radar_center_cm': 'runtime_state',
    'save_runtime_radar_center_cm': 'runtime_state',
    'build_steering_calibration': 'factory',
    'CalibrationStatusConfig': 'v2_models',
    'ConfigV2Error': 'v2_models',
    'DifferentialDriveConfig': 'v2_models',
    'DifferentialGeometryConfig': 'v2_models',
    'DifferentialRobotConfig': 'v2_models',
    'FusionConfig': 'v2_models',
    'DifferentialNavigationConfig': 'v2_models',
    'RuntimeMode': 'v2_runtime',
    'SafetyConfig': 'v2_models',
    'SensorMount3DConfig': 'v2_models',
    'T265Config': 'v2_models',
    'load_v2_config': 'v2_loader',
    'runtime_constraints': 'v2_runtime',
    'validate_runtime_readiness': 'v2_runtime',
}
__all__ = tuple(_EXPORT_MODULES)

def __getattr__(name: str):
    module_name = _EXPORT_MODULES.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = _import_module(f".{module_name}", __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value

def __dir__():
    return sorted(set(globals()) | set(_EXPORT_MODULES))
