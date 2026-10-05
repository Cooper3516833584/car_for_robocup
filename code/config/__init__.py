"""Lazy exports for the schema-v2 differential configuration."""

from importlib import import_module as _import_module

_EXPORT_MODULES = {
    'CalibrationStatusConfig': 'v2_models',
    'ConfigV2Error': 'v2_models',
    'DifferentialDriveConfig': 'v2_models',
    'DifferentialGeometryConfig': 'v2_models',
    'DifferentialRobotConfig': 'v2_models',
    'FusionConfig': 'v2_models',
    'NavigationConfig': 'v2_models',
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
