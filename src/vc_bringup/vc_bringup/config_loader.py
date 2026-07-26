"""Load and validate physical vehicle configuration."""

from math import isfinite
from pathlib import Path
from typing import Any

import yaml


POSITIVE_DIMENSIONS = (
    "base_length",
    "base_width",
    "base_height",
    "wheel_radius",
    "wheel_width",
    "wheel_base",
    "track_width",
)
IMU_TRANSFORM = (
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
)
SCHEMA_VERSION = 1


class VehicleConfigError(ValueError):
    """Raised when a configuration is unsafe or incomplete."""


def _finite_number(mapping: dict[str, Any], key: str, section: str) -> float:
    if key not in mapping:
        raise VehicleConfigError(f"missing {section}.{key}")
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VehicleConfigError(f"{section}.{key} must be a number")
    value = float(value)
    if not isfinite(value):
        raise VehicleConfigError(f"{section}.{key} must be finite")
    return value


def load_vehicle_config(path: str) -> dict[str, dict[str, float]]:
    """Return a validated vehicle configuration from *path*."""
    config_path = Path(path)
    if not config_path.is_file():
        raise VehicleConfigError(f"vehicle config does not exist: {path}")

    with config_path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise VehicleConfigError("vehicle config must be a YAML mapping")
    if document.get("schema_version") != SCHEMA_VERSION:
        raise VehicleConfigError(
            f"schema_version must be the integer {SCHEMA_VERSION}"
        )
    unknown_sections = set(document) - {"schema_version", "vehicle", "imu"}
    if unknown_sections:
        raise VehicleConfigError(
            f"unknown vehicle config sections: {sorted(unknown_sections)}"
        )

    vehicle = document.get("vehicle")
    imu = document.get("imu")
    if not isinstance(vehicle, dict) or not isinstance(imu, dict):
        raise VehicleConfigError("vehicle and imu mappings are required")
    if vehicle.get("calibrated") is not True:
        raise VehicleConfigError(
            "vehicle.calibrated must be true after physical measurement"
        )
    unknown_vehicle_keys = set(vehicle) - {"calibrated", *POSITIVE_DIMENSIONS}
    if unknown_vehicle_keys:
        raise VehicleConfigError(
            f"unknown vehicle keys: {sorted(unknown_vehicle_keys)}"
        )
    unknown_imu_keys = set(imu) - set(IMU_TRANSFORM)
    if unknown_imu_keys:
        raise VehicleConfigError(f"unknown imu keys: {sorted(unknown_imu_keys)}")

    validated_vehicle: dict[str, float] = {}
    for key in POSITIVE_DIMENSIONS:
        value = _finite_number(vehicle, key, "vehicle")
        if value <= 0.0:
            raise VehicleConfigError(f"vehicle.{key} must be positive")
        validated_vehicle[key] = value

    validated_imu = {
        key: _finite_number(imu, key, "imu") for key in IMU_TRANSFORM
    }
    return {"vehicle": validated_vehicle, "imu": validated_imu}


def validate_serial_device(path: str) -> str:
    """Require a stable Linux serial-by-id path without resolving the symlink."""
    prefix = "/dev/serial/by-id/"
    suffix = path[len(prefix):]
    if not path.startswith(prefix) or not suffix or "/" in suffix:
        raise VehicleConfigError(
            "serial_device must be a single /dev/serial/by-id/<device> path"
        )
    return path
