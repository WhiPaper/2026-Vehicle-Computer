"""Strict validation for the safety gate parameter file."""

from math import isfinite
from pathlib import Path

import yaml


POSITIVE_FLOAT_PARAMETERS = {
    "publish_rate_hz",
    "status_rate_hz",
    "max_linear_speed_mps",
    "max_angular_speed_rps",
}
POSITIVE_INTEGER_PARAMETERS = {
    "data_timeout_ms",
    "diagnostics_timeout_ms",
    "command_timeout_ms",
    "future_tolerance_ms",
}
STRING_PARAMETERS = {
    "odom_frame",
    "base_frame",
    "imu_frame",
    "statistics_topic",
}
BOOLEAN_PARAMETERS = {"enable_topic_statistics", "use_sim_time"}
SAFETY_PARAMETERS = (
    POSITIVE_FLOAT_PARAMETERS
    | POSITIVE_INTEGER_PARAMETERS
    | STRING_PARAMETERS
    | BOOLEAN_PARAMETERS
)


class SafetyConfigError(ValueError):
    """Raised when a safety configuration cannot be trusted."""


def _parameter_mapping(document):
    if not isinstance(document, dict) or set(document) != {"/**"}:
        raise SafetyConfigError(
            "safety config must contain only the '/**' node selector"
        )
    node_config = document["/**"]
    if not isinstance(node_config, dict) or set(node_config) != {"ros__parameters"}:
        raise SafetyConfigError(
            "safety config must contain only '/**.ros__parameters'"
        )
    parameters = node_config["ros__parameters"]
    if not isinstance(parameters, dict):
        raise SafetyConfigError("safety ros__parameters must be a mapping")
    return parameters


def _validate_keys(parameters):
    missing = SAFETY_PARAMETERS - set(parameters)
    unknown = set(parameters) - SAFETY_PARAMETERS
    if missing:
        raise SafetyConfigError(f"missing safety parameters: {sorted(missing)}")
    if unknown:
        raise SafetyConfigError(f"unknown safety parameters: {sorted(unknown)}")


def _validate_values(parameters):
    for name in POSITIVE_FLOAT_PARAMETERS:
        value = parameters[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(float(value))
            or not 0.001 <= float(value) <= 1000.0
        ):
            raise SafetyConfigError(
                f"{name} must be finite and in [0.001, 1000.0]"
            )
    for name in POSITIVE_INTEGER_PARAMETERS:
        value = parameters[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 60000
        ):
            raise SafetyConfigError(f"{name} must be an integer in [1, 60000]")
    for name in STRING_PARAMETERS:
        value = parameters[name]
        if not isinstance(value, str) or not value.strip():
            raise SafetyConfigError(f"{name} must be a non-empty string")
    for name in BOOLEAN_PARAMETERS:
        if not isinstance(parameters[name], bool):
            raise SafetyConfigError(f"{name} must be a boolean")


def load_safety_config(path):
    """Load and validate a complete safety gate configuration."""
    config_path = Path(path)
    if not config_path.is_file():
        raise SafetyConfigError(f"safety config is not a file: {config_path}")
    try:
        with config_path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise SafetyConfigError(
            f"cannot load safety config {config_path}: {error}"
        ) from error

    parameters = _parameter_mapping(document)
    _validate_keys(parameters)
    _validate_values(parameters)
    return parameters
