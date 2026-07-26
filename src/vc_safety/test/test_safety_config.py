from pathlib import Path

import pytest

from vc_safety.safety_config import load_safety_config, SafetyConfigError
import yaml


def valid_parameters():
    return {
        "publish_rate_hz": 20.0,
        "status_rate_hz": 5.0,
        "data_timeout_ms": 200,
        "diagnostics_timeout_ms": 500,
        "command_timeout_ms": 250,
        "future_tolerance_ms": 100,
        "max_linear_speed_mps": 1.0,
        "max_angular_speed_rps": 2.0,
        "odom_frame": "odom",
        "base_frame": "base_link",
        "imu_frame": "imu_link",
        "enable_topic_statistics": True,
        "statistics_topic": "vehicle/safety/statistics",
        "use_sim_time": False,
    }


def write_config(path, parameters):
    document = {"/**": {"ros__parameters": parameters}}
    path.write_text(yaml.safe_dump(document), encoding="utf-8")


def test_installed_default_config_is_valid():
    config_path = Path(__file__).resolve().parents[1] / "config" / "safety.yaml"
    parameters = load_safety_config(config_path)
    assert parameters["command_timeout_ms"] == 250


def test_missing_unknown_and_invalid_parameters_fail_closed(tmp_path):
    config_path = tmp_path / "safety.yaml"

    parameters = valid_parameters()
    del parameters["command_timeout_ms"]
    write_config(config_path, parameters)
    with pytest.raises(SafetyConfigError, match="missing safety parameters"):
        load_safety_config(config_path)

    parameters = valid_parameters()
    parameters["command_timeot_ms"] = 250
    write_config(config_path, parameters)
    with pytest.raises(SafetyConfigError, match="unknown safety parameters"):
        load_safety_config(config_path)

    parameters = valid_parameters()
    parameters["max_linear_speed_mps"] = float("nan")
    write_config(config_path, parameters)
    with pytest.raises(SafetyConfigError, match="max_linear_speed_mps"):
        load_safety_config(config_path)
