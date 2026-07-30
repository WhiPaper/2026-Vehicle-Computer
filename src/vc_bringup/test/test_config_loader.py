from pathlib import Path

import pytest

from vc_bringup.config_loader import (
    load_vehicle_config,
    validate_serial_device,
    VehicleConfigError,
)
import yaml


def valid_document():
    return {
        "schema_version": 1,
        "vehicle": {
            "calibrated": True,
            "base_length": 0.4,
            "base_width": 0.3,
            "base_height": 0.12,
            "wheel_radius": 0.05,
            "wheel_width": 0.03,
            "wheel_base": 0.28,
            "track_width": 0.26,
        },
        "imu": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.05,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    }


def write_yaml(path: Path, document):
    path.write_text(yaml.safe_dump(document), encoding="utf-8")


def test_valid_config_is_loaded(tmp_path):
    path = tmp_path / "vehicle.yaml"
    write_yaml(path, valid_document())
    config = load_vehicle_config(str(path))
    assert config["vehicle"]["wheel_radius"] == pytest.approx(0.05)
    assert config["imu"]["z"] == pytest.approx(0.05)


def test_uncalibrated_template_is_rejected(tmp_path):
    document = valid_document()
    document["vehicle"]["calibrated"] = False
    path = tmp_path / "vehicle.yaml"
    write_yaml(path, document)
    with pytest.raises(VehicleConfigError, match="calibrated"):
        load_vehicle_config(str(path))


def test_unknown_schema_and_keys_are_rejected(tmp_path):
    document = valid_document()
    document["schema_version"] = 2
    path = tmp_path / "vehicle.yaml"
    write_yaml(path, document)
    with pytest.raises(VehicleConfigError, match="schema_version"):
        load_vehicle_config(str(path))

    document = valid_document()
    document["vehicle"]["wheel_raduis"] = 0.05
    write_yaml(path, document)
    with pytest.raises(VehicleConfigError, match="unknown vehicle keys"):
        load_vehicle_config(str(path))


@pytest.mark.parametrize("value", [0.0, -0.1, float("inf"), "0.1", True])
def test_unsafe_dimensions_are_rejected(tmp_path, value):
    document = valid_document()
    document["vehicle"]["wheel_radius"] = value
    path = tmp_path / "vehicle.yaml"
    write_yaml(path, document)
    with pytest.raises(VehicleConfigError):
        load_vehicle_config(str(path))


def test_missing_imu_transform_is_rejected(tmp_path):
    document = valid_document()
    del document["imu"]["yaw"]
    path = tmp_path / "vehicle.yaml"
    write_yaml(path, document)
    with pytest.raises(VehicleConfigError, match="imu.yaw"):
        load_vehicle_config(str(path))


@pytest.mark.parametrize(
    "path",
    [
        "/dev/ttyUSB0",
        "/dev/serial/by-id/",
        "/dev/serial/by-id/device/child",
        "relative-device",
    ],
)
def test_unstable_serial_paths_are_rejected(path):
    with pytest.raises(VehicleConfigError):
        validate_serial_device(path)


def test_serial_by_id_path_is_accepted(monkeypatch):
    monkeypatch.setattr(Path, "exists", lambda _self: True)
    monkeypatch.setattr("vc_bringup.config_loader.os.access", lambda *_args: True)
    assert (
        validate_serial_device("/dev/serial/by-id/usb-Silicon_Labs_CP2102")
        == "/dev/serial/by-id/usb-Silicon_Labs_CP2102"
    )


def test_missing_serial_device_is_rejected(monkeypatch):
    monkeypatch.setattr(Path, "exists", lambda _self: False)
    with pytest.raises(VehicleConfigError, match="does not exist"):
        validate_serial_device("/dev/serial/by-id/usb-Silicon_Labs_CP2102")


def test_inaccessible_serial_device_is_rejected(monkeypatch):
    monkeypatch.setattr(Path, "exists", lambda _self: True)
    monkeypatch.setattr("vc_bringup.config_loader.os.access", lambda *_args: False)
    with pytest.raises(VehicleConfigError, match="not readable and writable"):
        validate_serial_device("/dev/serial/by-id/usb-Silicon_Labs_CP2102")
