import importlib.util
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
import yaml


def test_uncalibrated_hardware_template_fails_at_launch():
    share = get_package_share_directory("vc_bringup")
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "vc_bringup",
            "state_estimation.launch.py",
            f"vehicle_config:={share}/config/vehicle.yaml",
        ],
        capture_output=True,
        check=False,
        text=True,
        timeout=10,
    )
    assert result.returncode != 0
    assert "calibrated" in result.stdout + result.stderr


def load_launch_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_replay_profile_isolates_recorded_outputs():
    share = Path(get_package_share_directory("vc_bringup"))
    launch_path = share / "launch" / "replay.launch.py"
    source = launch_path.read_text(encoding="utf-8")
    replay = load_launch_module(launch_path, "vc_bringup_replay_launch")

    assert '"cmd_vel_output": "replay/cmd_vel_sink"' in source
    assert "agent.launch.py" not in source
    assert '"use_sim_time": "true"' in source
    remaps = replay._replay_remaps("")
    assert "/cmd_vel:=/replay/recorded/cmd_vel" in remaps
    assert "/odometry/filtered:=/replay/recorded/odometry/filtered" in remaps
    assert "/tf:=/replay/recorded/tf" in remaps
    assert "/tf_static:=/replay/recorded/tf_static" in remaps

    namespaced = replay._replay_remaps("ci_vehicle")
    assert (
        "/ci_vehicle/cmd_vel:=/ci_vehicle/replay/recorded/cmd_vel"
        in namespaced
    )
    assert (
        "/ci_vehicle/vehicle/motion_enabled:="
        "/ci_vehicle/replay/recorded/vehicle/motion_enabled"
        in namespaced
    )


def test_missing_safety_config_fails_closed(tmp_path):
    missing = tmp_path / "missing-safety.yaml"
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "vc_bringup",
            "safety.launch.py",
            f"safety_config:={missing}",
        ],
        capture_output=True,
        check=False,
        text=True,
        timeout=10,
    )
    assert result.returncode != 0
    assert "safety config is not a file" in result.stdout + result.stderr


def test_camera_profile_uses_camera_ros_and_validates_config(tmp_path):
    share = Path(get_package_share_directory("vc_bringup"))
    source = (share / "launch" / "camera.launch.py").read_text(encoding="utf-8")
    config = yaml.safe_load(
        (share / "config" / "camera.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert 'package="camera_ros"' in source
    assert 'executable="camera_node"' in source
    assert config["format"] == "RGB888"
    assert config["width"] == 1280
    assert config["height"] == 720
    assert config["FrameDurationLimits"] == [33333, 33333]

    missing = tmp_path / "missing-camera.yaml"
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "vc_bringup",
            "camera.launch.py",
            f"camera_config:={missing}",
        ],
        capture_output=True,
        check=False,
        text=True,
        timeout=10,
    )
    assert result.returncode != 0
    assert "camera config is not a file" in result.stdout + result.stderr


def test_vehicle_camera_is_optional_and_recording_uses_compressed_transport():
    share = Path(get_package_share_directory("vc_bringup"))
    vehicle_source = (share / "launch" / "vehicle.launch.py").read_text(
        encoding="utf-8"
    )
    recording_path = share / "launch" / "recording.launch.py"
    recording = load_launch_module(recording_path, "vc_bringup_recording_launch")

    assert 'DeclareLaunchArgument("camera", default_value="true")' in vehicle_source
    assert 'condition=IfCondition(LaunchConfiguration("camera"))' in vehicle_source

    without_camera = recording._recorded_topics("", False)
    assert "/camera/image_raw/compressed" not in without_camera

    with_camera = recording._recorded_topics("ci_vehicle", True)
    assert "/ci_vehicle/camera/image_raw/compressed" in with_camera
    assert "/ci_vehicle/camera/camera_info" in with_camera
    assert "/ci_vehicle/camera/image_raw" not in with_camera


def test_diagnostics_have_a_single_vehicle_root_and_sim_time_support():
    share = Path(get_package_share_directory("vc_bringup"))
    config = (share / "config" / "diagnostics.yaml").read_text(encoding="utf-8")
    launch_source = (share / "launch" / "diagnostics.launch.py").read_text(
        encoding="utf-8"
    )
    assert "path: Vehicle/ECU/Transport" in config
    assert "path: Vehicle/Computer/SafetyGate" in config
    assert 'DeclareLaunchArgument("use_sim_time"' in launch_source
    assert launch_source.count('"use_sim_time": LaunchConfiguration("use_sim_time")') == 3


def test_agent_restart_exhaustion_requests_clean_launch_shutdown():
    share = Path(get_package_share_directory("vc_bringup"))
    source = (share / "launch" / "agent.launch.py").read_text(encoding="utf-8")
    assert "event=Shutdown(" in source
    assert "restart limit reached" in source
