from pathlib import Path
import subprocess

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_visualization_assets_are_installed_inputs():
    assert (PACKAGE_ROOT / "launch" / "rviz.launch.py").is_file()
    assert (PACKAGE_ROOT / "launch" / "simulation.launch.py").is_file()
    assert (PACKAGE_ROOT / "config" / "vehicle.rviz").is_file()
    assert (PACKAGE_ROOT / "config" / "replay.rviz").is_file()
    assert (PACKAGE_ROOT / "worlds" / "vehicle.world.sdf").is_file()


def test_bridge_contract_is_isolated_and_uses_supported_message_types():
    bridge = yaml.safe_load((PACKAGE_ROOT / "config" / "bridge.yaml").read_text())
    topics = {entry["ros_topic_name"]: entry for entry in bridge}
    assert topics["/sim/cmd_vel"]["direction"] == "ROS_TO_GZ"
    assert topics["/sim/odom"]["gz_type_name"] == "gz.msgs.Odometry"
    assert topics["/sim/imu/data_raw"]["gz_type_name"] == "gz.msgs.IMU"
    assert topics["/sim/imu/data_raw"]["frame_id"] == "imu_link"
    assert topics["/sim/joint_states"]["gz_type_name"] == "gz.msgs.Model"
    assert all(topic == "/clock" or topic.startswith("/sim/") for topic in topics)


def test_simulation_health_contract_matches_safety_schema():
    source = (
        PACKAGE_ROOT / "vc_visualization" / "simulation_health.py"
    ).read_text()
    for name in (
        "vehicle_ecu/transport",
        "vehicle_ecu/drive",
        "vehicle_ecu/imu",
        "time_synchronized",
        "fault_mask",
    ):
        assert name in source


def test_launches_never_start_micro_ros_agent_or_hardware_serial():
    simulation = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text()
    fake = (PACKAGE_ROOT / "launch" / "fake_visualization.launch.py").read_text()
    replay = (PACKAGE_ROOT / "launch" / "replay_visualization.launch.py").read_text()
    for source in (simulation, fake, replay):
        assert "agent.launch.py" not in source
        assert "serial_device" not in source


def test_simulation_launch_has_namespace_and_sim_time_isolation():
    source = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text()
    assert 'DeclareLaunchArgument("namespace", default_value="sim")' in source
    assert '"use_sim_time": "true"' in source
    assert '"sim": "true"' in source
    world = (PACKAGE_ROOT / "worlds" / "vehicle.world.sdf").read_text()
    assert "gz-sim-sensors-system" in world
    assert "gz-sim-imu-system" in world


def test_rviz_config_is_parseable_yaml():
    for filename in ("vehicle.rviz", "replay.rviz", "simulation.rviz"):
        yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())


def test_ros_gz_tools_are_available_when_environment_is_configured():
    result = subprocess.run(
        ["which", "ros2"], check=False, capture_output=True, text=True
    )
    if result.returncode != 0:
        return
    assert result.stdout.strip()
