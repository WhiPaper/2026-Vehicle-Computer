from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def test_xacro_expands_to_expected_tree():
    xacro_path = Path(__file__).resolve().parents[1] / "urdf" / "vehicle.urdf.xacro"
    result = subprocess.run(
        [
            "xacro",
            xacro_path,
            "base_length:=0.40",
            "base_width:=0.30",
            "base_height:=0.12",
            "wheel_radius:=0.05",
            "wheel_width:=0.03",
            "wheel_base:=0.28",
            "track_width:=0.26",
            "imu_x:=0.0",
            "imu_y:=0.0",
            "imu_z:=0.05",
            "imu_roll:=0.0",
            "imu_pitch:=0.0",
            "imu_yaw:=0.0",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)

    links = {element.attrib["name"] for element in root.findall("link")}
    assert links == {
        "base_link",
        "imu_link",
        "front_left_wheel_link",
        "rear_left_wheel_link",
        "front_right_wheel_link",
        "rear_right_wheel_link",
    }

    joints = {element.attrib["name"]: element for element in root.findall("joint")}
    assert joints["left_wheel_joint"].attrib["type"] == "continuous"
    assert joints["right_wheel_joint"].attrib["type"] == "continuous"
    assert joints["imu_joint"].attrib["type"] == "fixed"
    assert (
        joints["rear_left_wheel_joint"].find("mimic").attrib["joint"]
        == "left_wheel_joint"
    )
    assert (
        joints["rear_right_wheel_joint"].find("mimic").attrib["joint"]
        == "right_wheel_joint"
    )
    assert not root.findall("gazebo")

    subprocess.run(
        ["check_urdf", "-"],
        input=result.stdout,
        check=True,
        capture_output=True,
        text=True,
    )


def test_sim_xacro_adds_gazebo_drive_and_sensor_plugins():
    xacro_path = Path(__file__).resolve().parents[1] / "urdf" / "vehicle.urdf.xacro"
    result = subprocess.run(
        ["xacro", xacro_path, "sim:=true"],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    plugins = root.findall("gazebo/plugin")
    plugin_names = {plugin.attrib["name"] for plugin in plugins}
    assert "gz::sim::systems::DiffDrive" in plugin_names
    assert "gz::sim::systems::JointStatePublisher" in plugin_names
    imu = root.find("gazebo[@reference='imu_link']/sensor")
    assert imu is not None
    assert imu.attrib["type"] == "imu"
