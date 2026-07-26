"""Record the vehicle observability contract to an MCAP rosbag."""

import hashlib
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _recorded_topics(namespace, record_camera):
    prefix = f"/{namespace.strip('/')}" if namespace.strip("/") else ""
    topics = [
        f"{prefix}/odom",
        f"{prefix}/imu/data_raw",
        f"{prefix}/joint_states",
        f"{prefix}/odometry/filtered",
        f"{prefix}/cmd_vel_request",
        f"{prefix}/cmd_vel",
        f"{prefix}/diagnostics",
        f"{prefix}/vehicle/motion_enabled",
        f"{prefix}/vehicle/safety/diagnostics",
        f"{prefix}/vehicle/safety/statistics",
        f"{prefix}/vehicle/diagnostics",
        "/parameter_events",
        "/tf",
        "/tf_static",
    ]
    if record_camera:
        topics.extend(
            [
                f"{prefix}/camera/image_raw/compressed",
                f"{prefix}/camera/camera_info",
            ]
        )
    return topics


def _metadata(config_path):
    try:
        revision = subprocess.check_output(
            ["git", "-c", "safe.directory=*", "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
        ).strip()
    except (OSError, subprocess.SubprocessError):
        revision = "unknown"
    try:
        with open(config_path, "rb") as config_file:
            config_hash = hashlib.sha256(config_file.read()).hexdigest()
    except OSError:
        config_hash = "unavailable"
    return revision, config_hash


def _recorder(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    record_camera = (
        LaunchConfiguration("record_camera").perform(context).lower() == "true"
    )
    topics = _recorded_topics(namespace, record_camera)
    revision, config_hash = _metadata(
        LaunchConfiguration("vehicle_config").perform(context)
    )
    command = [
        "ros2",
        "bag",
        "record",
        "--storage",
        LaunchConfiguration("record_storage_id").perform(context),
        "--output",
        LaunchConfiguration("bag_output").perform(context),
        "--qos-profile-overrides-path",
        LaunchConfiguration("qos_overrides").perform(context),
        "--custom-data",
        f"git_revision={revision}",
        f"vehicle_config_sha256={config_hash}",
    ]
    if LaunchConfiguration("snapshot_mode").perform(context).lower() == "true":
        command.append("--snapshot-mode")
    command.append("--topics")
    command.extend(topics)
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("bag_output", default_value="bags/vehicle"),
            DeclareLaunchArgument("record_storage_id", default_value="mcap"),
            DeclareLaunchArgument("snapshot_mode", default_value="false"),
            DeclareLaunchArgument("record_camera", default_value="false"),
            DeclareLaunchArgument("vehicle_config"),
            DeclareLaunchArgument("qos_overrides"),
            OpaqueFunction(function=_recorder),
        ]
    )
