"""Replay a vehicle bag without allowing output to reach hardware cmd_vel."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _replay_remaps(namespace):
    prefix = f"/{namespace.strip('/')}" if namespace.strip("/") else ""
    recorded_prefix = f"{prefix}/replay/recorded"
    outputs = [
        "cmd_vel",
        "odometry/filtered",
        "vehicle/motion_enabled",
        "vehicle/safety/diagnostics",
        "vehicle/safety/statistics",
        "vehicle/diagnostics",
    ]
    remaps = [
        f"{prefix}/{topic}:={recorded_prefix}/{topic}" for topic in outputs
    ]
    remaps.extend(
        [
            "/tf:=/replay/recorded/tf",
            "/tf_static:=/replay/recorded/tf_static",
        ]
    )
    return remaps


def _bag_player(context):
    command = [
        "ros2",
        "bag",
        "play",
        LaunchConfiguration("bag_path").perform(context),
        "--clock",
        "--remap",
        *_replay_remaps(LaunchConfiguration("namespace").perform(context)),
    ]
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    share = get_package_share_directory("vc_bringup")
    common = {
        "namespace": LaunchConfiguration("namespace"),
        "use_sim_time": "true",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path", description="MCAP bag directory to replay"
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Replay namespace"
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=share + "/config/vehicle.fake.yaml",
                description="Fake vehicle config for replay state estimation",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    share + "/launch/state_estimation.launch.py"
                ),
                launch_arguments={
                    **common,
                    "vehicle_config": LaunchConfiguration("vehicle_config"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share + "/launch/safety.launch.py"),
                launch_arguments={
                    **common,
                    "cmd_vel_output": "replay/cmd_vel_sink",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    share + "/launch/diagnostics.launch.py"
                ),
                launch_arguments={
                    **common,
                }.items(),
            ),
            OpaqueFunction(function=_bag_player),
        ]
    )
