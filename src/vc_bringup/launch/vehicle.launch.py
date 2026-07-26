"""Top-level hardware launch for the RPi5 vehicle computer."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from vc_bringup.config_loader import load_vehicle_config


def _validate_configuration(context):
    load_vehicle_config(LaunchConfiguration("vehicle_config").perform(context))
    return []


def generate_launch_description():
    share = get_package_share_directory("vc_bringup")
    safety_share = get_package_share_directory("vc_safety")
    common = {
        "namespace": LaunchConfiguration("namespace"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_device"),
            DeclareLaunchArgument("baudrate", default_value="921600"),
            DeclareLaunchArgument("agent_verbosity", default_value="4"),
            DeclareLaunchArgument("agent_respawn_delay", default_value="2.0"),
            DeclareLaunchArgument("agent_respawn_limit", default_value="300"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument("snapshot_mode", default_value="false"),
            DeclareLaunchArgument("trace", default_value="false"),
            DeclareLaunchArgument("bag_output", default_value="bags/vehicle"),
            DeclareLaunchArgument("record_storage_id", default_value="mcap"),
            DeclareLaunchArgument("trace_session", default_value="vehicle"),
            DeclareLaunchArgument("trace_path", default_value="traces"),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=share + "/config/vehicle.yaml",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=safety_share + "/config/safety.yaml",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=share + "/config/camera.yaml",
            ),
            OpaqueFunction(function=_validate_configuration),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share + "/launch/agent.launch.py"),
                launch_arguments={
                    "serial_device": LaunchConfiguration("serial_device"),
                    "baudrate": LaunchConfiguration("baudrate"),
                    "agent_verbosity": LaunchConfiguration("agent_verbosity"),
                    "agent_respawn_delay": LaunchConfiguration(
                        "agent_respawn_delay"
                    ),
                    "agent_respawn_limit": LaunchConfiguration(
                        "agent_respawn_limit"
                    ),
                    "namespace": LaunchConfiguration("namespace"),
                }.items(),
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
                    "safety_config": LaunchConfiguration("safety_config"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(share + "/launch/camera.launch.py"),
                launch_arguments={
                    **common,
                    "camera_config": LaunchConfiguration("camera_config"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("camera")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    share + "/launch/recording.launch.py"
                ),
                launch_arguments={
                    "bag_output": LaunchConfiguration("bag_output"),
                    "record_storage_id": LaunchConfiguration("record_storage_id"),
                    "snapshot_mode": LaunchConfiguration("snapshot_mode"),
                    "record_camera": LaunchConfiguration("camera"),
                    "namespace": LaunchConfiguration("namespace"),
                    "vehicle_config": LaunchConfiguration("vehicle_config"),
                    "qos_overrides": (
                        share + "/config/recording_qos.yaml"
                    ),
                }.items(),
                condition=IfCondition(LaunchConfiguration("record")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    share + "/launch/tracing.launch.py"
                ),
                launch_arguments={
                    "trace_session": LaunchConfiguration("trace_session"),
                    "trace_path": LaunchConfiguration("trace_path"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("trace")),
            ),
        ]
    )
