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
            DeclareLaunchArgument(
                "serial_device",
                description="Required single /dev/serial/by-id/<device> path",
            ),
            DeclareLaunchArgument(
                "baudrate", default_value="921600", description="micro-ROS baudrate"
            ),
            DeclareLaunchArgument(
                "agent_verbosity", default_value="4", description="Agent verbosity"
            ),
            DeclareLaunchArgument(
                "agent_respawn_delay",
                default_value="2.0",
                description="Agent restart delay in seconds",
            ),
            DeclareLaunchArgument(
                "agent_respawn_limit",
                default_value="300",
                description="Maximum Agent restart attempts",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Namespace for the stack"
            ),
            DeclareLaunchArgument(
                "record", default_value="false", description="Start MCAP recording"
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                description="Start the isolated camera_ros process",
            ),
            DeclareLaunchArgument(
                "snapshot_mode",
                default_value="false",
                description="Use bounded rosbag snapshot mode",
            ),
            DeclareLaunchArgument(
                "trace",
                default_value="false",
                description="Start the ros2_tracing profile",
            ),
            DeclareLaunchArgument(
                "bag_output",
                default_value="bags/vehicle",
                description="MCAP output directory",
            ),
            DeclareLaunchArgument(
                "record_storage_id",
                default_value="mcap",
                description="rosbag2 storage plugin",
            ),
            DeclareLaunchArgument(
                "trace_session",
                default_value="vehicle",
                description="Tracing session name",
            ),
            DeclareLaunchArgument(
                "trace_path",
                default_value="traces",
                description="Tracing output directory",
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=share + "/config/vehicle.yaml",
                description="Validated calibrated vehicle YAML",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=safety_share + "/config/safety.yaml",
                description="Validated safety gate parameter YAML",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=share + "/config/camera.yaml",
                description="camera_ros parameter YAML",
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
                    "diagnostics_config": (
                        share + "/config/diagnostics.hardware.yaml"
                    ),
                    "monitor_serial": "true",
                    "serial_device": LaunchConfiguration("serial_device"),
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
