"""Aggregate ECU, safety, and state-estimation diagnostics."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("vc_bringup")
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Diagnostics namespace"
            ),
            DeclareLaunchArgument(
                "diagnostics_config",
                default_value=share + "/config/diagnostics.yaml",
                description="diagnostic_aggregator configuration",
            ),
            DeclareLaunchArgument(
                "monitor_serial",
                default_value="false",
                description="Monitor the stable serial device",
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value="",
                description="Serial path used by the diagnostic monitor",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            Node(
                package="vc_bringup",
                executable="serial_connection_monitor",
                name="serial_connection_monitor",
                namespace=namespace,
                parameters=[
                    {
                        "serial_device": LaunchConfiguration("serial_device"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("monitor_serial")),
                output="screen",
            ),
            Node(
                package="vc_bringup",
                executable="state_estimation_monitor",
                name="state_estimation_monitor",
                namespace=namespace,
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[("/diagnostics", "computer/diagnostics")],
                output="screen",
            ),
            Node(
                package="vc_bringup",
                executable="diagnostics_mux",
                name="vehicle_diagnostics_mux",
                namespace=namespace,
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                namespace=namespace,
                parameters=[
                    LaunchConfiguration("diagnostics_config"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("/diagnostics", "vehicle/diagnostics_input"),
                    ("/diagnostics_agg", "vehicle/diagnostics"),
                ],
                output="screen",
            ),
        ]
    )
