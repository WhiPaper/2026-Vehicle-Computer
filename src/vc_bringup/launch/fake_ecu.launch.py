"""Run the ROS vehicle stack against a deterministic fake ECU."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("vc_bringup")
    common = {
        "namespace": LaunchConfiguration("namespace"),
        "use_sim_time": "false",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Fake ECU namespace"
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=share + "/config/vehicle.fake.yaml",
                description="Deterministic fake vehicle config",
            ),
            Node(
                package="vc_bringup",
                executable="fake_ecu",
                name="vehicle_ecu",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
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
                launch_arguments=common.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    share + "/launch/diagnostics.launch.py"
                ),
                launch_arguments={
                    **common,
                }.items(),
            ),
        ]
    )
