"""Run deterministic fake ECU stack with RViz2."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("vc_bringup")
    visualization_share = get_package_share_directory("vc_visualization")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Visualization namespace"
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=bringup_share + "/config/vehicle.fake.yaml",
                description="Deterministic fake vehicle config",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    bringup_share + "/launch/fake_ecu.launch.py"
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "vehicle_config": LaunchConfiguration("vehicle_config"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    visualization_share + "/launch/rviz.launch.py"
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "use_sim_time": "false",
                    "rviz_config": visualization_share + "/config/vehicle.rviz",
                }.items(),
            ),
        ]
    )
