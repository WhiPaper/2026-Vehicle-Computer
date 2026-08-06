"""Replay an MCAP bag with its safe output isolation and RViz2."""

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
                "bag_path", description="MCAP bag directory to replay"
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Replay namespace"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    bringup_share + "/launch/replay.launch.py"
                ),
                launch_arguments={
                    "bag_path": LaunchConfiguration("bag_path"),
                    "namespace": LaunchConfiguration("namespace"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    visualization_share + "/launch/rviz.launch.py"
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "use_sim_time": "true",
                    "replay": "true",
                    "rviz_config": visualization_share + "/config/replay.rviz",
                }.items(),
            ),
        ]
    )
