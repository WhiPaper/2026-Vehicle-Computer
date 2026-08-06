"""Include the vc_safety managed-node launch with bringup arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Safety gate namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Configure and activate the lifecycle gate",
            ),
            DeclareLaunchArgument(
                "cmd_vel_output",
                default_value="cmd_vel",
                description="Relative gated command output name",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("vc_safety"),
                            "launch",
                            "safety.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "auto_start": LaunchConfiguration("auto_start"),
                    "cmd_vel_output": LaunchConfiguration("cmd_vel_output"),
                }.items(),
            ),
        ]
    )
