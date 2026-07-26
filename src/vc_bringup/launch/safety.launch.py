"""Include the vc_safety managed-node launch with bringup arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("cmd_vel_output", default_value="cmd_vel"),
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
