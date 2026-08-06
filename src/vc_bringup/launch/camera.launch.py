"""Launch the libcamera-backed ROS 2 camera driver."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(context):
    config_path = LaunchConfiguration("camera_config").perform(context)
    if not Path(config_path).is_file():
        raise ValueError(f"camera config is not a file: {config_path}")

    return [
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            namespace=LaunchConfiguration("namespace"),
            parameters=[
                config_path,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                description="camera_ros parameter YAML",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Camera namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            OpaqueFunction(function=_camera_node),
        ]
    )
