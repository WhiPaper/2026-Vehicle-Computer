"""Start a non-interactive ros2_tracing session for callback profiling."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    session = LaunchConfiguration("trace_session")
    trace_path = LaunchConfiguration("trace_path")
    start = ExecuteProcess(
        cmd=["ros2", "trace", "start", session, "--path", trace_path],
        output="screen",
    )
    stop = ExecuteProcess(
        cmd=["ros2", "trace", "stop", session],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("trace_session", default_value="vehicle"),
            DeclareLaunchArgument("trace_path", default_value="traces"),
            start,
            RegisterEventHandler(OnShutdown(on_shutdown=[stop])),
        ]
    )
