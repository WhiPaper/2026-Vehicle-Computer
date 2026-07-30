"""Launch the serial micro-ROS Agent with stable-device validation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vc_bringup.config_loader import validate_serial_device


def _agent_actions(
    serial_device, baudrate, verbosity, namespace, delay, limit, attempt
):
    agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        namespace=namespace,
        arguments=[
            "serial",
            "--dev",
            serial_device,
            "-b",
            baudrate,
            "-v",
            verbosity,
        ],
        output="screen",
    )

    def on_exit(_event, _context):
        if attempt >= limit:
            return [
                LogInfo(
                    msg=(
                        "[ERROR] micro-ROS Agent restart limit reached; "
                        "stopping bringup so the service supervisor can restart it"
                    )
                ),
                EmitEvent(
                    event=Shutdown(reason="micro-ROS Agent restart limit reached")
                ),
            ]
        return [
            LogInfo(
                msg=f"[WARNING] micro-ROS Agent exited; restart {attempt + 1}/{limit}"
            ),
            TimerAction(
                period=delay,
                actions=_agent_actions(
                    serial_device,
                    baudrate,
                    verbosity,
                    namespace,
                    delay,
                    limit,
                    attempt + 1,
                ),
            ),
        ]

    return [
        agent,
        RegisterEventHandler(
            OnProcessExit(
                target_action=agent,
                on_exit=on_exit,
            )
        )
    ]


def _agent(context):
    serial_device = validate_serial_device(
        LaunchConfiguration("serial_device").perform(context)
    )
    limit = int(LaunchConfiguration("agent_respawn_limit").perform(context))
    delay = float(LaunchConfiguration("agent_respawn_delay").perform(context))
    if limit < 0 or delay <= 0.0:
        raise ValueError("Agent respawn limit must be non-negative and delay positive")
    return _agent_actions(
        serial_device,
        LaunchConfiguration("baudrate").perform(context),
        LaunchConfiguration("agent_verbosity").perform(context),
        LaunchConfiguration("namespace").perform(context),
        delay,
        limit,
        0,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_device",
                description="Required /dev/serial/by-id/<device> path",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("baudrate", default_value="921600"),
            DeclareLaunchArgument("agent_verbosity", default_value="4"),
            DeclareLaunchArgument("agent_respawn_delay", default_value="2.0"),
            DeclareLaunchArgument("agent_respawn_limit", default_value="300"),
            OpaqueFunction(function=_agent),
        ]
    )
