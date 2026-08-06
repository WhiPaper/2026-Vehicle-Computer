"""Launch and automatically activate the managed vehicle safety gate."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

from vc_safety.safety_config import load_safety_config


def _validate_safety_config(context):
    load_safety_config(LaunchConfiguration("safety_config").perform(context))
    return []


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    auto_start = LaunchConfiguration("auto_start")
    config = LaunchConfiguration("safety_config")
    cmd_vel_output = LaunchConfiguration("cmd_vel_output")

    safety_gate = LifecycleNode(
        package="vc_safety",
        executable="safety_gate_node",
        name="safety_gate",
        namespace=namespace,
        parameters=[config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("cmd_vel", cmd_vel_output)],
        output="screen",
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(safety_gate),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(safety_gate),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Safety gate namespace"
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Configure and activate the lifecycle gate",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            DeclareLaunchArgument(
                "cmd_vel_output",
                default_value="cmd_vel",
                description="Relative gated command output name",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vc_safety"), "config", "safety.yaml"]
                ),
                description="Validated safety gate parameter YAML",
            ),
            OpaqueFunction(function=_validate_safety_config),
            safety_gate,
            RegisterEventHandler(
                event_handler=OnStateTransition(
                    target_lifecycle_node=safety_gate,
                    goal_state="inactive",
                    entities=[activate],
                    handle_once=True,
                ),
                condition=IfCondition(auto_start),
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=safety_gate, on_start=[configure]
                ),
                condition=IfCondition(auto_start),
            ),
        ]
    )
