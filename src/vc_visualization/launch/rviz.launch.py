"""Launch RViz2 without starting any vehicle or control process."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_node(context):
    replay = LaunchConfiguration("replay").perform(context).lower() == "true"
    simulation = (
        LaunchConfiguration("simulation").perform(context).lower() == "true"
    )
    remappings = []
    if replay:
        namespace = LaunchConfiguration("namespace").perform(context).strip("/")
        recorded_prefix = (
            f"/{namespace}/replay/recorded"
            if namespace
            else "/replay/recorded"
        )
        remappings = [
            ("odometry/filtered", recorded_prefix + "/odometry/filtered"),
            ("/tf", "/replay/recorded/tf"),
            ("/tf_static", "/replay/recorded/tf_static"),
        ]
    elif simulation:
        namespace = LaunchConfiguration("namespace").perform(context).strip("/")
        if namespace:
            remappings = [
                ("/tf", f"/{namespace}/tf"),
                ("/tf_static", f"/{namespace}/tf_static"),
            ]
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=LaunchConfiguration("namespace"),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=remappings,
            output="screen",
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("vc_visualization")
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("replay", default_value="false"),
            DeclareLaunchArgument("simulation", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config", default_value=share + "/config/vehicle.rviz"
            ),
            OpaqueFunction(function=_rviz_node),
        ]
    )
