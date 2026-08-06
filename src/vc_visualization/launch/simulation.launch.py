"""Run isolated Gazebo vehicle simulation with the existing ROS stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_and_visualize(context):
    simulation_share = get_package_share_directory("vc_visualization")
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    robot_description_topic = (
        f"/{namespace}/robot_description" if namespace else "/robot_description"
    )
    spawn = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            "vehicle",
            "-name",
            "vehicle",
            "-z",
            "0.05",
            "-topic",
            robot_description_topic,
        ],
        output="screen",
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_share + "/launch/rviz.launch.py"),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "use_sim_time": "true",
            "simulation": "true",
            "rviz_config": simulation_share + "/config/simulation.rviz",
        }.items(),
    )
    actions = [spawn]
    if not headless:
        actions.append(rviz)
    return [TimerAction(period=2.0, actions=actions)]


def _gazebo_launch(context):
    visualization_share = get_package_share_directory("vc_visualization")
    world = visualization_share + "/worlds/vehicle.world.sdf"
    args = ["-r"]
    if LaunchConfiguration("headless").perform(context).lower() == "true":
        args.append("-s")
    args.append(world)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("ros_gz_sim")
                + "/launch/gz_sim.launch.py"
            ),
            launch_arguments={"gz_args": " ".join(args)}.items(),
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("vc_bringup")
    visualization_share = get_package_share_directory("vc_visualization")
    namespace = LaunchConfiguration("namespace")
    common = {"namespace": namespace, "use_sim_time": "true"}
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="sim"),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=bringup_share + "/config/vehicle.fake.yaml",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=get_package_share_directory("vc_safety")
                + "/config/safety.yaml",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            OpaqueFunction(function=_gazebo_launch),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    bringup_share + "/launch/state_estimation.launch.py"
                ),
                launch_arguments={
                    **common,
                    "vehicle_config": LaunchConfiguration("vehicle_config"),
                    "sim": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    bringup_share + "/launch/safety.launch.py"
                ),
                launch_arguments={
                    **common,
                    "safety_config": LaunchConfiguration("safety_config"),
                    "cmd_vel_output": "cmd_vel",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    bringup_share + "/launch/diagnostics.launch.py"
                ),
                launch_arguments={
                    **common,
                    "diagnostics_config": bringup_share + "/config/diagnostics.yaml",
                    "monitor_serial": "false",
                }.items(),
            ),
            Node(
                package="vc_visualization",
                executable="simulation_health",
                name="simulation_ecu",
                namespace=namespace,
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="simulation_bridge",
                parameters=[
                    {
                        "config_file": visualization_share
                        + "/config/bridge.yaml"
                    }
                ],
                output="screen",
            ),
            OpaqueFunction(function=_spawn_and_visualize),
        ]
    )
