"""Launch the authoritative vehicle model and odom-to-base state estimator."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from vc_bringup.config_loader import load_vehicle_config


def _state_nodes(context):
    vehicle_config_path = LaunchConfiguration("vehicle_config").perform(context)
    config = load_vehicle_config(vehicle_config_path)
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_share = get_package_share_directory("vc_description")
    bringup_share = get_package_share_directory("vc_bringup")
    xacro_file = description_share + "/urdf/vehicle.urdf.xacro"

    dimensions = config["vehicle"]
    imu = config["imu"]
    xacro_arguments = {
        **dimensions,
        "imu_x": imu["x"],
        "imu_y": imu["y"],
        "imu_z": imu["z"],
        "imu_roll": imu["roll"],
        "imu_pitch": imu["pitch"],
        "imu_yaw": imu["yaw"],
    }
    command = ["xacro ", xacro_file]
    for key, value in xacro_arguments.items():
        command.extend([f" {key}:=", str(value)])
    robot_description = ParameterValue(Command(command), value_type=str)

    state_container = ComposableNodeContainer(
        name="vehicle_state_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                namespace=namespace,
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "",
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        ],
        output="screen",
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=namespace,
        parameters=[
            bringup_share + "/config/ekf.yaml",
            {
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "odom",
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )
    return [state_container, ekf]


def generate_launch_description():
    bringup_share = get_package_share_directory("vc_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=bringup_share + "/config/vehicle.yaml",
            ),
            OpaqueFunction(function=_state_nodes),
        ]
    )
