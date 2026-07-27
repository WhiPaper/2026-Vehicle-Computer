import os
import signal
import statistics
import subprocess
import tempfile
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage


def spin_until(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def call(node, client, request, timeout=5.0):
    assert client.wait_for_service(timeout_sec=timeout)
    future = client.call_async(request)
    assert spin_until(node, future.done, timeout)
    return future.result()


def median_message_rate(messages):
    stamps = [
        message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        for message in messages
    ]
    periods = [
        current - previous
        for previous, current in zip(stamps, stamps[1:])
        if current > previous
    ]
    assert periods
    return 1.0 / statistics.median(periods)


def test_fake_stack_uses_namespaced_ros_contract_and_fails_closed():
    namespace = "ci_vehicle"
    domain_id = str(120 + os.getpid() % 80)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = domain_id
    environment["ROS2CLI_DISABLE_DAEMON"] = "1"

    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = domain_id
    log = tempfile.TemporaryFile(mode="w+")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "vc_bringup",
            "fake_ecu.launch.py",
            f"namespace:={namespace}",
        ],
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )

    rclpy.init()
    node = Node("fake_stack_test")
    reliable = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    sensor = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    filtered = []
    safety_status = []
    aggregated = []
    enabled = []
    commands = []
    raw_odom = []
    raw_imu = []
    raw_diagnostics = []
    odom_tf_count = 0

    def on_tf(message):
        nonlocal odom_tf_count
        odom_tf_count += sum(
            transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
            for transform in message.transforms
        )

    subscriptions = [
        node.create_subscription(
            Odometry,
            f"/{namespace}/odometry/filtered",
            filtered.append,
            reliable,
        ),
        node.create_subscription(
            Odometry, f"/{namespace}/odom", raw_odom.append, sensor
        ),
        node.create_subscription(
            Imu, f"/{namespace}/imu/data_raw", raw_imu.append, sensor
        ),
        node.create_subscription(
            DiagnosticArray,
            f"/{namespace}/diagnostics",
            raw_diagnostics.append,
            reliable,
        ),
        node.create_subscription(
            DiagnosticArray,
            f"/{namespace}/vehicle/safety/diagnostics",
            safety_status.append,
            reliable,
        ),
        node.create_subscription(
            DiagnosticArray,
            f"/{namespace}/vehicle/diagnostics",
            aggregated.append,
            reliable,
        ),
        node.create_subscription(
            Bool,
            f"/{namespace}/vehicle/motion_enabled",
            lambda message: enabled.append(message.data),
            latched,
        ),
        node.create_subscription(
            Twist, f"/{namespace}/cmd_vel", commands.append, reliable
        ),
        node.create_subscription(TFMessage, "/tf", on_tf, 100),
    ]
    command_publisher = node.create_publisher(
        Twist, f"/{namespace}/cmd_vel_request", reliable
    )
    lifecycle_client = node.create_client(
        GetState, f"/{namespace}/safety_gate/get_state"
    )
    enable_client = node.create_client(
        SetBool, f"/{namespace}/vehicle/motion_enable"
    )

    try:
        assert lifecycle_client.wait_for_service(timeout_sec=10.0)
        lifecycle_active = False
        lifecycle_deadline = time.monotonic() + 10.0
        while time.monotonic() < lifecycle_deadline and not lifecycle_active:
            state = call(node, lifecycle_client, GetState.Request(), timeout=2.0)
            lifecycle_active = (
                state.current_state.id == State.PRIMARY_STATE_ACTIVE
            )
            rclpy.spin_once(node, timeout_sec=0.05)
        assert lifecycle_active
        assert spin_until(
            node,
            lambda: filtered
            and odom_tf_count > 0
            and any(
                any(
                    value.key == "ready" and value.value == "true"
                    for value in status.values
                )
                for array in safety_status
                for status in array.status
            )
            and any(
                status.name.endswith("/Vehicle/Computer/SafetyGate")
                for array in aggregated
                for status in array.status
            ),
            timeout=12.0,
        )

        enable_request = SetBool.Request()
        enable_request.data = True
        response = call(node, enable_client, enable_request)
        assert response.success, response.message
        assert spin_until(node, lambda: enabled and enabled[-1], timeout=2.0)
        command = Twist()
        command.linear.x = 0.15
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            command_publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.03)
        assert any(message.linear.x == 0.15 for message in commands)

        # An incompatible diagnostics writer must be observable and fail closed.
        incompatible_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        incompatible_publisher = node.create_publisher(
            DiagnosticArray, f"/{namespace}/diagnostics", incompatible_qos
        )
        assert spin_until(node, lambda: enabled and not enabled[-1], timeout=5.0)
        assert spin_until(
            node,
            lambda: commands and commands[-1].linear.x == 0.0,
            timeout=2.0,
        )
        node.destroy_publisher(incompatible_publisher)

        assert node.count_publishers("/cmd_vel") == 0
        topic_names = dict(node.get_topic_names_and_types())
        assert f"/{namespace}/vehicle/safety/statistics" in topic_names
        assert f"/{namespace}/vehicle/diagnostics" in topic_names
        assert spin_until(
            node,
            lambda: len(raw_odom) >= 10
            and len(raw_imu) >= 10
            and len(raw_diagnostics) >= 3,
            timeout=5.0,
        ), (
            "insufficient rate samples: "
            f"odom={len(raw_odom)}, imu={len(raw_imu)}, "
            f"diagnostics={len(raw_diagnostics)}"
        )
        assert 25.0 <= median_message_rate(raw_odom) <= 35.0
        assert 40.0 <= median_message_rate(raw_imu) <= 60.0
        assert 4.0 <= median_message_rate(raw_diagnostics) <= 6.0
    except Exception as error:
        log.seek(0)
        pytest.fail(f"{error!r}\n\nLaunch output:\n{log.read()}", pytrace=True)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3.0)
        log.close()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
