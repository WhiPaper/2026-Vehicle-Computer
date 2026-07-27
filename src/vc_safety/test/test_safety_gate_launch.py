import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def generate_test_description():
    safety_gate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vc_safety"), "launch", "safety.launch.py"]
            )
        )
    )
    return launch.LaunchDescription(
        [safety_gate, launch_testing.actions.ReadyToTest()]
    )


def kv(key, value):
    return KeyValue(key=key, value=value)


class TestSafetyGate(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("safety_gate_test")
        reliable = QoSProfile(
            depth=1,
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
        self.diag_pub = self.node.create_publisher(
            DiagnosticArray, "/diagnostics", reliable
        )
        self.reliable_qos = reliable
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", sensor)
        self.imu_pub = self.node.create_publisher(Imu, "/imu/data_raw", sensor)
        self.command_pub = self.node.create_publisher(
            Twist, "/cmd_vel_request", reliable
        )
        self.commands = []
        self.enabled = None
        self.command_sub = self.node.create_subscription(
            Twist, "/cmd_vel", lambda msg: self.commands.append(msg), reliable
        )
        self.enabled_sub = self.node.create_subscription(
            Bool,
            "/vehicle/motion_enabled",
            lambda msg: setattr(self, "enabled", msg.data),
            latched,
        )
        self.enable_client = self.node.create_client(
            SetBool, "/vehicle/motion_enable"
        )
        self.change_state_client = self.node.create_client(
            ChangeState, "/safety_gate/change_state"
        )
        self.get_state_client = self.node.create_client(
            GetState, "/safety_gate/get_state"
        )

    def tearDown(self):
        self.node.destroy_node()

    def publish_health(self):
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        self.odom_pub.publish(odom)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        self.imu_pub.publish(imu)

        transport = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="vehicle_ecu/transport",
            message="connected",
            values=[
                kv("session_state", "CONNECTED"),
                kv("agent_connected", "true"),
                kv("time_synchronized", "true"),
                kv("last_error", "none"),
            ],
        )
        drive = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="vehicle_ecu/drive",
            message="ready",
            values=[
                kv("calibrated", "true"),
                kv("command_active", "false"),
                kv("command_age_ms", "0"),
                kv("encoder_ok", "true"),
                kv("stalled", "false"),
                kv("motor_ok", "true"),
                kv("fault_mask", "0x00000000"),
            ],
        )
        imu_status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="vehicle_ecu/imu",
            message="ready",
            values=[
                kv("imu_ok", "true"),
                kv("calibrated", "true"),
                kv("last_error", "none"),
            ],
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        diagnostics.status = [transport, drive, imu_status]
        self.diag_pub.publish(diagnostics)

    def pump(self, duration, publish_health=True, command=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if publish_health:
                self.publish_health()
            if command is not None:
                self.command_pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def pump_until(self, predicate, timeout, publish_health=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish_health:
                self.publish_health()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return predicate()

    def set_enabled(self, enabled):
        self.assertTrue(self.enable_client.wait_for_service(timeout_sec=5.0))
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            self.publish_health()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(future.done())
        return future.result()

    def change_lifecycle_state(self, transition_id):
        self.assertTrue(
            self.change_state_client.wait_for_service(timeout_sec=5.0)
        )
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_state_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)

    def test_gate_times_out_and_never_replays_a_command(self):
        self.pump(0.4)
        response = self.set_enabled(True)
        self.assertTrue(response.success, response.message)

        requested = Twist()
        requested.linear.x = 0.2
        self.pump(0.15, command=requested)
        self.assertTrue(any(msg.linear.x == 0.2 for msg in self.commands))

        self.commands.clear()
        self.assertTrue(
            self.pump_until(lambda: self.enabled is False, timeout=1.0),
            "motion latch did not clear after the command timeout",
        )
        self.pump(0.1)
        self.assertTrue(self.commands)
        self.assertTrue(all(msg.linear.x == 0.0 for msg in self.commands[-3:]))

        response = self.set_enabled(True)
        self.assertTrue(response.success, response.message)
        self.commands.clear()
        self.pump(0.15)
        self.assertTrue(self.commands)
        self.assertTrue(all(msg.linear.x == 0.0 for msg in self.commands))

        # Replacing the ECU diagnostics publisher represents an Agent/ECU
        # restart. A new DDS writer identity must clear the latch.
        self.pump(0.15, command=requested)
        self.assertTrue(any(msg.linear.x == 0.2 for msg in self.commands))

        # The first command after enable owns the stream. A second valid
        # publisher must trip the latch instead of racing the active source.
        alternate_command_pub = self.node.create_publisher(
            Twist, "/cmd_vel_request", self.reliable_qos
        )
        self.commands.clear()
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.publish_health()
            alternate_command_pub.publish(requested)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertFalse(self.enabled)
        self.assertTrue(self.commands)
        self.assertTrue(all(msg.linear.x == 0.0 for msg in self.commands[-3:]))
        self.node.destroy_publisher(alternate_command_pub)

        response = self.set_enabled(True)
        self.assertTrue(response.success, response.message)
        self.commands.clear()
        self.pump(0.15, command=requested)
        self.assertTrue(any(msg.linear.x == 0.2 for msg in self.commands))

        self.node.destroy_publisher(self.diag_pub)
        self.diag_pub = self.node.create_publisher(
            DiagnosticArray, "/diagnostics", self.reliable_qos
        )
        self.commands.clear()
        self.pump(0.4)
        self.assertFalse(self.enabled)
        self.assertTrue(self.commands)
        self.assertTrue(all(msg.linear.x == 0.0 for msg in self.commands[-3:]))

        # Lifecycle activity controls supervision, never the motion latch.
        self.commands.clear()
        self.change_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        self.pump(0.1, publish_health=False)
        self.assertTrue(self.commands)
        self.assertEqual(self.commands[-1].linear.x, 0.0)
        self.assertEqual(self.commands[-1].angular.z, 0.0)
        response = self.set_enabled(True)
        self.assertFalse(response.success)
        self.assertEqual(response.message, "lifecycle_not_active")
