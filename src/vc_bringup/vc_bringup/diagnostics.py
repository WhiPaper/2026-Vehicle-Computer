"""ROS diagnostics adapters that remain outside the safety decision path."""

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_updater import (
    FrequencyStatusParam,
    TimeStampStatusParam,
    TopicDiagnostic,
    Updater,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class StateEstimationMonitor(Node):

    def __init__(self):
        super().__init__("state_estimation_monitor")
        expected_rate = 15.0
        min_rate = expected_rate * 0.8
        max_rate = expected_rate * 1.2
        self.updater = Updater(self, period=1.0)
        self.updater.setHardwareID("rpi5-vehicle-computer")
        self.topic_diagnostic = TopicDiagnostic(
            "odometry/filtered",
            self.updater,
            FrequencyStatusParam({"min": min_rate, "max": max_rate}, 0.1, 30),
            TimeStampStatusParam(-0.1, 0.2),
        )
        self.subscription = self.create_subscription(
            Odometry, "odometry/filtered", self.on_odometry, 10
        )

    def on_odometry(self, message):
        stamp = message.header.stamp
        self.topic_diagnostic.tick(stamp.sec + stamp.nanosec / 1e9)


class DiagnosticsMux(Node):

    def __init__(self):
        super().__init__("vehicle_diagnostics_mux")
        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            DiagnosticArray, "vehicle/diagnostics_input", reliable
        )
        self._diagnostic_subscriptions = [
            self.create_subscription(
                DiagnosticArray, "diagnostics", self.publisher.publish, reliable
            ),
            self.create_subscription(
                DiagnosticArray,
                "vehicle/safety/diagnostics",
                self.publisher.publish,
                reliable,
            ),
            self.create_subscription(
                DiagnosticArray,
                "computer/diagnostics",
                self.publisher.publish,
                reliable,
            ),
        ]


def _spin(node_type, args=None):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def state_estimation_monitor_main(args=None):
    _spin(StateEstimationMonitor, args)


def diagnostics_mux_main(args=None):
    _spin(DiagnosticsMux, args)
