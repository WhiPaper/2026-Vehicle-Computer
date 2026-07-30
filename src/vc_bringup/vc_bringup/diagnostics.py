"""ROS diagnostics adapters that remain outside the safety decision path."""

import os
from pathlib import Path
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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


HARDWARE_ID_COMPUTER = "rpi5-vehicle-computer"
HARDWARE_ID_ECU = "esp32-vehicle-ecu"
DIAGNOSTICS_INPUT_TOPIC = "vehicle/diagnostics_input"
ECU_DIAGNOSTICS_TOPIC = "diagnostics"


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _key_value(key: str, value: object) -> KeyValue:
    return KeyValue(key=key, value=str(value))


def serial_device_state(path: str) -> tuple[int, str, list[KeyValue]]:
    """Return level, message, and diagnostic values for a serial device."""
    device = Path(path)
    values = [_key_value("configured_path", path)]
    if not device.exists():
        values.append(_key_value("resolved_path", "unavailable"))
        return DiagnosticStatus.ERROR, "serial device missing", values

    values.append(_key_value("resolved_path", device.resolve()))
    readable = os.access(path, os.R_OK)
    writable = os.access(path, os.W_OK)
    values.extend(
        [
            _key_value("readable", str(readable).lower()),
            _key_value("writable", str(writable).lower()),
        ]
    )
    if not readable or not writable:
        return DiagnosticStatus.ERROR, "serial device permission denied", values
    return DiagnosticStatus.OK, "serial device ready", values


def ecu_heartbeat_state(
    age_seconds: Optional[float],
    timeout_seconds: float,
    node_visible: bool,
) -> tuple[int, str, list[KeyValue]]:
    """Return a diagnostic state for the ECU graph and heartbeat."""
    values = [
        _key_value("node_visible", str(node_visible).lower()),
        _key_value(
            "diagnostic_age_s",
            "never" if age_seconds is None else f"{age_seconds:.3f}",
        ),
        _key_value("heartbeat_timeout_s", f"{timeout_seconds:.3f}"),
    ]
    if age_seconds is None:
        return DiagnosticStatus.ERROR, "ECU diagnostics never received", values
    if age_seconds > timeout_seconds:
        return DiagnosticStatus.ERROR, "ECU diagnostics stale", values
    if not node_visible:
        return DiagnosticStatus.WARN, "ECU heartbeat present but node hidden", values
    return DiagnosticStatus.OK, "ECU connected", values


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
        reliable = _reliable_qos()
        self.publisher = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_INPUT_TOPIC, reliable
        )
        self._diagnostic_subscriptions = [
            self.create_subscription(
                DiagnosticArray,
                ECU_DIAGNOSTICS_TOPIC,
                self.publisher.publish,
                reliable,
            ),
            self.create_subscription(
                DiagnosticArray,
                "vehicle/safety/diagnostics",
                self.publisher.publish,
                reliable,
            ),
        ]


class SerialConnectionMonitor(Node):

    def __init__(self):
        super().__init__("serial_connection_monitor")
        self.declare_parameter("serial_device", "")
        self.declare_parameter("ecu_heartbeat_timeout", 2.5)
        self.serial_device = (
            self.get_parameter("serial_device").get_parameter_value().string_value
        )
        self.heartbeat_timeout = (
            self.get_parameter("ecu_heartbeat_timeout")
            .get_parameter_value()
            .double_value
        )
        reliable = _reliable_qos()
        self.publisher = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_INPUT_TOPIC, reliable
        )
        self.subscription = self.create_subscription(
            DiagnosticArray,
            ECU_DIAGNOSTICS_TOPIC,
            self.on_ecu_diagnostics,
            reliable,
        )
        self.last_ecu_diagnostic = None
        self.timer = self.create_timer(1.0, self.publish_status)

    def on_ecu_diagnostics(self, _message):
        self.last_ecu_diagnostic = time.monotonic()

    def publish_status(self):
        now = time.monotonic()
        age = (
            None
            if self.last_ecu_diagnostic is None
            else now - self.last_ecu_diagnostic
        )
        node_visible = any(
            name == "vehicle_ecu"
            for name, _namespace in self.get_node_names_and_namespaces()
        )
        serial_level, serial_message, serial_values = serial_device_state(
            self.serial_device
        )
        ecu_level, ecu_message, ecu_values = ecu_heartbeat_state(
            age, self.heartbeat_timeout, node_visible
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            DiagnosticStatus(
                level=serial_level,
                name="vehicle_computer/serial_device",
                message=serial_message,
                hardware_id=HARDWARE_ID_COMPUTER,
                values=serial_values,
            ),
            DiagnosticStatus(
                level=ecu_level,
                name="vehicle_computer/ecu_connection",
                message=ecu_message,
                hardware_id=HARDWARE_ID_ECU,
                values=ecu_values,
            ),
        ]
        self.publisher.publish(message)


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


def serial_connection_monitor_main(args=None):
    _spin(SerialConnectionMonitor, args)
