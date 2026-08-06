"""Publish the ECU diagnostic contract for the isolated Gazebo vehicle."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _value(key, value):
    if isinstance(value, bool):
        value = str(value).lower()
    return KeyValue(key=key, value=str(value))


class SimulationHealth(Node):
    """Provide a deterministic healthy ECU facade to the safety gate."""

    def __init__(self):
        super().__init__("simulation_ecu")
        self.declare_parameter("healthy", True)
        self.declare_parameter("time_synchronized", True)
        self.declare_parameter("publish_rate_hz", 5.0)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(DiagnosticArray, "diagnostics", qos)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate, self.publish_diagnostics)

    def publish_diagnostics(self):
        healthy = bool(self.get_parameter("healthy").value)
        synchronized = bool(self.get_parameter("time_synchronized").value)
        level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        transport = DiagnosticStatus(
            level=level,
            name="vehicle_ecu/transport",
            message="connected" if healthy else "fault",
            values=[
                _value("session_state", "CONNECTED" if healthy else "DISCONNECTED"),
                _value("agent_connected", healthy),
                _value("time_synchronized", synchronized),
                _value("last_error", "none" if healthy else "injected"),
            ],
        )
        drive = DiagnosticStatus(
            level=level,
            name="vehicle_ecu/drive",
            message="ready" if healthy else "fault",
            values=[
                _value("calibrated", True),
                _value("command_active", False),
                _value("command_age_ms", 0),
                _value("encoder_ok", healthy),
                _value("stalled", False),
                _value("motor_ok", healthy),
                _value("fault_mask", "0x00000000" if healthy else "0x00000001"),
            ],
        )
        imu = DiagnosticStatus(
            level=level,
            name="vehicle_ecu/imu",
            message="ready" if healthy else "fault",
            values=[
                _value("imu_ok", healthy),
                _value("calibrated", healthy),
                _value("last_error", "none" if healthy else "injected"),
            ],
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [transport, drive, imu]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
