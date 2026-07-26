"""Deterministic ROS-side stand-in for the micro-ROS vehicle ECU."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState


def _kv(key, value):
    return KeyValue(key=key, value=str(value).lower() if isinstance(value, bool) else str(value))


class FakeEcu(Node):
    def __init__(self):
        super().__init__("vehicle_ecu")
        self.declare_parameter("healthy", True)
        self.declare_parameter("time_synchronized", True)
        self.declare_parameter("stamp_offset_ms", 0)
        self.declare_parameter("linear_velocity", 0.0)
        self.declare_parameter("angular_velocity", 0.0)

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_publisher = self.create_publisher(Odometry, "odom", sensor_qos)
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", sensor_qos)
        self.joint_publisher = self.create_publisher(
            JointState, "joint_states", sensor_qos
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", reliable_qos
        )
        self.position = 0.0
        self.create_timer(1.0 / 30.0, self.publish_odom_and_joints)
        self.create_timer(1.0 / 50.0, self.publish_imu)
        self.create_timer(1.0 / 5.0, self.publish_diagnostics)

    def stamp(self):
        offset_ns = self.get_parameter("stamp_offset_ms").value * 1_000_000
        return (self.get_clock().now() + Duration(nanoseconds=offset_ns)).to_msg()

    def publish_odom_and_joints(self):
        stamp = self.stamp()
        linear = float(self.get_parameter("linear_velocity").value)
        angular = float(self.get_parameter("angular_velocity").value)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_publisher.publish(odom)

        self.position += linear / 30.0
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["left_wheel_joint", "right_wheel_joint"]
        joints.position = [self.position, self.position]
        joints.velocity = [linear, linear]
        self.joint_publisher.publish(joints)

    def publish_imu(self):
        imu = Imu()
        imu.header.stamp = self.stamp()
        imu.header.frame_id = "imu_link"
        imu.angular_velocity.z = float(self.get_parameter("angular_velocity").value)
        imu.orientation_covariance[0] = -1.0
        self.imu_publisher.publish(imu)

    def publish_diagnostics(self):
        healthy = bool(self.get_parameter("healthy").value)
        synchronized = bool(self.get_parameter("time_synchronized").value)
        level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        transport = DiagnosticStatus(
            level=level,
            name="vehicle_ecu/transport",
            message="connected" if healthy else "fault",
            values=[
                _kv("session_state", "CONNECTED" if healthy else "DISCONNECTED"),
                _kv("agent_connected", healthy),
                _kv("time_synchronized", synchronized),
                _kv("last_error", "none" if healthy else "injected"),
            ],
        )
        drive = DiagnosticStatus(
            level=level,
            name="vehicle_ecu/drive",
            message="ready" if healthy else "fault",
            values=[
                _kv("calibrated", True),
                _kv("command_active", False),
                _kv("command_age_ms", 0),
                _kv("encoder_ok", healthy),
                _kv("stalled", False),
                _kv("motor_ok", healthy),
                _kv("fault_mask", "0x00000000" if healthy else "0x00000001"),
            ],
        )
        imu = DiagnosticStatus(
            level=level,
            name="vehicle_ecu/imu",
            message="ready" if healthy else "fault",
            values=[
                _kv("imu_ok", healthy),
                _kv("calibrated", healthy),
                _kv("last_error", "none" if healthy else "injected"),
            ],
        )
        message = DiagnosticArray()
        message.header.stamp = self.stamp()
        message.status = [transport, drive, imu]
        self.diagnostics_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FakeEcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
