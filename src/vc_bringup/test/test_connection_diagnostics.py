from diagnostic_msgs.msg import DiagnosticStatus

from vc_bringup.diagnostics import ecu_heartbeat_state, serial_device_state


def test_missing_serial_device_is_error(tmp_path):
    level, message, values = serial_device_state(str(tmp_path / "missing"))

    assert level == DiagnosticStatus.ERROR
    assert message == "serial device missing"
    assert values[1].value == "unavailable"


def test_accessible_serial_device_is_ready(tmp_path):
    device = tmp_path / "serial"
    device.touch()

    level, message, _values = serial_device_state(str(device))

    assert level == DiagnosticStatus.OK
    assert message == "serial device ready"


def test_ecu_heartbeat_states():
    level, message, _values = ecu_heartbeat_state(None, 2.5, False)
    assert level == DiagnosticStatus.ERROR
    assert message == "ECU diagnostics never received"

    level, message, _values = ecu_heartbeat_state(3.0, 2.5, True)
    assert level == DiagnosticStatus.ERROR
    assert message == "ECU diagnostics stale"

    level, message, _values = ecu_heartbeat_state(0.1, 2.5, False)
    assert level == DiagnosticStatus.WARN
    assert message == "ECU heartbeat present but node hidden"

    level, message, _values = ecu_heartbeat_state(0.1, 2.5, True)
    assert level == DiagnosticStatus.OK
    assert message == "ECU connected"
