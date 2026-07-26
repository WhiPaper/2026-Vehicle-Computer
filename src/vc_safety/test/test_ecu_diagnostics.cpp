#include "vc_safety/ecu_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <gtest/gtest.h>

#include <string>

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

KeyValue key_value(const std::string& key, const std::string& value)
{
    KeyValue result;
    result.key = key;
    result.value = value;
    return result;
}

DiagnosticArray healthy_diagnostics()
{
    DiagnosticStatus transport;
    transport.level = DiagnosticStatus::OK;
    transport.name = "vehicle_ecu/transport";
    transport.values = {key_value("session_state", "CONNECTED"),
                        key_value("agent_connected", "true"),
                        key_value("time_synchronized", "true"), key_value("last_error", "none")};

    DiagnosticStatus drive;
    drive.level = DiagnosticStatus::OK;
    drive.name = "vehicle_ecu/drive";
    drive.values = {key_value("calibrated", "true"),      key_value("command_active", "false"),
                    key_value("command_age_ms", "0"),     key_value("encoder_ok", "true"),
                    key_value("stalled", "false"),        key_value("motor_ok", "true"),
                    key_value("fault_mask", "0x00000000")};

    DiagnosticStatus imu;
    imu.level = DiagnosticStatus::OK;
    imu.name = "vehicle_ecu/imu";
    imu.values = {key_value("imu_ok", "true"), key_value("calibrated", "true"),
                  key_value("last_error", "none")};

    DiagnosticArray diagnostics;
    diagnostics.status = {transport, drive, imu};
    return diagnostics;
}

TEST(EcuDiagnostics, AcceptsTheCompleteHealthyContract)
{
    const auto health = vc_safety::evaluate_ecu_diagnostics(healthy_diagnostics());
    EXPECT_TRUE(health.transport_ok);
    EXPECT_TRUE(health.drive_ok);
    EXPECT_TRUE(health.imu_ok);
    EXPECT_TRUE(health.time_synchronized);
}

TEST(EcuDiagnostics, FailsClosedOnMissingOrMalformedFields)
{
    auto diagnostics = healthy_diagnostics();
    diagnostics.status[0].values.clear();
    auto health = vc_safety::evaluate_ecu_diagnostics(diagnostics);
    EXPECT_FALSE(health.transport_ok);
    EXPECT_FALSE(health.drive_ok);
    EXPECT_FALSE(health.imu_ok);
    EXPECT_FALSE(health.time_synchronized);

    diagnostics = healthy_diagnostics();
    diagnostics.status[1].values[0].value = "TRUE";
    health = vc_safety::evaluate_ecu_diagnostics(diagnostics);
    EXPECT_FALSE(health.drive_ok);
    EXPECT_TRUE(health.transport_ok);
    EXPECT_TRUE(health.imu_ok);
}

TEST(EcuDiagnostics, SeparatesTransportDriveAndImuHealth)
{
    auto diagnostics = healthy_diagnostics();
    diagnostics.status[2].level = DiagnosticStatus::ERROR;
    const auto health = vc_safety::evaluate_ecu_diagnostics(diagnostics);
    EXPECT_TRUE(health.transport_ok);
    EXPECT_TRUE(health.drive_ok);
    EXPECT_FALSE(health.imu_ok);
    EXPECT_TRUE(health.time_synchronized);
}

} // namespace
