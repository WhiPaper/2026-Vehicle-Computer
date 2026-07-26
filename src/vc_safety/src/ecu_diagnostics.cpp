#include "vc_safety/ecu_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <initializer_list>
#include <optional>
#include <string>

namespace vc_safety
{
namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

std::optional<std::string> value_for(const DiagnosticStatus& status, const std::string& key)
{
    for (const auto& value : status.values)
    {
        if (value.key == key)
        {
            return value.value;
        }
    }
    return std::nullopt;
}

std::optional<bool> strict_bool(const DiagnosticStatus& status, const std::string& key)
{
    const auto value = value_for(status, key);
    if (value == "true")
    {
        return true;
    }
    if (value == "false")
    {
        return false;
    }
    return std::nullopt;
}

bool has_keys(const DiagnosticStatus& status, std::initializer_list<const char*> keys)
{
    for (const auto* key : keys)
    {
        if (!value_for(status, key).has_value())
        {
            return false;
        }
    }
    return true;
}

const DiagnosticStatus* find_status(const diagnostic_msgs::msg::DiagnosticArray& diagnostics,
                                    const std::string& name)
{
    const DiagnosticStatus* match = nullptr;
    for (const auto& status : diagnostics.status)
    {
        if (status.name == name)
        {
            match = &status;
        }
    }
    return match;
}

bool valid_schema(const DiagnosticStatus& transport, const DiagnosticStatus& drive,
                  const DiagnosticStatus& imu)
{
    return has_keys(transport,
                    {"session_state", "agent_connected", "time_synchronized", "last_error"}) &&
           has_keys(drive, {"calibrated", "command_active", "command_age_ms", "encoder_ok",
                            "stalled", "motor_ok", "fault_mask"}) &&
           has_keys(imu, {"imu_ok", "calibrated", "last_error"});
}

} // namespace

EcuHealth evaluate_ecu_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& diagnostics)
{
    const auto* transport = find_status(diagnostics, "vehicle_ecu/transport");
    const auto* drive = find_status(diagnostics, "vehicle_ecu/drive");
    const auto* imu = find_status(diagnostics, "vehicle_ecu/imu");
    if (transport == nullptr || drive == nullptr || imu == nullptr ||
        !valid_schema(*transport, *drive, *imu))
    {
        return {};
    }

    EcuHealth health;
    health.time_synchronized = strict_bool(*transport, "time_synchronized").value_or(false);
    health.transport_ok = transport->level == DiagnosticStatus::OK &&
                          value_for(*transport, "session_state") == "CONNECTED" &&
                          strict_bool(*transport, "agent_connected").value_or(false) &&
                          health.time_synchronized;
    health.drive_ok = drive->level == DiagnosticStatus::OK &&
                      strict_bool(*drive, "calibrated").value_or(false) &&
                      strict_bool(*drive, "encoder_ok").value_or(false) &&
                      !strict_bool(*drive, "stalled").value_or(true) &&
                      strict_bool(*drive, "motor_ok").value_or(false);
    health.imu_ok = imu->level == DiagnosticStatus::OK &&
                    strict_bool(*imu, "imu_ok").value_or(false) &&
                    strict_bool(*imu, "calibrated").value_or(false);
    return health;
}

} // namespace vc_safety
