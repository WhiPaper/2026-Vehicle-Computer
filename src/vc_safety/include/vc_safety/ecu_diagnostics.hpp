#ifndef VC_SAFETY__ECU_DIAGNOSTICS_HPP_
#define VC_SAFETY__ECU_DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace vc_safety
{

/// Health flags extracted from the ECU diagnostics contract.
struct EcuHealth
{
    bool transport_ok{false};
    bool drive_ok{false};
    bool imu_ok{false};
    bool time_synchronized{false};
};

/// Fail closed when a required ECU status or key is missing or malformed.
EcuHealth evaluate_ecu_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& diagnostics);

} // namespace vc_safety

#endif // VC_SAFETY__ECU_DIAGNOSTICS_HPP_
