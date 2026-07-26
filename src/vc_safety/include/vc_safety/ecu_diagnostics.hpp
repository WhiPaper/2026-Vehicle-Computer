#ifndef VC_SAFETY__ECU_DIAGNOSTICS_HPP_
#define VC_SAFETY__ECU_DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace vc_safety
{

struct EcuHealth
{
    bool transport_ok{false};
    bool drive_ok{false};
    bool imu_ok{false};
    bool time_synchronized{false};
};

EcuHealth evaluate_ecu_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& diagnostics);

} // namespace vc_safety

#endif // VC_SAFETY__ECU_DIAGNOSTICS_HPP_
