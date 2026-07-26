#include "vc_safety/safety_state.hpp"

#include <cmath>
#include <utility>

namespace vc_safety
{

SafetyState::SafetyState(SafetyConfig config) : config_(std::move(config)) {}

void SafetyState::disarm(const std::string& reason)
{
    const bool was_enabled = enabled_;
    if (was_enabled)
    {
        ++trip_count_;
        last_trip_reason_ = reason;
    }
    enabled_ = false;
    command_received_ = false;
    command_ = VelocityCommand{};
}

void SafetyState::update_diagnostics(bool transport_ok, bool drive_ok, bool imu_ok,
                                     bool time_synchronized, std::int64_t steady_now_ns)
{
    diagnostics_received_ = true;
    diagnostics_steady_ns_ = steady_now_ns;
    transport_ok_ = transport_ok;
    drive_ok_ = drive_ok;
    imu_ok_ = imu_ok;
    time_synchronized_ = time_synchronized;

    if (!transport_ok_)
    {
        disarm("transport_not_ready");
    }
    else if (!time_synchronized_)
    {
        disarm("time_not_synchronized");
    }
    else if (!drive_ok_)
    {
        disarm("drive_not_ready");
    }
    else if (!imu_ok_)
    {
        disarm("imu_not_ready");
    }
}

bool SafetyState::update_sensor(SensorState& sensor, const char* name, std::int64_t stamp_ns,
                                std::int64_t ros_now_ns, std::int64_t steady_now_ns)
{
    std::string reason{name};
    if (stamp_ns <= 0)
    {
        disarm(reason + "_stamp_zero");
        return false;
    }
    if (stamp_ns > ros_now_ns + config_.future_tolerance_ns)
    {
        disarm(reason + "_stamp_future");
        return false;
    }
    if (ros_now_ns - stamp_ns > config_.data_timeout_ns)
    {
        disarm(reason + "_stamp_stale");
        return false;
    }
    if (sensor.received && stamp_ns <= sensor.stamp_ns)
    {
        disarm(reason + "_stamp_non_monotonic");
        return false;
    }

    sensor.received = true;
    sensor.stamp_ns = stamp_ns;
    sensor.received_steady_ns = steady_now_ns;
    return true;
}

bool SafetyState::update_odom(std::int64_t stamp_ns, std::int64_t ros_now_ns,
                              std::int64_t steady_now_ns)
{
    return update_sensor(odom_, "odom", stamp_ns, ros_now_ns, steady_now_ns);
}

bool SafetyState::update_imu(std::int64_t stamp_ns, std::int64_t ros_now_ns,
                             std::int64_t steady_now_ns)
{
    return update_sensor(imu_, "imu", stamp_ns, ros_now_ns, steady_now_ns);
}

bool SafetyState::update_command(double linear_x, double angular_z, std::int64_t steady_now_ns)
{
    if (!std::isfinite(linear_x) || !std::isfinite(angular_z))
    {
        disarm("command_not_finite");
        return false;
    }
    if (std::abs(linear_x) > config_.max_linear_speed_mps ||
        std::abs(angular_z) > config_.max_angular_speed_rps)
    {
        disarm("command_limit_exceeded");
        return false;
    }
    if (!enabled_)
    {
        return false;
    }

    command_ = VelocityCommand{linear_x, angular_z};
    command_received_ = true;
    command_steady_ns_ = steady_now_ns;
    return true;
}

std::string SafetyState::readiness_reason(std::int64_t ros_now_ns, std::int64_t steady_now_ns) const
{
    if (!diagnostics_received_)
    {
        return "diagnostics_missing";
    }
    if (steady_now_ns - diagnostics_steady_ns_ > config_.diagnostics_timeout_ns)
    {
        return "diagnostics_timeout";
    }
    if (!transport_ok_)
    {
        return "transport_not_ready";
    }
    if (!time_synchronized_)
    {
        return "time_not_synchronized";
    }
    if (!drive_ok_)
    {
        return "drive_not_ready";
    }
    if (!imu_ok_)
    {
        return "imu_not_ready";
    }

    const SensorState* sensors[] = {&odom_, &imu_};
    const char* names[] = {"odom", "imu"};
    for (std::size_t index = 0; index < 2; ++index)
    {
        const auto& sensor = *sensors[index];
        const std::string name{names[index]};
        if (!sensor.received)
        {
            return name + "_missing";
        }
        if (steady_now_ns - sensor.received_steady_ns > config_.data_timeout_ns)
        {
            return name + "_receive_timeout";
        }
        if (sensor.stamp_ns > ros_now_ns + config_.future_tolerance_ns)
        {
            return name + "_stamp_future";
        }
        if (ros_now_ns - sensor.stamp_ns > config_.data_timeout_ns)
        {
            return name + "_stamp_stale";
        }
    }
    return "none";
}

bool SafetyState::request_enable(std::int64_t ros_now_ns, std::int64_t steady_now_ns,
                                 std::string* reason)
{
    const std::string not_ready = readiness_reason(ros_now_ns, steady_now_ns);
    if (not_ready != "none")
    {
        disarm(not_ready);
        if (reason != nullptr)
        {
            *reason = not_ready;
        }
        return false;
    }

    enabled_ = true;
    enabled_steady_ns_ = steady_now_ns;
    command_received_ = false;
    command_ = VelocityCommand{};
    if (reason != nullptr)
    {
        *reason = "enabled";
    }
    return true;
}

void SafetyState::request_disable(const std::string& reason) { disarm(reason); }

void SafetyState::reject_input(const std::string& reason) { disarm(reason); }

std::int64_t SafetyState::age_ms(bool received, std::int64_t now_ns, std::int64_t then_ns)
{
    return received ? (now_ns - then_ns) / 1000000 : -1;
}

GateDecision SafetyState::evaluate(std::int64_t ros_now_ns, std::int64_t steady_now_ns)
{
    const std::string not_ready = readiness_reason(ros_now_ns, steady_now_ns);
    const bool ready = not_ready == "none";
    if (!ready && enabled_)
    {
        disarm(not_ready);
    }

    if (enabled_)
    {
        const std::int64_t command_reference =
            command_received_ ? command_steady_ns_ : enabled_steady_ns_;
        if (steady_now_ns - command_reference > config_.command_timeout_ns)
        {
            disarm(command_received_ ? "command_timeout" : "command_not_received");
        }
    }

    GateDecision decision;
    if (enabled_ && command_received_)
    {
        decision.command = command_;
    }
    decision.snapshot.enabled = enabled_;
    decision.snapshot.ready = ready;
    decision.snapshot.transport_ok = transport_ok_;
    decision.snapshot.drive_ok = drive_ok_;
    decision.snapshot.imu_ok = imu_ok_;
    decision.snapshot.time_synchronized = time_synchronized_;
    decision.snapshot.block_reason =
        !ready ? not_ready
               : (enabled_ && !command_received_ ? "command_pending"
                                                 : (enabled_ ? "none" : "operator_disabled"));
    decision.snapshot.last_trip_reason = last_trip_reason_;
    decision.snapshot.trip_count = trip_count_;
    decision.snapshot.odom_age_ms = age_ms(odom_.received, steady_now_ns, odom_.received_steady_ns);
    decision.snapshot.imu_age_ms = age_ms(imu_.received, steady_now_ns, imu_.received_steady_ns);
    decision.snapshot.command_age_ms = age_ms(command_received_, steady_now_ns, command_steady_ns_);
    return decision;
}

} // namespace vc_safety
