#ifndef VC_SAFETY__SAFETY_STATE_HPP_
#define VC_SAFETY__SAFETY_STATE_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace vc_safety
{

struct SafetyConfig
{
    std::int64_t data_timeout_ns{200000000};
    std::int64_t diagnostics_timeout_ns{500000000};
    std::int64_t command_timeout_ns{250000000};
    std::int64_t future_tolerance_ns{100000000};
    double max_linear_speed_mps{1.0};
    double max_angular_speed_rps{2.0};
};

struct VelocityCommand
{
    double linear_x{0.0};
    double angular_z{0.0};
};

struct SafetySnapshot
{
    bool enabled{false};
    bool ready{false};
    bool transport_ok{false};
    bool drive_ok{false};
    bool imu_ok{false};
    bool time_synchronized{false};
    std::string block_reason{"diagnostics_missing"};
    std::string last_trip_reason{"startup"};
    std::uint64_t trip_count{0};
    std::int64_t odom_age_ms{-1};
    std::int64_t imu_age_ms{-1};
    std::int64_t command_age_ms{-1};
};

struct GateDecision
{
    std::optional<VelocityCommand> command;
    SafetySnapshot snapshot;
};

class SafetyState
{
  public:
    explicit SafetyState(SafetyConfig config = SafetyConfig{});

    void update_diagnostics(bool transport_ok, bool drive_ok, bool imu_ok, bool time_synchronized,
                            std::int64_t steady_now_ns);
    bool update_odom(std::int64_t stamp_ns, std::int64_t ros_now_ns, std::int64_t steady_now_ns);
    bool update_imu(std::int64_t stamp_ns, std::int64_t ros_now_ns, std::int64_t steady_now_ns);
    bool update_command(double linear_x, double angular_z, std::int64_t steady_now_ns);

    bool request_enable(std::int64_t ros_now_ns, std::int64_t steady_now_ns,
                        std::string* reason = nullptr);
    void request_disable(const std::string& reason = "operator_disabled");
    void reject_input(const std::string& reason);
    GateDecision evaluate(std::int64_t ros_now_ns, std::int64_t steady_now_ns);

  private:
    struct SensorState
    {
        bool received{false};
        std::int64_t stamp_ns{0};
        std::int64_t received_steady_ns{0};
    };

    bool update_sensor(SensorState& sensor, const char* name, std::int64_t stamp_ns,
                       std::int64_t ros_now_ns, std::int64_t steady_now_ns);
    std::string readiness_reason(std::int64_t ros_now_ns, std::int64_t steady_now_ns) const;
    static std::int64_t age_ms(bool received, std::int64_t now_ns, std::int64_t then_ns);
    void disarm(const std::string& reason);

    SafetyConfig config_;
    bool diagnostics_received_{false};
    bool transport_ok_{false};
    bool drive_ok_{false};
    bool imu_ok_{false};
    bool time_synchronized_{false};
    std::int64_t diagnostics_steady_ns_{0};
    SensorState odom_;
    SensorState imu_;

    bool enabled_{false};
    std::int64_t enabled_steady_ns_{0};
    bool command_received_{false};
    VelocityCommand command_;
    std::int64_t command_steady_ns_{0};
    std::string last_trip_reason_{"startup"};
    std::uint64_t trip_count_{0};
};

} // namespace vc_safety

#endif // VC_SAFETY__SAFETY_STATE_HPP_
