#ifndef VC_SAFETY__SAFETY_STATE_HPP_
#define VC_SAFETY__SAFETY_STATE_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace vc_safety
{

/// Validated timing and command limits used by the fail-closed state machine.
struct SafetyConfig
{
    /// Maximum age of odometry and IMU samples.
    std::int64_t data_timeout_ns{200000000};
    /// Maximum age of the ECU diagnostics heartbeat.
    std::int64_t diagnostics_timeout_ns{500000000};
    /// Maximum age of a command request after enable.
    std::int64_t command_timeout_ns{250000000};
    /// Maximum future offset accepted for sensor timestamps.
    std::int64_t future_tolerance_ns{100000000};
    /// Maximum absolute linear.x command in metres per second.
    double max_linear_speed_mps{1.0};
    /// Maximum absolute angular.z command in radians per second.
    double max_angular_speed_rps{2.0};
};

/// The last command accepted by the gate.
struct VelocityCommand
{
    double linear_x{0.0};
    double angular_z{0.0};
};

/// Observable state of the safety gate at one decision point.
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

/// Result of evaluating the current inputs: a command or a fail-closed zero.
struct GateDecision
{
    std::optional<VelocityCommand> command;
    SafetySnapshot snapshot;
};

/// ROS-independent motion safety state machine.
class SafetyState
{
  public:
    /// Construct a state machine with validated limits.
    explicit SafetyState(SafetyConfig config = SafetyConfig{});

    /// Update the latest ECU health and synchronization state.
    void update_diagnostics(bool transport_ok, bool drive_ok, bool imu_ok, bool time_synchronized,
                            std::int64_t steady_now_ns);
    /// Validate and store an odometry timestamp.
    bool update_odom(std::int64_t stamp_ns, std::int64_t ros_now_ns, std::int64_t steady_now_ns);
    /// Validate and store an IMU timestamp.
    bool update_imu(std::int64_t stamp_ns, std::int64_t ros_now_ns, std::int64_t steady_now_ns);
    /// Store a command request received on the current motion stream.
    bool update_command(double linear_x, double angular_z, std::int64_t steady_now_ns);

    /// Enable only when all readiness and freshness conditions are satisfied.
    bool request_enable(std::int64_t ros_now_ns, std::int64_t steady_now_ns,
                        std::string* reason = nullptr);
    /// Clear the enable latch and cached command.
    void request_disable(const std::string& reason = "operator_disabled");
    /// Record an invalid input as the current block/trip reason.
    void reject_input(const std::string& reason);
    /// Evaluate freshness and return the only command that may be published.
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
