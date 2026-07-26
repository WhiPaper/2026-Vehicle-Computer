#include "vc_safety/safety_state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace
{

using vc_safety::SafetyConfig;
using vc_safety::SafetyState;

constexpr std::int64_t kMs = 1000000;
constexpr std::int64_t kRosStart = 10000000000;
constexpr std::int64_t kSteadyStart = 5000000000;

void make_ready(SafetyState& state)
{
    state.update_diagnostics(true, true, true, true, kSteadyStart);
    ASSERT_TRUE(state.update_odom(kRosStart, kRosStart, kSteadyStart));
    ASSERT_TRUE(state.update_imu(kRosStart, kRosStart, kSteadyStart));
}

TEST(SafetyState, StartsDisabledAndRejectsEarlyEnable)
{
    SafetyState state;
    std::string reason;
    EXPECT_FALSE(state.request_enable(kRosStart, kSteadyStart, &reason));
    EXPECT_EQ(reason, "diagnostics_missing");
    const auto decision = state.evaluate(kRosStart, kSteadyStart);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_FALSE(decision.snapshot.ready);
    EXPECT_FALSE(decision.command.has_value());
}

TEST(SafetyState, RequiresACommandReceivedAfterEnable)
{
    SafetyState state;
    EXPECT_FALSE(state.update_command(0.2, 0.1, kSteadyStart));
    make_ready(state);
    ASSERT_TRUE(state.request_enable(kRosStart, kSteadyStart));

    auto decision = state.evaluate(kRosStart, kSteadyStart + 10 * kMs);
    EXPECT_TRUE(decision.snapshot.enabled);
    EXPECT_FALSE(decision.command.has_value());
    EXPECT_EQ(decision.snapshot.block_reason, "command_pending");

    ASSERT_TRUE(state.update_command(0.2, -0.1, kSteadyStart + 20 * kMs));
    decision = state.evaluate(kRosStart + 20 * kMs, kSteadyStart + 20 * kMs);
    ASSERT_TRUE(decision.command.has_value());
    EXPECT_DOUBLE_EQ(decision.command->linear_x, 0.2);
    EXPECT_DOUBLE_EQ(decision.command->angular_z, -0.1);
}

TEST(SafetyState, InvalidCommandDisarms)
{
    SafetyState state;
    make_ready(state);
    ASSERT_TRUE(state.request_enable(kRosStart, kSteadyStart));
    EXPECT_FALSE(state.update_command(std::nan(""), 0.0, kSteadyStart + 10 * kMs));
    const auto decision = state.evaluate(kRosStart + 10 * kMs, kSteadyStart + 10 * kMs);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_EQ(decision.snapshot.block_reason, "operator_disabled");
    EXPECT_EQ(decision.snapshot.last_trip_reason, "command_not_finite");
}

TEST(SafetyState, CommandLimitsDisarm)
{
    SafetyState state{SafetyConfig{200 * kMs, 500 * kMs, 250 * kMs, 100 * kMs, 0.5, 1.0}};
    make_ready(state);
    ASSERT_TRUE(state.request_enable(kRosStart, kSteadyStart));
    EXPECT_FALSE(state.update_command(0.51, 0.0, kSteadyStart + 10 * kMs));
    const auto decision = state.evaluate(kRosStart + 10 * kMs, kSteadyStart + 10 * kMs);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_EQ(decision.snapshot.last_trip_reason, "command_limit_exceeded");
    EXPECT_EQ(decision.snapshot.trip_count, 1U);
}

TEST(SafetyState, ImuDiagnosticHealthIsRequired)
{
    SafetyState state;
    state.update_diagnostics(true, true, false, true, kSteadyStart);
    ASSERT_TRUE(state.update_odom(kRosStart, kRosStart, kSteadyStart));
    ASSERT_TRUE(state.update_imu(kRosStart, kRosStart, kSteadyStart));
    const auto decision = state.evaluate(kRosStart, kSteadyStart);
    EXPECT_FALSE(decision.snapshot.ready);
    EXPECT_EQ(decision.snapshot.block_reason, "imu_not_ready");
}

TEST(SafetyState, CommandTimeoutDisarmsAndDoesNotReplay)
{
    SafetyState state;
    make_ready(state);
    ASSERT_TRUE(state.request_enable(kRosStart, kSteadyStart));
    ASSERT_TRUE(state.update_command(0.3, 0.0, kSteadyStart));

    state.update_diagnostics(true, true, true, true, kSteadyStart + 200 * kMs);
    ASSERT_TRUE(
        state.update_odom(kRosStart + 200 * kMs, kRosStart + 200 * kMs, kSteadyStart + 200 * kMs));
    ASSERT_TRUE(
        state.update_imu(kRosStart + 200 * kMs, kRosStart + 200 * kMs, kSteadyStart + 200 * kMs));
    auto decision = state.evaluate(kRosStart + 251 * kMs, kSteadyStart + 251 * kMs);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_FALSE(decision.command.has_value());
    EXPECT_EQ(decision.snapshot.block_reason, "operator_disabled");
    EXPECT_EQ(decision.snapshot.last_trip_reason, "command_timeout");

    state.update_diagnostics(true, true, true, true, kSteadyStart + 260 * kMs);
    ASSERT_TRUE(
        state.update_odom(kRosStart + 260 * kMs, kRosStart + 260 * kMs, kSteadyStart + 260 * kMs));
    ASSERT_TRUE(
        state.update_imu(kRosStart + 260 * kMs, kRosStart + 260 * kMs, kSteadyStart + 260 * kMs));
    decision = state.evaluate(kRosStart + 260 * kMs, kSteadyStart + 260 * kMs);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_FALSE(decision.command.has_value());
}

TEST(SafetyState, HealthLossLatchesDisabledAcrossRecovery)
{
    SafetyState state;
    make_ready(state);
    ASSERT_TRUE(state.request_enable(kRosStart, kSteadyStart));
    ASSERT_TRUE(state.update_command(0.1, 0.0, kSteadyStart));

    state.update_diagnostics(false, true, true, true, kSteadyStart + 10 * kMs);
    auto decision = state.evaluate(kRosStart + 10 * kMs, kSteadyStart + 10 * kMs);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_EQ(decision.snapshot.block_reason, "transport_not_ready");
    EXPECT_EQ(decision.snapshot.last_trip_reason, "transport_not_ready");

    state.update_diagnostics(true, true, true, true, kSteadyStart + 20 * kMs);
    ASSERT_TRUE(
        state.update_odom(kRosStart + 20 * kMs, kRosStart + 20 * kMs, kSteadyStart + 20 * kMs));
    ASSERT_TRUE(
        state.update_imu(kRosStart + 20 * kMs, kRosStart + 20 * kMs, kSteadyStart + 20 * kMs));
    decision = state.evaluate(kRosStart + 20 * kMs, kSteadyStart + 20 * kMs);
    EXPECT_TRUE(decision.snapshot.ready);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_FALSE(decision.command.has_value());
    EXPECT_EQ(decision.snapshot.block_reason, "operator_disabled");
    EXPECT_EQ(decision.snapshot.last_trip_reason, "transport_not_ready");
    EXPECT_EQ(decision.snapshot.trip_count, 1U);
}

TEST(SafetyState, RejectsZeroFutureStaleAndNonMonotonicStamps)
{
    SafetyState state;
    EXPECT_FALSE(state.update_odom(0, kRosStart, kSteadyStart));
    EXPECT_FALSE(state.update_odom(kRosStart + 101 * kMs, kRosStart, kSteadyStart));
    EXPECT_FALSE(state.update_odom(kRosStart - 201 * kMs, kRosStart, kSteadyStart));
    ASSERT_TRUE(state.update_odom(kRosStart, kRosStart, kSteadyStart));
    EXPECT_FALSE(state.update_odom(kRosStart, kRosStart + 1 * kMs, kSteadyStart + 1 * kMs));
}

TEST(SafetyState, SensorAndDiagnosticsReceiveTimeoutsBlockReadiness)
{
    SafetyState state{SafetyConfig{200 * kMs, 500 * kMs, 250 * kMs, 100 * kMs, 1.0, 2.0}};
    make_ready(state);
    auto decision = state.evaluate(kRosStart + 201 * kMs, kSteadyStart + 201 * kMs);
    EXPECT_FALSE(decision.snapshot.ready);
    EXPECT_EQ(decision.snapshot.block_reason, "odom_receive_timeout");

    state.update_diagnostics(true, true, true, true, kSteadyStart);
    ASSERT_TRUE(
        state.update_odom(kRosStart + 400 * kMs, kRosStart + 400 * kMs, kSteadyStart + 400 * kMs));
    ASSERT_TRUE(
        state.update_imu(kRosStart + 400 * kMs, kRosStart + 400 * kMs, kSteadyStart + 400 * kMs));
    decision = state.evaluate(kRosStart + 501 * kMs, kSteadyStart + 501 * kMs);
    EXPECT_FALSE(decision.snapshot.ready);
    EXPECT_EQ(decision.snapshot.block_reason, "diagnostics_timeout");
}

TEST(SafetyState, ExplicitDisableAlwaysClearsCommand)
{
    SafetyState state;
    make_ready(state);
    ASSERT_TRUE(state.request_enable(kRosStart, kSteadyStart));
    ASSERT_TRUE(state.update_command(0.3, 0.2, kSteadyStart));
    state.request_disable();
    const auto decision = state.evaluate(kRosStart, kSteadyStart);
    EXPECT_FALSE(decision.snapshot.enabled);
    EXPECT_FALSE(decision.command.has_value());
    EXPECT_EQ(decision.snapshot.block_reason, "operator_disabled");
    EXPECT_EQ(decision.snapshot.last_trip_reason, "operator_disabled");
}

} // namespace
