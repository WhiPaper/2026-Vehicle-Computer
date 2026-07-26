#include "vc_safety/input_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace vc_safety
{
namespace
{

template <typename Container> bool all_finite(const Container& values)
{
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

} // namespace

ValidationResult validate_command(const geometry_msgs::msg::Twist& command,
                                  double max_linear_speed_mps, double max_angular_speed_rps)
{
    const std::array<double, 6> values = {command.linear.x,  command.linear.y,  command.linear.z,
                                          command.angular.x, command.angular.y, command.angular.z};
    if (!all_finite(values))
    {
        return {false, "command_not_finite"};
    }

    constexpr double epsilon = 1.0e-12;
    if (std::abs(command.linear.y) > epsilon || std::abs(command.linear.z) > epsilon ||
        std::abs(command.angular.x) > epsilon || std::abs(command.angular.y) > epsilon)
    {
        return {false, "command_unsupported_axis"};
    }

    if (std::abs(command.linear.x) > max_linear_speed_mps ||
        std::abs(command.angular.z) > max_angular_speed_rps)
    {
        return {false, "command_limit_exceeded"};
    }
    return {true, "none"};
}

ValidationResult validate_odometry(const nav_msgs::msg::Odometry& odometry,
                                   const std::string& expected_parent_frame,
                                   const std::string& expected_child_frame)
{
    if (odometry.header.frame_id != expected_parent_frame ||
        odometry.child_frame_id != expected_child_frame)
    {
        return {false, "odom_frame_invalid"};
    }
    if (!std::isfinite(odometry.twist.twist.linear.x) ||
        !std::isfinite(odometry.twist.twist.angular.z) || !all_finite(odometry.twist.covariance))
    {
        return {false, "odom_payload_invalid"};
    }
    return {true, "none"};
}

ValidationResult validate_imu(const sensor_msgs::msg::Imu& imu, const std::string& expected_frame)
{
    if (imu.header.frame_id != expected_frame)
    {
        return {false, "imu_frame_invalid"};
    }
    if (!std::isfinite(imu.angular_velocity.z) || !all_finite(imu.angular_velocity_covariance))
    {
        return {false, "imu_payload_invalid"};
    }
    return {true, "none"};
}

} // namespace vc_safety
