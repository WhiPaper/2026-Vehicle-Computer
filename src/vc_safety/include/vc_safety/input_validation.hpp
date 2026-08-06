#ifndef VC_SAFETY__INPUT_VALIDATION_HPP_
#define VC_SAFETY__INPUT_VALIDATION_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

namespace vc_safety
{

/// Validation result with a stable diagnostic reason for rejected input.
struct ValidationResult
{
    bool valid;
    std::string reason;

    explicit operator bool() const { return valid; }
};

/// Validate supported Twist axes, finite values, and configured speed limits.
ValidationResult validate_command(const geometry_msgs::msg::Twist& command,
                                  double max_linear_speed_mps, double max_angular_speed_rps);

/// Validate odometry frames and finite payload values.
ValidationResult validate_odometry(const nav_msgs::msg::Odometry& odometry,
                                   const std::string& expected_parent_frame,
                                   const std::string& expected_child_frame);

/// Validate the IMU frame and finite payload values.
ValidationResult validate_imu(const sensor_msgs::msg::Imu& imu, const std::string& expected_frame);

} // namespace vc_safety

#endif // VC_SAFETY__INPUT_VALIDATION_HPP_
