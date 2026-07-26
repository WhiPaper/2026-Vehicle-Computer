#include "vc_safety/input_validation.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{

TEST(InputValidation, AcceptsPlanarCommandWithinLimits)
{
    geometry_msgs::msg::Twist command;
    command.linear.x = 0.5;
    command.angular.z = -1.0;
    EXPECT_TRUE(vc_safety::validate_command(command, 0.5, 1.0));
}

TEST(InputValidation, RejectsNonFiniteUnsupportedAndExcessiveCommands)
{
    geometry_msgs::msg::Twist command;
    command.angular.z = std::numeric_limits<double>::infinity();
    EXPECT_EQ(vc_safety::validate_command(command, 1.0, 2.0).reason, "command_not_finite");

    command.angular.z = 0.0;
    command.linear.y = 0.1;
    EXPECT_EQ(vc_safety::validate_command(command, 1.0, 2.0).reason, "command_unsupported_axis");

    command.linear.y = 0.0;
    command.linear.x = 1.01;
    EXPECT_EQ(vc_safety::validate_command(command, 1.0, 2.0).reason, "command_limit_exceeded");
}

TEST(InputValidation, EnforcesSensorFramesAndFiniteFusedFields)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    EXPECT_TRUE(vc_safety::validate_odometry(odometry, "odom", "base_link"));
    odometry.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(vc_safety::validate_odometry(odometry, "odom", "base_link").reason,
              "odom_payload_invalid");

    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = "wrong";
    EXPECT_EQ(vc_safety::validate_imu(imu, "imu_link").reason, "imu_frame_invalid");
}

} // namespace
