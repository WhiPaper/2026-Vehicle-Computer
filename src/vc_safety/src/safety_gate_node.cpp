#include "vc_safety/safety_gate_node.hpp"

#include "vc_safety/ecu_diagnostics.hpp"
#include "vc_safety/input_validation.hpp"
#include "vc_safety/safety_state.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace vc_safety
{
namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

std::int64_t steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

diagnostic_msgs::msg::KeyValue key_value(const std::string& key, const std::string& value)
{
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
}

rcl_interfaces::msg::ParameterDescriptor positive_double_descriptor(const std::string& description,
                                                                    bool read_only = true)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = read_only;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.001;
    range.to_value = 1000.0;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor positive_integer_descriptor(const std::string& description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 60000;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    return descriptor;
}

std::string gid_text(const rclcpp::MessageInfo& info)
{
    const auto& gid = info.get_rmw_message_info().publisher_gid;
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto value : gid.data)
    {
        stream << std::setw(2) << static_cast<unsigned int>(value);
    }
    return stream.str();
}

} // namespace

class SafetyGateNode::Impl
{
  public:
    explicit Impl(SafetyGateNode* node) : node_(node)
    {
        node_->declare_parameter<double>(
            "publish_rate_hz", 20.0,
            positive_double_descriptor("ECU command publication rate in Hz"));
        node_->declare_parameter<double>(
            "status_rate_hz", 5.0,
            positive_double_descriptor("Safety diagnostic publication rate in Hz"));
        node_->declare_parameter<int>(
            "data_timeout_ms", 200,
            positive_integer_descriptor("Maximum odometry and IMU age in ms"));
        node_->declare_parameter<int>(
            "diagnostics_timeout_ms", 500,
            positive_integer_descriptor("Maximum ECU diagnostics receive gap in ms"));
        node_->declare_parameter<int>(
            "command_timeout_ms", 250,
            positive_integer_descriptor("Maximum command request receive gap in ms"));
        node_->declare_parameter<int>(
            "future_tolerance_ms", 100,
            positive_integer_descriptor("Maximum future sensor timestamp offset in ms"));
        node_->declare_parameter<double>(
            "max_linear_speed_mps", 1.0,
            positive_double_descriptor("Maximum accepted absolute linear.x"));
        node_->declare_parameter<double>(
            "max_angular_speed_rps", 2.0,
            positive_double_descriptor("Maximum accepted absolute angular.z"));
        node_->declare_parameter<std::string>(
            "odom_frame", "odom",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Expected raw odometry parent frame")
                .set__read_only(true));
        node_->declare_parameter<std::string>(
            "base_frame", "base_link",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Expected raw odometry child frame")
                .set__read_only(true));
        node_->declare_parameter<std::string>("imu_frame", "imu_link",
                                              rcl_interfaces::msg::ParameterDescriptor()
                                                  .set__description("Expected raw IMU frame")
                                                  .set__read_only(true));
        node_->declare_parameter<bool>(
            "enable_topic_statistics", true,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Publish ROS 2 message period/age statistics")
                .set__read_only(true));
        node_->declare_parameter<std::string>(
            "statistics_topic", "vehicle/safety/statistics",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Relative topic used for subscription statistics")
                .set__read_only(true));
    }

    CallbackReturn configure()
    {
        try
        {
            state_ = SafetyState(load_config());
            odom_gid_.clear();
            imu_gid_.clear();
            diagnostics_gid_.clear();
            command_gid_.clear();
            ecu_writer_generation_ = 0;
            qos_incompatibility_count_ = 0;
            message_lost_count_ = 0;
            create_endpoints();
            RCLCPP_INFO(node_->get_logger(), "Safety gate configured");
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(node_->get_logger(), "Configuration failed: %s", error.what());
            return CallbackReturn::FAILURE;
        }
    }

    CallbackReturn activate()
    {
        state_ = SafetyState(load_config());
        odom_gid_.clear();
        imu_gid_.clear();
        diagnostics_gid_.clear();
        command_gid_.clear();
        ecu_writer_generation_ = 0;
        command_publisher_->on_activate();
        enabled_publisher_->on_activate();
        diagnostics_publisher_->on_activate();
        motion_timer_->reset();
        status_timer_->reset();
        publish_zero();
        publish_enabled(false);
        RCLCPP_INFO(node_->get_logger(), "Safety gate active and motion disabled");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn deactivate(const std::string& reason)
    {
        state_.request_disable(reason);
        publish_zero();
        publish_enabled(false);
        motion_timer_->cancel();
        status_timer_->cancel();
        diagnostics_publisher_->on_deactivate();
        enabled_publisher_->on_deactivate();
        command_publisher_->on_deactivate();
        RCLCPP_INFO(node_->get_logger(), "Safety gate inactive: %s", reason.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn cleanup()
    {
        state_.request_disable("lifecycle_cleanup");
        motion_timer_.reset();
        status_timer_.reset();
        enable_service_.reset();
        command_subscription_.reset();
        odom_subscription_.reset();
        imu_subscription_.reset();
        ecu_diagnostics_subscription_.reset();
        command_publisher_.reset();
        enabled_publisher_.reset();
        diagnostics_publisher_.reset();
        odom_gid_.clear();
        imu_gid_.clear();
        diagnostics_gid_.clear();
        command_gid_.clear();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn shutdown(const std::string& reason)
    {
        if (is_active())
        {
            deactivate(reason);
        }
        else
        {
            state_.request_disable(reason);
        }
        return CallbackReturn::SUCCESS;
    }

    void emergency_stop(const std::string& reason)
    {
        state_.request_disable(reason);
        try
        {
            publish_zero();
            publish_enabled(false);
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(node_->get_logger(), "Emergency stop publication failed: %s",
                         error.what());
        }
    }

  private:
    bool is_active() const
    {
        return node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    }

    SafetyConfig load_config() const
    {
        const auto milliseconds = [this](const char* name)
        {
            const auto value = node_->get_parameter(name).as_int();
            if (value <= 0)
            {
                throw std::invalid_argument(std::string{name} + " must be positive");
            }
            return value * 1000000LL;
        };
        const double publish_rate = node_->get_parameter("publish_rate_hz").as_double();
        const double status_rate = node_->get_parameter("status_rate_hz").as_double();
        if (publish_rate <= 0.0 || status_rate <= 0.0)
        {
            throw std::invalid_argument("publish rates must be positive");
        }
        return SafetyConfig{milliseconds("data_timeout_ms"),
                            milliseconds("diagnostics_timeout_ms"),
                            milliseconds("command_timeout_ms"),
                            milliseconds("future_tolerance_ms"),
                            node_->get_parameter("max_linear_speed_mps").as_double(),
                            node_->get_parameter("max_angular_speed_rps").as_double()};
    }

    rclcpp::SubscriptionOptions subscription_options(const std::string& name)
    {
        rclcpp::SubscriptionOptions options;
        options.event_callbacks.incompatible_qos_callback =
            [this, name](rclcpp::QOSRequestedIncompatibleQoSInfo&)
        {
            ++qos_incompatibility_count_;
            disarm("qos_incompatible_" + name);
        };
        options.event_callbacks.message_lost_callback = [this, name](rclcpp::QOSMessageLostInfo&)
        {
            ++message_lost_count_;
            disarm("message_lost_" + name);
        };
        options.event_callbacks.matched_callback = [this, name](rclcpp::MatchedInfo& info)
        {
            if (info.current_count == 0)
            {
                disarm("publisher_unmatched_" + name);
            }
        };
        if (node_->get_parameter("enable_topic_statistics").as_bool())
        {
            options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
            options.topic_stats_options.publish_topic =
                node_->get_parameter("statistics_topic").as_string();
            options.topic_stats_options.publish_period = std::chrono::seconds(1);
        }
        return options;
    }

    void create_endpoints()
    {
        const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        const auto sensor_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        command_publisher_ =
            node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", reliable_qos);
        enabled_publisher_ =
            node_->create_publisher<std_msgs::msg::Bool>("vehicle/motion_enabled", latched_qos);
        diagnostics_publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "vehicle/safety/diagnostics", reliable_qos);

        command_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_request", reliable_qos,
            std::bind(&Impl::on_command, this, std::placeholders::_1, std::placeholders::_2),
            subscription_options("command"));
        odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom", sensor_qos,
            std::bind(&Impl::on_odom, this, std::placeholders::_1, std::placeholders::_2),
            subscription_options("odom"));
        imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", sensor_qos,
            std::bind(&Impl::on_imu, this, std::placeholders::_1, std::placeholders::_2),
            subscription_options("imu"));
        ecu_diagnostics_subscription_ =
            node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                "diagnostics", reliable_qos,
                std::bind(&Impl::on_ecu_diagnostics, this, std::placeholders::_1,
                          std::placeholders::_2),
                subscription_options("diagnostics"));

        enable_service_ = node_->create_service<std_srvs::srv::SetBool>(
            "vehicle/motion_enable",
            std::bind(&Impl::on_enable, this, std::placeholders::_1, std::placeholders::_2));

        const double publish_rate = node_->get_parameter("publish_rate_hz").as_double();
        const double status_rate = node_->get_parameter("status_rate_hz").as_double();
        motion_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                                                 std::bind(&Impl::publish_motion, this));
        status_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / status_rate),
                                                 std::bind(&Impl::publish_status, this));
        motion_timer_->cancel();
        status_timer_->cancel();
    }

    std::int64_t ros_now_ns() const { return node_->get_clock()->now().nanoseconds(); }

    void disarm(const std::string& reason)
    {
        state_.request_disable(reason);
        if (is_active())
        {
            publish_zero();
            publish_enabled(false);
        }
    }

    bool observe_writer(std::string& current_gid, const rclcpp::MessageInfo& info,
                        const std::string& stream)
    {
        const auto incoming_gid = gid_text(info);
        if (current_gid.empty())
        {
            current_gid = incoming_gid;
            if (ecu_writer_generation_ == 0)
            {
                ecu_writer_generation_ = 1;
            }
            return true;
        }
        if (incoming_gid == current_gid)
        {
            return true;
        }
        current_gid = incoming_gid;
        ++ecu_writer_generation_;
        disarm("ecu_publisher_changed_" + stream);
        return false;
    }

    void on_command(const geometry_msgs::msg::Twist::SharedPtr message,
                    const rclcpp::MessageInfo& info)
    {
        if (!is_active())
        {
            return;
        }
        const auto validation =
            validate_command(*message, node_->get_parameter("max_linear_speed_mps").as_double(),
                             node_->get_parameter("max_angular_speed_rps").as_double());
        if (!validation)
        {
            state_.reject_input(validation.reason);
            publish_zero();
            return;
        }
        const auto incoming_gid = gid_text(info);
        if (!command_gid_.empty() && incoming_gid != command_gid_)
        {
            disarm("command_publisher_changed");
            return;
        }
        if (!state_.update_command(message->linear.x, message->angular.z, steady_now_ns()))
        {
            publish_zero();
            return;
        }
        if (command_gid_.empty())
        {
            command_gid_ = incoming_gid;
        }
    }

    void on_odom(const nav_msgs::msg::Odometry::SharedPtr message, const rclcpp::MessageInfo& info)
    {
        if (!is_active())
        {
            return;
        }
        if (!observe_writer(odom_gid_, info, "odom"))
        {
            return;
        }
        const auto validation =
            validate_odometry(*message, node_->get_parameter("odom_frame").as_string(),
                              node_->get_parameter("base_frame").as_string());
        if (!validation)
        {
            state_.reject_input(validation.reason);
            publish_zero();
            return;
        }
        const rclcpp::Time stamp{message->header.stamp};
        if (!state_.update_odom(stamp.nanoseconds(), ros_now_ns(), steady_now_ns()))
        {
            publish_zero();
        }
    }

    void on_imu(const sensor_msgs::msg::Imu::SharedPtr message, const rclcpp::MessageInfo& info)
    {
        if (!is_active())
        {
            return;
        }
        if (!observe_writer(imu_gid_, info, "imu"))
        {
            return;
        }
        const auto validation =
            validate_imu(*message, node_->get_parameter("imu_frame").as_string());
        if (!validation)
        {
            state_.reject_input(validation.reason);
            publish_zero();
            return;
        }
        const rclcpp::Time stamp{message->header.stamp};
        if (!state_.update_imu(stamp.nanoseconds(), ros_now_ns(), steady_now_ns()))
        {
            publish_zero();
        }
    }

    void on_ecu_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message,
                            const rclcpp::MessageInfo& info)
    {
        if (!is_active())
        {
            return;
        }
        if (!observe_writer(diagnostics_gid_, info, "diagnostics"))
        {
            return;
        }

        const auto health = evaluate_ecu_diagnostics(*message);
        state_.update_diagnostics(health.transport_ok, health.drive_ok, health.imu_ok,
                                  health.time_synchronized, steady_now_ns());
        if (!health.transport_ok || !health.drive_ok || !health.imu_ok || !health.time_synchronized)
        {
            publish_zero();
        }
    }

    void on_enable(const std_srvs::srv::SetBool::Request::SharedPtr request,
                   std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        if (!request->data)
        {
            state_.request_disable();
            if (is_active())
            {
                publish_zero();
                publish_enabled(false);
            }
            response->success = true;
            response->message = "motion disabled";
            return;
        }
        if (!is_active())
        {
            response->success = false;
            response->message = "lifecycle_not_active";
            return;
        }

        std::string reason;
        response->success = state_.request_enable(ros_now_ns(), steady_now_ns(), &reason);
        response->message = reason;
        if (response->success)
        {
            command_gid_.clear();
        }
        publish_zero();
        publish_enabled(response->success);
    }

    void publish_zero()
    {
        if (command_publisher_ && command_publisher_->is_activated())
        {
            command_publisher_->publish(geometry_msgs::msg::Twist{});
        }
    }

    void publish_enabled(bool enabled)
    {
        if (!enabled_publisher_ || !enabled_publisher_->is_activated())
        {
            return;
        }
        std_msgs::msg::Bool message;
        message.data = enabled;
        enabled_publisher_->publish(message);
    }

    void publish_motion()
    {
        if (!is_active())
        {
            return;
        }
        const auto decision = state_.evaluate(ros_now_ns(), steady_now_ns());
        geometry_msgs::msg::Twist message;
        if (decision.command.has_value())
        {
            message.linear.x = decision.command->linear_x;
            message.angular.z = decision.command->angular_z;
        }
        command_publisher_->publish(message);
    }

    void publish_status()
    {
        if (!is_active())
        {
            return;
        }
        const auto decision = state_.evaluate(ros_now_ns(), steady_now_ns());
        publish_enabled(decision.snapshot.enabled);

        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = node_->get_clock()->now();
        DiagnosticStatus status;
        status.name = "vehicle_computer/safety_gate";
        status.hardware_id = "rpi5-vehicle-computer";
        if (!decision.snapshot.ready)
        {
            status.level = DiagnosticStatus::ERROR;
            status.message = "motion blocked";
        }
        else if (!decision.snapshot.enabled || !decision.command.has_value())
        {
            status.level = DiagnosticStatus::WARN;
            status.message = "ready but not commanding";
        }
        else
        {
            status.level = DiagnosticStatus::OK;
            status.message = "motion enabled";
        }

        const auto bool_text = [](bool value) { return value ? "true" : "false"; };
        status.values = {
            key_value("enabled", bool_text(decision.snapshot.enabled)),
            key_value("ready", bool_text(decision.snapshot.ready)),
            key_value("block_reason", decision.snapshot.block_reason),
            key_value("last_trip_reason", decision.snapshot.last_trip_reason),
            key_value("trip_count", std::to_string(decision.snapshot.trip_count)),
            key_value("transport_ok", bool_text(decision.snapshot.transport_ok)),
            key_value("drive_ok", bool_text(decision.snapshot.drive_ok)),
            key_value("imu_ok", bool_text(decision.snapshot.imu_ok)),
            key_value("time_synchronized", bool_text(decision.snapshot.time_synchronized)),
            key_value("odom_age_ms", std::to_string(decision.snapshot.odom_age_ms)),
            key_value("imu_age_ms", std::to_string(decision.snapshot.imu_age_ms)),
            key_value("command_age_ms", std::to_string(decision.snapshot.command_age_ms)),
            key_value("lifecycle_state", node_->get_current_state().label()),
            key_value("ecu_publisher_generation", std::to_string(ecu_writer_generation_)),
            key_value("command_owner_bound",
                      bool_text(decision.snapshot.enabled && !command_gid_.empty())),
            key_value("qos_incompatibility_count", std::to_string(qos_incompatibility_count_)),
            key_value("message_lost_count", std::to_string(message_lost_count_))};
        array.status.push_back(std::move(status));
        diagnostics_publisher_->publish(array);
    }

    SafetyGateNode* node_;
    SafetyState state_;
    std::string odom_gid_;
    std::string imu_gid_;
    std::string diagnostics_gid_;
    std::string command_gid_;
    std::uint64_t ecu_writer_generation_{0};
    std::uint64_t qos_incompatibility_count_{0};
    std::uint64_t message_lost_count_{0};

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr enabled_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostics_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        ecu_diagnostics_subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::TimerBase::SharedPtr motion_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

SafetyGateNode::SafetyGateNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("safety_gate", options), impl_(std::make_unique<Impl>(this))
{
}

SafetyGateNode::~SafetyGateNode() { impl_->emergency_stop("node_destructor"); }

void SafetyGateNode::emergency_stop(const std::string& reason) { impl_->emergency_stop(reason); }

CallbackReturn SafetyGateNode::on_configure(const rclcpp_lifecycle::State&)
{
    return impl_->configure();
}

CallbackReturn SafetyGateNode::on_activate(const rclcpp_lifecycle::State&)
{
    return impl_->activate();
}

CallbackReturn SafetyGateNode::on_deactivate(const rclcpp_lifecycle::State&)
{
    return impl_->deactivate("lifecycle_deactivate");
}

CallbackReturn SafetyGateNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    return impl_->cleanup();
}

CallbackReturn SafetyGateNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    return impl_->shutdown("lifecycle_shutdown");
}

CallbackReturn SafetyGateNode::on_error(const rclcpp_lifecycle::State&)
{
    impl_->shutdown("lifecycle_error");
    impl_->cleanup();
    return CallbackReturn::SUCCESS;
}

} // namespace vc_safety

RCLCPP_COMPONENTS_REGISTER_NODE(vc_safety::SafetyGateNode)
