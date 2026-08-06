#ifndef VC_SAFETY__SAFETY_GATE_NODE_HPP_
#define VC_SAFETY__SAFETY_GATE_NODE_HPP_

#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>

namespace vc_safety
{

/// Lifecycle wrapper that publishes the fail-closed SafetyState decision.
class SafetyGateNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
    /// Create the component with relative topic and service names.
    explicit SafetyGateNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~SafetyGateNode() override;
    void emergency_stop(const std::string& reason = "process_shutdown");

  protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace vc_safety

#endif // VC_SAFETY__SAFETY_GATE_NODE_HPP_
