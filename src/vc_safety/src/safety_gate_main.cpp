#include "vc_safety/safety_gate_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<vc_safety::SafetyGateNode>();
    std::weak_ptr<vc_safety::SafetyGateNode> weak_node = node;
    rclcpp::on_shutdown(
        [weak_node]()
        {
            if (const auto safety_gate = weak_node.lock())
            {
                safety_gate->emergency_stop("process_shutdown");
            }
        });
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    node->emergency_stop("executor_stopped");
    executor.remove_node(node->get_node_base_interface());
    node.reset();
    rclcpp::shutdown();
    return 0;
}
