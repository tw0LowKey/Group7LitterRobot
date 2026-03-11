#include <rclcpp/rclcpp.hpp>
#include "gripper_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("gripper_server_node", node_options);

    // CRITICAL: We pass "gripper" here instead of "arm"
    GripperControl gripper_control(node, "gripper");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}