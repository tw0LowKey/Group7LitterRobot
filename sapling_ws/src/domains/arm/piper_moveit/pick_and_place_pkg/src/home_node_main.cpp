#include <rclcpp/rclcpp.hpp>
#include "home_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("home_server_node", node_options);

    // CRITICAL: We pass "arm" here because we are controlling the main arm group
    HomeControl home_control(node, "arm");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}