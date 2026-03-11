#include <rclcpp/rclcpp.hpp>
#include "pick_and_place_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // MoveIt 2 strictly requires this parameter option
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("pick_and_place_server_node", node_options);

    // Instantiate the class
    PickAndPlace pick_place(node, "arm");

    // We must use a MultiThreadedExecutor so MoveIt can process trajectories 
    // while the service server is simultaneously running.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // Keep the node alive forever

    rclcpp::shutdown();
    return 0;
}
