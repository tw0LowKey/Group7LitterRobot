#include "home_node.hpp"

HomeControl::HomeControl(rclcpp::Node::SharedPtr node, const std::string& group_name) : node_(node)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

    move_group_->setPlanningTime(15.0);
    move_group_->setNumPlanningAttempts(50);
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    service_ = node_->create_service<piper_msgs::srv::MoveToHome>(
        "move_to_home",
        std::bind(&HomeControl::handle_service_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(node_->get_logger(), "Home node initialized. Service '/move_to_home' is ready.");
}

void HomeControl::handle_service_request(
    const std::shared_ptr<piper_msgs::srv::MoveToHome::Request> request,
    std::shared_ptr<piper_msgs::srv::MoveToHome::Response> response)
{
    // We cast request to void to tell the compiler to ignore it, since it's empty
    (void)request; 
    
    RCLCPP_INFO(node_->get_logger(), "Return home command received.");
    response->success = return_home();
}

bool HomeControl::return_home()
{
    move_group_->setNamedTarget("zero");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Home plan succeeded. Moving to 'zero'...");
        move_group_->move();
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Home planning failed. Does the 'zero' named target exist in the SRDF?");
    }

    return success;
}
